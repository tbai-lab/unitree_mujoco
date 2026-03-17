#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/idl/ros2/PointCloud2_.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <csignal>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// Minimal PNG writer (uncompressed, grayscale 8-bit)
// Avoids external dependencies.  Produces valid but uncompressed PNGs.
// ---------------------------------------------------------------------------
namespace minipng
{

static void putBE32(std::vector<uint8_t> &out, uint32_t v)
{
    out.push_back((v >> 24) & 0xFF);
    out.push_back((v >> 16) & 0xFF);
    out.push_back((v >> 8) & 0xFF);
    out.push_back(v & 0xFF);
}

static uint32_t crc32(const uint8_t *data, size_t len)
{
    uint32_t c = 0xFFFFFFFF;
    for (size_t i = 0; i < len; ++i)
    {
        c ^= data[i];
        for (int j = 0; j < 8; ++j)
            c = (c >> 1) ^ (0xEDB88320 & (-(c & 1)));
    }
    return c ^ 0xFFFFFFFF;
}

static uint32_t adler32(const uint8_t *data, size_t len)
{
    uint32_t a = 1, b = 0;
    for (size_t i = 0; i < len; ++i)
    {
        a = (a + data[i]) % 65521;
        b = (b + a) % 65521;
    }
    return (b << 16) | a;
}

static void writeChunk(std::vector<uint8_t> &out, const char type[4],
                        const uint8_t *data, size_t len)
{
    putBE32(out, static_cast<uint32_t>(len));
    size_t start = out.size();
    out.insert(out.end(), type, type + 4);
    if (data && len)
        out.insert(out.end(), data, data + len);
    uint32_t c = crc32(out.data() + start, 4 + len);
    putBE32(out, c);
}

// Encode a W x H grayscale (8-bit) image to an uncompressed PNG.
static std::vector<uint8_t> encode(const uint8_t *pixels, int w, int h)
{
    std::vector<uint8_t> png;
    // Signature
    const uint8_t sig[] = {137, 80, 78, 71, 13, 10, 26, 10};
    png.insert(png.end(), sig, sig + 8);

    // IHDR
    std::vector<uint8_t> ihdr;
    putBE32(ihdr, w);
    putBE32(ihdr, h);
    ihdr.push_back(8); // bit depth
    ihdr.push_back(0); // grayscale
    ihdr.push_back(0); // compression
    ihdr.push_back(0); // filter
    ihdr.push_back(0); // interlace
    writeChunk(png, "IHDR", ihdr.data(), ihdr.size());

    // Build raw filtered data (filter byte 0 = None for each row)
    std::vector<uint8_t> raw;
    for (int y = 0; y < h; ++y)
    {
        raw.push_back(0); // filter: None
        raw.insert(raw.end(), pixels + y * w, pixels + (y + 1) * w);
    }

    // Wrap in zlib (uncompressed deflate)
    std::vector<uint8_t> zlib;
    zlib.push_back(0x78); // CMF
    zlib.push_back(0x01); // FLG
    // One uncompressed deflate block
    size_t rlen = raw.size();
    // Split into 65535-byte blocks
    size_t offset = 0;
    while (offset < rlen)
    {
        size_t block = std::min(rlen - offset, (size_t)65535);
        bool last = (offset + block >= rlen);
        zlib.push_back(last ? 0x01 : 0x00);
        uint16_t blen = static_cast<uint16_t>(block);
        uint16_t nlen = ~blen;
        zlib.push_back(blen & 0xFF);
        zlib.push_back((blen >> 8) & 0xFF);
        zlib.push_back(nlen & 0xFF);
        zlib.push_back((nlen >> 8) & 0xFF);
        zlib.insert(zlib.end(), raw.data() + offset, raw.data() + offset + block);
        offset += block;
    }
    uint32_t a = adler32(raw.data(), raw.size());
    putBE32(zlib, a);

    writeChunk(png, "IDAT", zlib.data(), zlib.size());
    writeChunk(png, "IEND", nullptr, 0);
    return png;
}

} // namespace minipng

// ---------------------------------------------------------------------------

static std::atomic<bool> running{true};
static std::mutex cloud_mtx;
static sensor_msgs::msg::dds_::PointCloud2_ latest_cloud;
static bool have_cloud = false;

void signalHandler(int) { running = false; }

void cloudCallback(const void *msg)
{
    auto *cloud = static_cast<const sensor_msgs::msg::dds_::PointCloud2_ *>(msg);
    std::lock_guard<std::mutex> lock(cloud_mtx);
    latest_cloud = *cloud;
    have_cloud = true;
}

int main(int argc, const char **argv)
{
    std::signal(SIGINT, signalHandler);

    std::string topic = "rt/pointcloud";
    int domain_id = 1;
    std::string interface = "lo";
    int image_width = 320;
    int image_height = 240;
    float max_depth_m = 10.0f; // depth values beyond this are clipped to white

    // Parse optional arguments
    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (arg == "--topic" && i + 1 < argc)
            topic = argv[++i];
        else if (arg == "--domain" && i + 1 < argc)
            domain_id = std::atoi(argv[++i]);
        else if (arg == "--interface" && i + 1 < argc)
            interface = argv[++i];
        else if (arg == "--width" && i + 1 < argc)
            image_width = std::atoi(argv[++i]);
        else if (arg == "--height" && i + 1 < argc)
            image_height = std::atoi(argv[++i]);
        else if (arg == "--max-depth" && i + 1 < argc)
            max_depth_m = std::atof(argv[++i]);
        else if (arg == "--help" || arg == "-h")
        {
            std::cout << "Usage: " << argv[0] << " [options]\n"
                      << "  --topic TOPIC        DDS topic (default: rt/pointcloud)\n"
                      << "  --domain ID          DDS domain ID (default: 1)\n"
                      << "  --interface IFACE    Network interface (default: lo)\n"
                      << "  --width W            Output depth image width (default: 320)\n"
                      << "  --height H           Output depth image height (default: 240)\n"
                      << "  --max-depth M        Max depth in meters for mapping to 255 (default: 10)\n";
            return 0;
        }
    }

    unitree::robot::ChannelFactory::Instance()->Init(domain_id, interface);

    unitree::robot::ChannelSubscriber<sensor_msgs::msg::dds_::PointCloud2_> sub(topic);
    sub.InitChannel(cloudCallback, 1);

    std::cout << "Subscribed to '" << topic << "' (domain=" << domain_id
              << ", interface=" << interface << ")" << std::endl;
    std::cout << "Depth images will be " << image_width << "x" << image_height
              << ", max depth " << max_depth_m << "m" << std::endl;
    std::cout << "Press Ctrl+C to stop." << std::endl;

    int saved_count = 0;

    while (running)
    {
        sensor_msgs::msg::dds_::PointCloud2_ cloud;
        bool got = false;
        {
            std::lock_guard<std::mutex> lock(cloud_mtx);
            if (have_cloud)
            {
                cloud = latest_cloud;
                have_cloud = false;
                got = true;
            }
        }

        if (!got)
        {
            usleep(50000); // 50ms poll
            continue;
        }

        uint32_t num_points = cloud.width();
        uint32_t pstep = cloud.point_step();
        const auto &data = cloud.data();

        if (num_points == 0 || pstep < 12)
        {
            usleep(50000);
            continue;
        }

        // Build a depth image by projecting points back to image plane.
        // We assume the point cloud is in camera frame (x right, y down, z forward)
        // and reconstruct pixel coords using a simple pinhole model.
        //
        // Since we don't know the exact camera intrinsics on the subscriber side,
        // we use the point cloud's min/max x,y extent to map into the image.
        // This gives a perspective-correct depth image regardless of the original
        // camera parameters.

        // First pass: find z range and angular extent
        float z_min = 1e9f, z_max = -1e9f;
        struct Point3
        {
            float x, y, z;
        };
        std::vector<Point3> points;
        points.reserve(num_points);

        for (uint32_t i = 0; i < num_points; ++i)
        {
            size_t off = i * pstep;
            if (off + 12 > data.size())
                break;
            Point3 p;
            std::memcpy(&p.x, data.data() + off + 0, 4);
            std::memcpy(&p.y, data.data() + off + 4, 4);
            std::memcpy(&p.z, data.data() + off + 8, 4);
            if (p.z <= 0.0f || !std::isfinite(p.z))
                continue;
            points.push_back(p);
            z_min = std::min(z_min, p.z);
            z_max = std::max(z_max, p.z);
        }

        if (points.empty())
        {
            usleep(50000);
            continue;
        }

        // Find angular extent for mapping to pixel coordinates
        float tan_max_x = 0, tan_max_y = 0;
        for (auto &p : points)
        {
            tan_max_x = std::max(tan_max_x, std::abs(p.x / p.z));
            tan_max_y = std::max(tan_max_y, std::abs(p.y / p.z));
        }
        // Add a small margin
        tan_max_x *= 1.05f;
        tan_max_y *= 1.05f;
        if (tan_max_x < 1e-6f)
            tan_max_x = 1.0f;
        if (tan_max_y < 1e-6f)
            tan_max_y = 1.0f;

        // Rasterize: project each point and write depth into image
        std::vector<float> depth_image(image_width * image_height, -1.0f);

        float cx = image_width / 2.0f;
        float cy = image_height / 2.0f;
        float fx = cx / tan_max_x;
        float fy = cy / tan_max_y;

        for (auto &p : points)
        {
            int u = static_cast<int>(p.x / p.z * fx + cx);
            int v = static_cast<int>(p.y / p.z * fy + cy);
            if (u < 0 || u >= image_width || v < 0 || v >= image_height)
                continue;
            int idx = v * image_width + u;
            // Keep nearest depth (z-buffer)
            if (depth_image[idx] < 0 || p.z < depth_image[idx])
                depth_image[idx] = p.z;
        }

        // Convert to grayscale (0 = near, 255 = far/empty)
        std::vector<uint8_t> pixels(image_width * image_height);
        for (int i = 0; i < image_width * image_height; ++i)
        {
            if (depth_image[i] < 0)
            {
                pixels[i] = 255; // no data
            }
            else
            {
                float normalized = std::min(depth_image[i] / max_depth_m, 1.0f);
                pixels[i] = static_cast<uint8_t>(normalized * 254.0f);
            }
        }

        // Encode and save PNG
        auto png = minipng::encode(pixels.data(), image_width, image_height);

        char filename[128];
        snprintf(filename, sizeof(filename), "depth_%04d.png", saved_count);

        std::ofstream file(filename, std::ios::binary);
        if (file.is_open())
        {
            file.write(reinterpret_cast<const char *>(png.data()), png.size());
            file.close();
            std::cout << "Saved " << filename << " (" << num_points << " points, "
                      << png.size() << " bytes, z=[" << z_min << ", " << z_max << "]m)" << std::endl;
        }
        else
        {
            std::cerr << "Failed to open " << filename << " for writing" << std::endl;
        }

        ++saved_count;
        usleep(100000); // 100ms between saves
    }

    std::cout << "\nSaved " << saved_count << " depth images." << std::endl;
    return 0;
}
