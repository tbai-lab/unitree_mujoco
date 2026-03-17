#include <unitree/robot/go2/video/video_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <iostream>
#include <fstream>
#include <ctime>
#include <csignal>
#include <atomic>
#include <sys/stat.h>

static std::atomic<bool> running{true};

void signalHandler(int) { running = false; }

int main(int argc, const char **argv)
{
    std::signal(SIGINT, signalHandler);

    // Domain 1 + loopback for simulation, or pass interface as argv[1]
    if (argc < 2)
        unitree::robot::ChannelFactory::Instance()->Init(1, "lo");
    else
        unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);

    // Create output directory for frames
    const std::string output_dir = "franka_frames";
    mkdir(output_dir.c_str(), 0755);

    unitree::robot::go2::VideoClient video_client;
    video_client.SetTimeout(2.0f);
    video_client.Init();

    std::cout << "Franka video client initialized. Saving frames to " << output_dir << "/" << std::endl;

    int frame_count = 0;
    std::vector<uint8_t> image_data;

    while (running)
    {
        int32_t ret = video_client.GetImageSample(image_data);

        if (ret == 0)
        {
            ++frame_count;

            // Save every frame
            char filename[256];
            snprintf(filename, sizeof(filename), "%s/frame_%06d.png",
                     output_dir.c_str(), frame_count);

            std::ofstream file(filename, std::ios::binary);
            if (file.is_open())
            {
                file.write(reinterpret_cast<const char *>(image_data.data()),
                           image_data.size());
                file.close();
            }

            if (frame_count % 100 == 0)
                std::cout << "Saved " << frame_count << " frames" << std::endl;
        }
        else
        {
            std::cerr << "GetImageSample failed (ret=" << ret << ")" << std::endl;
        }

        usleep(33000); // ~30 fps polling
    }

    std::cout << "\nTotal frames saved: " << frame_count << std::endl;
    return 0;
}
