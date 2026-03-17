#pragma once

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/idl/ros2/PointCloud2_.hpp>
#include <unitree/idl/ros2/PointField_.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// ---------------------------------------------------------------------------
// PointCloudPublisher – renders a named MuJoCo camera's depth buffer in an
// offscreen GL context, converts it to a 3D point cloud, and publishes it as
// a sensor_msgs::msg::dds_::PointCloud2_ message via DDS.
//
// The `stride` parameter controls granularity: only every stride-th pixel in
// each dimension is sampled, reducing the point count by stride^2.
//
// Runs on its own thread with its own GL context (a hidden GLFW window
// created on the main thread and handed in at construction time).
// ---------------------------------------------------------------------------
class PointCloudPublisher
{
public:
    PointCloudPublisher(mjModel *model, mjData *data,
                        std::recursive_mutex &sim_mtx,
                        GLFWwindow *gl_window,
                        const std::string &camera_name,
                        int width, int height, double fps,
                        int stride = 4,
                        const std::string &topic = "rt/pointcloud")
        : model_(model), data_(data), sim_mtx_(sim_mtx),
          gl_window_(gl_window), camera_name_(camera_name),
          width_(width), height_(height), stride_(std::max(stride, 1)),
          publish_interval_(1.0 / fps), running_(false), topic_(topic) {}

    ~PointCloudPublisher() { stop(); }

    void start()
    {
        running_ = true;
        thread_ = std::thread(&PointCloudPublisher::publishLoop, this);
    }

    void stop()
    {
        running_ = false;
        if (thread_.joinable())
            thread_.join();
    }

private:
    void publishLoop()
    {
        // Make the hidden GL context current on this thread
        glfwMakeContextCurrent(gl_window_);

        // Look up camera id
        int cam_id = mj_name2id(model_, mjOBJ_CAMERA, camera_name_.c_str());
        if (cam_id < 0)
        {
            std::cerr << "[PointCloud] Camera '" << camera_name_
                      << "' not found in model – pointcloud publishing disabled." << std::endl;
            glfwMakeContextCurrent(nullptr);
            return;
        }

        // MuJoCo rendering resources (owned by this thread / GL context)
        mjvScene scn;
        mjvCamera cam;
        mjvOption opt;
        mjrContext con;

        mjv_defaultScene(&scn);
        mjv_defaultCamera(&cam);
        mjv_defaultOption(&opt);
        mjr_defaultContext(&con);

        mjv_makeScene(model_, &scn, 2000);
        mjr_makeContext(model_, &con, mjFONTSCALE_150);

        cam.type = mjCAMERA_FIXED;
        cam.fixedcamid = cam_id;

        mjrRect viewport = {0, 0, width_, height_};

        // Depth buffer (float, one value per pixel)
        std::vector<float> depth_buf(width_ * height_);

        // Camera intrinsics from MuJoCo camera fovy
        double fovy_deg = model_->cam_fovy[cam_id];
        double fovy_rad = fovy_deg * M_PI / 180.0;
        double fy = static_cast<double>(height_) / (2.0 * std::tan(fovy_rad / 2.0));
        double fx = fy; // square pixels
        double cx = static_cast<double>(width_) / 2.0;
        double cy = static_cast<double>(height_) / 2.0;

        // Clipping planes (MuJoCo convention)
        float extent = static_cast<float>(model_->stat.extent);
        float znear = static_cast<float>(model_->vis.map.znear) * extent;
        float zfar = static_cast<float>(model_->vis.map.zfar) * extent;

        // DDS publisher
        unitree::robot::ChannelPublisher<sensor_msgs::msg::dds_::PointCloud2_> publisher(topic_);
        publisher.InitChannel();

        // Pre-build PointField descriptors (x, y, z as FLOAT32)
        using PF = sensor_msgs::msg::dds_::PointField_;
        constexpr uint8_t FLOAT32 = sensor_msgs::msg::dds_::PointField_Constants::FLOAT32_;
        std::vector<PF> fields = {
            PF("x", 0, FLOAT32, 1),
            PF("y", 4, FLOAT32, 1),
            PF("z", 8, FLOAT32, 1),
        };
        constexpr uint32_t point_step = 12; // 3 * sizeof(float)

        // Max possible sampled points (for reserve)
        int sampled_w = (width_ + stride_ - 1) / stride_;
        int sampled_h = (height_ + stride_ - 1) / stride_;

        std::cout << "[PointCloud] Camera '" << camera_name_
                  << "' depth " << width_ << "x" << height_
                  << " stride=" << stride_
                  << " (" << sampled_w << "x" << sampled_h << " sampled)"
                  << " @ " << (1.0 / publish_interval_) << " fps → topic '" << topic_ << "'" << std::endl;

        while (running_)
        {
            auto t0 = std::chrono::steady_clock::now();

            // 1. Update the scene from simulation state (needs the sim lock)
            {
                std::lock_guard<std::recursive_mutex> lock(sim_mtx_);
                mjv_updateScene(model_, data_, &opt, nullptr, &cam, mjCAT_ALL, &scn);
            }

            // 2. Render and read depth buffer
            mjr_render(viewport, &scn, &con);
            mjr_readPixels(nullptr, depth_buf.data(), viewport, &con);

            // 3. Convert depth buffer to point cloud (with stride subsampling)
            // The depth buffer comes from OpenGL with origin at bottom-left.
            std::vector<uint8_t> cloud_data;
            cloud_data.reserve(sampled_w * sampled_h * point_step);

            uint32_t valid_points = 0;
            for (int v = 0; v < height_; v += stride_)
            {
                for (int u = 0; u < width_; u += stride_)
                {
                    // OpenGL depth buffer: row 0 is at bottom
                    float d = depth_buf[(height_ - 1 - v) * width_ + u];

                    // Skip pixels at the far plane (no geometry)
                    if (d >= 1.0f)
                        continue;

                    // Convert OpenGL normalized depth to linear depth (meters)
                    float z = znear * zfar / (zfar - d * (zfar - znear));

                    // Back-project to 3D (camera frame: z forward, x right, y down)
                    float x = static_cast<float>((static_cast<double>(u) - cx) * z / fx);
                    float y = static_cast<float>((static_cast<double>(v) - cy) * z / fy);

                    // Append x, y, z as raw bytes
                    size_t offset = cloud_data.size();
                    cloud_data.resize(offset + point_step);
                    std::memcpy(cloud_data.data() + offset + 0, &x, sizeof(float));
                    std::memcpy(cloud_data.data() + offset + 4, &y, sizeof(float));
                    std::memcpy(cloud_data.data() + offset + 8, &z, sizeof(float));
                    ++valid_points;
                }
            }

            // 4. Build PointCloud2 message
            sensor_msgs::msg::dds_::PointCloud2_ msg;

            // Header
            auto now = std::chrono::system_clock::now();
            auto epoch = now.time_since_epoch();
            auto secs = std::chrono::duration_cast<std::chrono::seconds>(epoch);
            auto nsecs = std::chrono::duration_cast<std::chrono::nanoseconds>(epoch - secs);
            msg.header().stamp().sec(static_cast<int32_t>(secs.count()));
            msg.header().stamp().nanosec(static_cast<uint32_t>(nsecs.count()));
            msg.header().frame_id(camera_name_);

            msg.height(1);
            msg.width(valid_points);
            msg.fields(fields);
            msg.is_bigendian(false);
            msg.point_step(point_step);
            msg.row_step(valid_points * point_step);
            msg.data(std::move(cloud_data));
            msg.is_dense(true);

            // 5. Publish
            publisher.Write(msg);

            // 6. Sleep to maintain target FPS
            auto elapsed = std::chrono::steady_clock::now() - t0;
            auto target = std::chrono::duration<double>(publish_interval_);
            if (elapsed < target)
                std::this_thread::sleep_for(target - elapsed);
        }

        mjr_freeContext(&con);
        mjv_freeScene(&scn);
        glfwMakeContextCurrent(nullptr);
    }

    mjModel *model_;
    mjData *data_;
    std::recursive_mutex &sim_mtx_;
    GLFWwindow *gl_window_;
    std::string camera_name_;
    int width_;
    int height_;
    int stride_;
    double publish_interval_;
    std::atomic<bool> running_;
    std::thread thread_;
    std::string topic_;
};
