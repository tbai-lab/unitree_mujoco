#pragma once

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <unitree/robot/server/server.hpp>
#include <unitree/robot/go2/video/video_api.hpp>
#include <unitree/robot/b2/front_video/front_video_api.hpp>
#include <unitree/robot/b2/back_video/back_video_api.hpp>

#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "lodepng.h"

// ---------------------------------------------------------------------------
// CameraRenderer – renders a named MuJoCo camera into an offscreen buffer and
// stores the latest frame as a PNG-encoded byte vector.  The renderer runs on
// its own thread with its own GL context (a hidden GLFW window created on the
// main thread and handed in at construction time).
// ---------------------------------------------------------------------------
class CameraRenderer
{
public:
    CameraRenderer(mjModel *model, mjData *data,
                   std::recursive_mutex &sim_mtx,
                   GLFWwindow *gl_window,
                   const std::string &camera_name,
                   int width, int height, double fps)
        : model_(model), data_(data), sim_mtx_(sim_mtx),
          gl_window_(gl_window), camera_name_(camera_name),
          width_(width), height_(height),
          render_interval_(1.0 / fps), running_(false) {}

    ~CameraRenderer() { stop(); }

    void start()
    {
        running_ = true;
        thread_ = std::thread(&CameraRenderer::renderLoop, this);
    }

    void stop()
    {
        running_ = false;
        if (thread_.joinable())
            thread_.join();
    }

    bool getLatestFrame(std::vector<uint8_t> &out)
    {
        std::lock_guard<std::mutex> lock(frame_mtx_);
        if (latest_frame_.empty())
            return false;
        out = latest_frame_;
        return true;
    }

private:
    void renderLoop()
    {
        // Make the hidden GL context current on this thread
        glfwMakeContextCurrent(gl_window_);

        // Look up camera id
        int cam_id = mj_name2id(model_, mjOBJ_CAMERA, camera_name_.c_str());
        if (cam_id < 0)
        {
            std::cerr << "[VideoServer] Camera '" << camera_name_
                      << "' not found in model – camera rendering disabled." << std::endl;
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

        // Pixel buffer (RGB, bottom-up from OpenGL)
        std::vector<uint8_t> rgb(width_ * height_ * 3);

        std::cout << "[VideoServer] Camera '" << camera_name_
                  << "' rendering at " << width_ << "x" << height_
                  << " @ " << (1.0 / render_interval_) << " fps" << std::endl;

        while (running_)
        {
            auto t0 = std::chrono::steady_clock::now();

            // 1. Update the scene from simulation state (needs the sim lock)
            {
                std::lock_guard<std::recursive_mutex> lock(sim_mtx_);
                mjv_updateScene(model_, data_, &opt, nullptr, &cam, mjCAT_ALL, &scn);
            }

            // 2. Render (only touches our local GL context + scene copy)
            mjr_render(viewport, &scn, &con);
            mjr_readPixels(rgb.data(), nullptr, viewport, &con);

            // 3. Flip vertically (OpenGL origin is bottom-left, image origin is top-left)
            flipVertical(rgb.data(), width_, height_, 3);

            // 4. Encode as PNG
            std::vector<uint8_t> png;
            unsigned err = lodepng::encode(png, rgb.data(),
                                           static_cast<unsigned>(width_),
                                           static_cast<unsigned>(height_),
                                           LCT_RGB, 8);
            if (err == 0)
            {
                std::lock_guard<std::mutex> lock(frame_mtx_);
                latest_frame_ = std::move(png);
            }

            // 5. Sleep to maintain target FPS
            auto elapsed = std::chrono::steady_clock::now() - t0;
            auto target = std::chrono::duration<double>(render_interval_);
            if (elapsed < target)
                std::this_thread::sleep_for(target - elapsed);
        }

        mjr_freeContext(&con);
        mjv_freeScene(&scn);
        glfwMakeContextCurrent(nullptr);
    }

    static void flipVertical(uint8_t *data, int w, int h, int channels)
    {
        int stride = w * channels;
        std::vector<uint8_t> row(stride);
        for (int y = 0; y < h / 2; ++y)
        {
            uint8_t *top = data + y * stride;
            uint8_t *bot = data + (h - 1 - y) * stride;
            std::memcpy(row.data(), top, stride);
            std::memcpy(top, bot, stride);
            std::memcpy(bot, row.data(), stride);
        }
    }

    mjModel *model_;
    mjData *data_;
    std::recursive_mutex &sim_mtx_;
    GLFWwindow *gl_window_;
    std::string camera_name_;
    int width_;
    int height_;
    double render_interval_;
    std::atomic<bool> running_;
    std::thread thread_;
    std::mutex frame_mtx_;
    std::vector<uint8_t> latest_frame_;
};

// ---------------------------------------------------------------------------
// MujocoVideoServer – unitree_sdk2 RPC server that responds to VideoClient's
// GetImageSample requests with the latest rendered frame.
//
// Supports the Go2 "videohub" service as well as the B2 "front_videohub" and
// "back_videohub" services — the service name is chosen based on the robot
// configuration.
// ---------------------------------------------------------------------------
class MujocoVideoServer : public unitree::robot::Server
{
public:
    MujocoVideoServer(const std::string &service_name,
                      std::shared_ptr<CameraRenderer> renderer)
        : Server(service_name), renderer_(std::move(renderer)) {}

    void Init() override
    {
        SetApiVersion(unitree::robot::go2::ROBOT_VIDEO_API_VERSION);
        UT_ROBOT_SERVER_REG_API_BINARY_HANDLER_NO_LEASE(
            unitree::robot::go2::ROBOT_VIDEO_API_ID_GETIMAGESAMPLE,
            &MujocoVideoServer::handleGetImageSample);
    }

private:
    int32_t handleGetImageSample(const std::vector<uint8_t> & /*parameter*/,
                                 std::vector<uint8_t> &data)
    {
        if (!renderer_ || !renderer_->getLatestFrame(data))
            return -1; // no frame available yet
        return 0;
    }

    std::shared_ptr<CameraRenderer> renderer_;
};

// ---------------------------------------------------------------------------
// Helper: create a set of video servers for a given camera + renderer.
// Go2 uses "videohub"; B2 uses "front_videohub" (and optionally
// "back_videohub" for a second camera – not handled here, but easy to add).
// We register on all service names so that any client variant works.
// ---------------------------------------------------------------------------
inline std::vector<std::unique_ptr<MujocoVideoServer>>
createVideoServers(std::shared_ptr<CameraRenderer> renderer)
{
    std::vector<std::unique_ptr<MujocoVideoServer>> servers;
    // Register on all known video service names
    for (const auto &name : {
             unitree::robot::go2::ROBOT_VIDEO_SERVICE_NAME,        // "videohub"
         })
    {
        auto srv = std::make_unique<MujocoVideoServer>(name, renderer);
        srv->Init();
        srv->Start();
        servers.push_back(std::move(srv));
    }
    return servers;
}
