#include <unitree/robot/go2/video/video_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <iostream>
#include <fstream>
#include <ctime>
#include <csignal>
#include <atomic>

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

    unitree::robot::go2::VideoClient video_client;
    video_client.SetTimeout(2.0f);
    video_client.Init();

    std::cout << "Video client initialized. Requesting frames..." << std::endl;

    int frame_count = 0;
    std::vector<uint8_t> image_data;

    while (running)
    {
        int32_t ret = video_client.GetImageSample(image_data);

        if (ret == 0)
        {
            ++frame_count;

            // Save every 30th frame to disk as PNG
            if (frame_count % 30 == 1)
            {
                time_t rawtime;
                struct tm *timeinfo;
                char buffer[80];

                time(&rawtime);
                timeinfo = localtime(&rawtime);
                strftime(buffer, sizeof(buffer), "frame_%Y%m%d_%H%M%S.png", timeinfo);

                std::ofstream file(buffer, std::ios::binary);
                if (file.is_open())
                {
                    file.write(reinterpret_cast<const char *>(image_data.data()),
                               image_data.size());
                    file.close();
                    std::cout << "Saved " << buffer
                              << " (" << image_data.size() << " bytes)" << std::endl;
                }
            }

            if (frame_count % 100 == 0)
                std::cout << "Received " << frame_count << " frames" << std::endl;
        }
        else
        {
            std::cerr << "GetImageSample failed (ret=" << ret << ")" << std::endl;
        }

        usleep(33000); // ~30 fps polling
    }

    std::cout << "\nTotal frames received: " << frame_count << std::endl;
    return 0;
}
