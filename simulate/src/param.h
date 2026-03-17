#pragma once

#include <iostream>
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>

namespace param
{

inline struct SimulationConfig
{
    std::string robot;
    std::filesystem::path robot_scene;

    int domain_id;
    std::string interface;

    int use_joystick;
    std::string joystick_type;
    std::string joystick_device;
    int joystick_bits;

    int print_scene_information;

    int enable_elastic_band;
    int band_attached_link = 0;

    std::string camera_name;  // MuJoCo camera name for video server (empty = disabled)
    int camera_width = 640;
    int camera_height = 480;
    double camera_fps = 30.0;

    std::string depth_camera_name;  // MuJoCo camera name for depth/pointcloud (empty = disabled)
    int depth_camera_width = 320;
    int depth_camera_height = 240;
    double depth_camera_fps = 10.0;
    int depth_camera_stride = 4;    // Sample every Nth pixel in each dimension (1 = every pixel)
    std::string pointcloud_topic = "rt/pointcloud";

    void load_from_yaml(const std::string &filename)
    {
        auto cfg = YAML::LoadFile(filename);
        try
        {
            robot = cfg["robot"].as<std::string>();
            robot_scene = cfg["robot_scene"].as<std::string>();
            domain_id = cfg["domain_id"].as<int>();
            interface = cfg["interface"].as<std::string>();
            use_joystick = cfg["use_joystick"].as<int>();
            joystick_type = cfg["joystick_type"].as<std::string>();
            joystick_device = cfg["joystick_device"].as<std::string>();
            joystick_bits = cfg["joystick_bits"].as<int>();
            print_scene_information = cfg["print_scene_information"].as<int>();
            enable_elastic_band = cfg["enable_elastic_band"].as<int>();

            if (cfg["camera_name"])
                camera_name = cfg["camera_name"].as<std::string>();
            if (cfg["camera_width"])
                camera_width = cfg["camera_width"].as<int>();
            if (cfg["camera_height"])
                camera_height = cfg["camera_height"].as<int>();
            if (cfg["camera_fps"])
                camera_fps = cfg["camera_fps"].as<double>();

            if (cfg["depth_camera_name"])
                depth_camera_name = cfg["depth_camera_name"].as<std::string>();
            if (cfg["depth_camera_width"])
                depth_camera_width = cfg["depth_camera_width"].as<int>();
            if (cfg["depth_camera_height"])
                depth_camera_height = cfg["depth_camera_height"].as<int>();
            if (cfg["depth_camera_fps"])
                depth_camera_fps = cfg["depth_camera_fps"].as<double>();
            if (cfg["depth_camera_stride"])
                depth_camera_stride = cfg["depth_camera_stride"].as<int>();
            if (cfg["pointcloud_topic"])
                pointcloud_topic = cfg["pointcloud_topic"].as<std::string>();
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            exit(EXIT_FAILURE);
        }
    }
} config;

/* ---------- Command Line Parameters ---------- */
namespace po = boost::program_options;

//※ This function must be called at the beginning of main() function
inline po::variables_map helper(int argc, char** argv)
{
    po::options_description desc("Unitree Mujoco");
    desc.add_options()
        ("help,h", "Show help message")
        ("domain_id,i", po::value<int>(&config.domain_id), "DDS domain ID; -i 0")
        ("network,n", po::value<std::string>(&config.interface), "DDS network interface; -n eth0")
        ("robot,r", po::value<std::string>(&config.robot), "Robot type; -r go2")
        ("scene,s", po::value<std::filesystem::path>(&config.robot_scene), "Robot scene file; -s scene_terrain.xml")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    
    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        exit(0);
    }

    return vm;
}

}