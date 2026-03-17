cmake_minimum_required(VERSION 3.16)
project(stand_go2)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)

add_executable(stand_go2 stand_go2.cpp)
target_link_libraries(stand_go2  unitree_sdk2)

add_executable(video_client_go2 video_client_go2.cpp)
target_link_libraries(video_client_go2  unitree_sdk2)

add_executable(move_franka move_franka.cpp)
target_link_libraries(move_franka  unitree_sdk2)

add_executable(video_client_franka video_client_franka.cpp)
target_link_libraries(video_client_franka  unitree_sdk2)

SET(CMAKE_BUILD_TYPE Release)