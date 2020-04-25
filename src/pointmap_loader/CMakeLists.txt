cmake_minimum_required(VERSION 2.8.3)
project(pointmap_loader)

add_compile_options(-std=c++17 -Wunused)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    pcl_ros
    sensor_msgs
)

find_package(PCL 1.7.2 REQUIRED)

catkin_package(
    CATKIN_DEPENDS
        roscpp
        pcl_ros
        sensor_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(pointmap_loader
    src/pointmap_loader.cpp
    src/main.cpp
)

target_link_libraries(pointmap_loader
    ${catkin_LIBRARIES}
    -lstdc++fs
)

add_dependencies(pointmap_loader
    ${catkin_EXPORTED_TARGETS}
)
