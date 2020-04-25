cmake_minimum_required(VERSION 2.8.3)
project(lidar_to_map)

add_compile_options(-std=c++17 -Wunused)

# Dependencies

    find_package(catkin REQUIRED COMPONENTS
        pcl_ros
        rosbag
        roscpp
        sensor_msgs
        tf
    )

    find_package(PCL 1.7.2 REQUIRED)

# Package definition

    catkin_package(
        CATKIN_DEPENDS
            pcl_ros
            rosbag
            roscpp
            sensor_msgs
    )

# Targets

    add_library(${PROJECT_NAME} STATIC
        src/lidar_to_map.cpp
    )

    target_include_directories(${PROJECT_NAME}
        PUBLIC
            include
            ${catkin_INCLUDE_DIRS}
            ${PCL_INCLUDE_DIRS}
            ${EIGEN3_INCLUDE_DIR}
    )

    target_link_libraries(${PROJECT_NAME}
        PRIVATE
            ${catkin_LIBRARIES}
            ${PCL_LIBRARIES}
            -lstdc++fs
    )

    add_dependencies(${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
        ${${PROJECT_NAME}_node_EXPORTED_TARGETS}
    )

    add_executable(${PROJECT_NAME}_node
        src/main.cpp
    )

    target_link_libraries(${PROJECT_NAME}_node
        PRIVATE
            ${PROJECT_NAME}
    )
