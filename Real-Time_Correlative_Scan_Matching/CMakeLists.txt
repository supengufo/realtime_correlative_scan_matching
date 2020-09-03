cmake_minimum_required(VERSION 2.8.3)
project(realtime_correlative_scan_matching)


add_compile_options(-std=c++11)
SET(CMAKE_BUILD_TYPE Release)
#set (Gperftools_DIR" ${CMAKE_CURRENT_LIST_DIR}/../cmake/")
#SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../cmake)
message("CMAKE_CURRENT_LIST_DIR:" ${CMAKE_CURRENT_LIST_DIR})
message("PROJECT_SOURCE_DIR:" ${PROJECT_SOURCE_DIR})
#find_package(Gperftools REQUIRED)

## Find catkin macros and libraries

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rosbag
        cv_bridge
        geometry_msgs
        nav_msgs
        sensor_msgs
        std_msgs
        tf
        # cslibs
        cslibs_time
        cslibs_math_2d
        )

catkin_package(
)

find_package(OpenCV 3.0 QUIET)
find_package(Eigen3 REQUIRED)
include_directories(
        ${catkin_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
)

add_executable(mapping_node src/mapping_node.cpp include/mapper.cpp include/grid.cpp include/multi_resolution_map.cpp include/single_layer.cpp)

target_link_libraries(mapping_node yaml-cpp ${catkin_LIBRARIES} /usr/local/lib/libprofiler.so)

