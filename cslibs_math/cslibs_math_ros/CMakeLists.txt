cmake_minimum_required(VERSION 2.8.3)
project(cslibs_math_ros)

find_package(catkin REQUIRED COMPONENTS
    cslibs_math
    cslibs_math_2d
    cslibs_math_3d
    cslibs_time
    tf
)

find_package(roscpp         QUIET)
find_package(sensor_msgs    QUIET)
find_package(geometry_msgs  QUIET)

catkin_package(
    INCLUDE_DIRS    include
)

include_directories(
    ${Eigen3_INCLUDE_DIR}
    ${roscpp_INCLUDE_DIRS}
    ${sensors_msgs_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    include
)

cslibs_math_ros_show_headers()

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
