cmake_minimum_required(VERSION 2.8.3)
project(cslibs_math_3d)

find_package(catkin REQUIRED COMPONENTS
    cslibs_math
)

find_package(Eigen3 REQUIRED)
find_package(tf                    QUIET)
find_package(sensor_msgs           QUIET)

catkin_package(
    INCLUDE_DIRS    include
    CATKIN_DEPENDS  cslibs_math
)

include_directories(
    ${Eigen3_INCLUDE_DIR}
    ${sensors_msgs_INCLUDE_DIRS}
    ${tf_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    include
)

cslibs_math_3d_show_headers()

#### test
if(${tf_FOUND})
    cslibs_math_3d_add_unit_test_gtest(${PROJECT_NAME}_test_quaternion
        SRCS test/quaternion.cpp
    )
    cslibs_math_3d_add_unit_test_gtest(${PROJECT_NAME}_test_transform
        SRCS test/transform.cpp
    )
    cslibs_math_3d_add_unit_test_gtest(${PROJECT_NAME}_test_ray_tracing
        SRCS test/ray_tracing.cpp
    )
endif()

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
