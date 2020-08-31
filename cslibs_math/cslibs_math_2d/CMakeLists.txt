cmake_minimum_required(VERSION 2.8.3)
project(cslibs_math_2d)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    cslibs_math
)

find_package(Eigen3 REQUIRED)
find_package(tf                    QUIET)
find_package(cslibs_boost_geometry QUIET)

catkin_package(
    INCLUDE_DIRS    include
    CATKIN_DEPENDS  cslibs_math
)

include_directories(
    ${sensors_msgs_INCLUDE_DIRS}
    ${tf_INCLUDE_DIRS}
    ${cslibs_boost_geometry_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    include
)

cslibs_math_2d_show_headers()

#### test
if(${tf_FOUND})
    cslibs_math_2d_add_unit_test_gtest(${PROJECT_NAME}_test_transform
        SRCS test/transform.cpp
    )
    cslibs_math_2d_add_unit_test_gtest(${PROJECT_NAME}_test_vector
        SRCS test/vector.cpp
    )
    cslibs_math_2d_add_unit_test_gtest(${PROJECT_NAME}_test_ray_tracing
        SRCS test/ray_tracing.cpp
    )
endif()

cslibs_math_2d_add_unit_test_gtest(${PROJECT_NAME}_test_ellipse
    SRCS test/ellipse_test.cpp
)


cslibs_math_2d_add_unit_test_gtest(${PROJECT_NAME}_test_box
    SRCS test/box.cpp
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

