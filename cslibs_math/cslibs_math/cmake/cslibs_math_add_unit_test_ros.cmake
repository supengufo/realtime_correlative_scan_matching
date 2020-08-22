# ${PROJECT_NAME}_add_unit_test_gtest(<NAME_OF_THE_TEST> [<SRCS> ...] [<LIBS> ...])
function(${PROJECT_NAME}_add_unit_test_ros)
    cmake_parse_arguments(unit_test
        ""          # list of names of the boolean arguments (only defined ones will be true)
        "LAUNCH"    # list of names of mono-valued arguments
        "SRCS;LIBS" # list of names of multi-valued arguments (output variables are lists)
        ${ARGN}     # arguments of the function to parse, here we take the all original ones
    )

    find_package(rostest QUIET)
    find_package(roscpp QUIET)
    if(${rostest_FOUND})
        include_directories(${rostest_INCLUDE_DIRS}
                            ${roscpp_INCLUDE_DIRS})
        set(unit_test_NAME ${ARGV0})
        add_rostest_gtest(${unit_test_NAME}
                          ${unit_test_LAUNCH}
                          ${unit_test_SRCS})

        target_link_libraries(${unit_test_NAME}
              ${unit_test_LIBS}
              ${rostest_LIBRARIES}
              ${roscpp_LIBRARIES}
              ${GTEST_LIBRARIES}
        )
    endif()
endfunction()