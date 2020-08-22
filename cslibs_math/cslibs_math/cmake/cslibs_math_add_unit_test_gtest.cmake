# ${PROJECT_NAME}_add_unit_test_gtest(<NAME_OF_THE_TEST> [<SRCS> ...] [<LIBS> ...]) 
function(${PROJECT_NAME}_add_unit_test_gtest)
    cmake_parse_arguments(unit_test
        ""          # list of names of the boolean arguments (only defined ones will be true)
        ""          # list of names of mono-valued arguments
        "SRCS;LIBS" # list of names of multi-valued arguments (output variables are lists)
        ${ARGN}     # arguments of the function to parse, here we take the all original ones
    )

    if(${catkin_FOUND})
        set(unit_test_NAME ${ARGV0})
        find_package(Boost REQUIRED COMPONENTS system)
        add_definitions(-pthread)
        catkin_add_gtest(${unit_test_NAME}
            ${unit_test_SRCS}
        )
        target_link_libraries(${unit_test_NAME}
            ${unit_test_LIBS}
            ${GTEST_LIBRARIES}
            ${Boost_LIBRARIES}
            -lpthread
        )
    else()
        set(unit_test_NAME ${PROJECT_NAME}_${ARGV0})
        enable_testing()
        add_executable(${unit_test_NAME}
            ${unit_test_SRCS}
        )
        target_link_libraries(${unit_test_NAME}
            ${unit_test_LIBS}
            ${GTEST_LIBRARIES}
        )
        add_test(AllTestsIn${unit_test_NAME} ${unit_test_NAME})
    endif()
endfunction()
