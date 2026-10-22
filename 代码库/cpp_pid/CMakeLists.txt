cmake_minimum_required(VERSION 3.16)

project(cpp_pid VERSION 1.0.0 LANGUAGES CXX)

add_library(pid_controller src/PIDController.cpp)
target_include_directories(pid_controller PUBLIC include)
target_compile_features(pid_controller PUBLIC cxx_std_17)

add_executable(pid_example examples/main.cpp)
target_link_libraries(pid_example PRIVATE pid_controller)

include(CTest)
if(BUILD_TESTING)
    add_executable(pid_controller_test tests/PIDControllerTest.cpp)
    target_link_libraries(pid_controller_test PRIVATE pid_controller)
    add_test(NAME pid_controller_test COMMAND pid_controller_test)
endif()
