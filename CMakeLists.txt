cmake_minimum_required(VERSION 3.13)
project(KDL_Hello)

set(CMAKE_CXX_STANDARD 14)




find_package(orocos_kdl REQUIRED)
include_directories(${orocos_kdl_INCLUDE_DIRS})

add_executable(KDL_Hello main.cpp)
target_link_libraries(${PROJECT_NAME} ${orocos_kdl_LIBRARIES})