cmake_minimum_required(VERSION 3.15)
project(transform)

set(CMAKE_CXX_STANDARD 14)

add_executable(transform main.cpp)

find_package(absl REQUIRED)
target_link_libraries(transform absl::strings absl::str_format)