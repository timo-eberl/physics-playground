cmake_minimum_required(VERSION 3.5)
project(geomath)
set (CMAKE_CXX_STANDARD 20) # Require C++20

# Generate compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# This Project
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE include)
