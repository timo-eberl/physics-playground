cmake_minimum_required(VERSION 3.5)
project(tics)
set (CMAKE_CXX_STANDARD 20) # Require C++20

# Generate compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Our Project
set(SOURCES
	src/world.cpp
)
add_library(${PROJECT_NAME} ${SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC include)
