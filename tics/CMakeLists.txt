cmake_minimum_required(VERSION 3.5)
project(tics)
set (CMAKE_CXX_STANDARD 20) # Require C++20

# Generate compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Dependencies
add_subdirectory (${CMAKE_CURRENT_SOURCE_DIR}/../terathonmath ${CMAKE_CURRENT_BINARY_DIR}/terathonmath)

# Our Project
set(SOURCES
	src/world.cpp
	src/collision_test.cpp
	src/impulse_solver.cpp
	src/position_solver.cpp
	src/static_body.cpp
	src/rigid_body.cpp
	src/collision_area.cpp
	src/collision_area_solver.cpp
	src/raycast.cpp
)
add_library(${PROJECT_NAME} ${SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(${PROJECT_NAME} PUBLIC terathonmath)
