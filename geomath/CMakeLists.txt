cmake_minimum_required(VERSION 3.5)
project(geomath)
set (CMAKE_CXX_STANDARD 20) # Require C++20

# Generate compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# This Project
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE include)

# Tests
include(CTest)
set (TESTS
	tests/vector.cpp
)
# Create a executable for each test
foreach(test IN ZIP_LISTS TESTS)
	# create variable test_name: Convert test/blablub.cpp to test_blablub
	set (TEST_NAME ${test_0})
	string(REPLACE "/" "_" TEST_NAME ${TEST_NAME})
	string(REPLACE ".cpp" "" TEST_NAME ${TEST_NAME})
	add_executable(${PROJECT_NAME}_${TEST_NAME} ${test_0})
	# link against this library
	target_link_libraries(${PROJECT_NAME}_${TEST_NAME} ${PROJECT_NAME})

	add_test(NAME ${TEST_NAME} COMMAND ${PROJECT_NAME}_${TEST_NAME})
endforeach()
