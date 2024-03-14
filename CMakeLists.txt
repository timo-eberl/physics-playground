cmake_minimum_required(VERSION 3.11) # FetchContent is available in 3.11+
project(physics-playground)
set (CMAKE_CXX_STANDARD 20) # Require C++20

# dependencies
	# tics (own physics library)
	add_subdirectory (tics)

	# ron (own graphics library)
	set(RON_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE) # dont build examples
	set(RON_ASSET_DIRECTORY ${PROJECT_SOURCE_DIR}/assets/) # use our own assets directory
	add_subdirectory (ron-the-renderer)

# this Project
	set(SOURCES

	)
	add_executable(${PROJECT_NAME}
		src/main.cpp
		${SOURCES}
	)
	target_include_directories(${PROJECT_NAME} PRIVATE src)
	# link libraries
	target_link_libraries(${PROJECT_NAME} PRIVATE tics)
	target_link_libraries(${PROJECT_NAME} PRIVATE ron)
