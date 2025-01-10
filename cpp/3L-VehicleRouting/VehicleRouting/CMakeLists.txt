# https://cliutils.gitlab.io/modern-cmake/
# for CI: https://raymii.org/s/articles/Github_Actions_cpp_boost_cmake_speedup.html

cmake_minimum_required(VERSION 3.18)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

project(
	VehicleRouting
	VERSION 0.9
	LANGUAGES CXX)
message(STATUS "project: ${PROJECT_NAME}")
message(STATUS "version: ${PROJECT_VERSION}")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

file(GLOB_RECURSE _HDRS "include/${PROJECT_NAME}/*.h")
file(GLOB_RECURSE _SRCS "src/*.cpp")

# Append files from another directory
file(GLOB_RECURSE _ADDITIONAL_SRCS "${CMAKE_SOURCE_DIR}/external/cvrpsep/*.cpp")
list(APPEND _SRCS ${_ADDITIONAL_SRCS})

# Append files from another directory
file(GLOB_RECURSE _ADDITIONAL_HDRS "${CMAKE_SOURCE_DIR}/external/cvrpsep/*.h")
list(APPEND _HDRS ${_ADDITIONAL_HDRS})

# Projects
add_library(${PROJECT_NAME} "")
target_sources(${PROJECT_NAME} PRIVATE ${_HDRS} ${_SRCS})
message(STATUS "Headers ${_HDRS}")
message(STATUS "Sources ${_SRCS}")

target_include_directories(${PROJECT_NAME} PUBLIC
	${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
	${CMAKE_SOURCE_DIR}/external
	${CMAKE_SOURCE_DIR}/CommonBasics/include
	${CMAKE_SOURCE_DIR}/ContainerLoading/include
	${Boost_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME} PRIVATE CommonBasics ContainerLoading)

if(UNIX)
	target_link_libraries(${PROJECT_NAME} PRIVATE tbb)
	message(STATUS "Linking tbb")
endif()
