# https://cliutils.gitlab.io/modern-cmake/
# for CI: https://raymii.org/s/articles/Github_Actions_cpp_boost_cmake_speedup.html

cmake_minimum_required(VERSION 3.18)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

project(
	ContainerLoading
	VERSION 0.9
	LANGUAGES CXX)
message(STATUS "project: ${PROJECT_NAME}")
message(STATUS "version: ${PROJECT_VERSION}")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Boost
# set(Boost_DEBUG 1)
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

find_package(Boost REQUIRED) # COMPONENTS graph dynamic_bitset property_map) # REQUIRED COMPONENTS graph dynamic_bitset property_map

if(${BOOST_FOUND})
	message(STATUS "Boost_FOUND: ${Boost_FOUND}")
	message(STATUS "Boost version: ${Boost_VERSION}")
	message(STATUS "Boost include: ${Boost_INCLUDE_DIR}")
else()
	message(FATAL_ERROR "Boost not found")
endif()

file(GLOB_RECURSE _HDRS "include/${PROJECT_NAME}/*.h")
file(GLOB_RECURSE _SRCS "src/*.cpp")

# Projects
add_library(${PROJECT_NAME} "")

target_sources(${PROJECT_NAME} PRIVATE ${_HDRS} ${_SRCS})
message(STATUS "Headers ${_HDRS}")
message(STATUS "Sources ${_SRCS}")

target_include_directories(${PROJECT_NAME} PUBLIC
	${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
	${CMAKE_SOURCE_DIR}/external
	${CMAKE_SOURCE_DIR}/CommonBasics/include
	${Boost_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME} PRIVATE CommonBasics)
