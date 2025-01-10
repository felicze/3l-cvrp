# https://cliutils.gitlab.io/modern-cmake/
# for CI: https://raymii.org/s/articles/Github_Actions_cpp_boost_cmake_speedup.html

cmake_minimum_required(VERSION 3.18)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

project(
	3L-VehicleRoutingApplication
	VERSION 0.9
	LANGUAGES CXX)
message(STATUS "project: ${PROJECT_NAME}")
message(STATUS "version: ${PROJECT_VERSION}")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

file(GLOB_RECURSE _HDRS "include/${PROJECT_NAME}/*.h")
file(GLOB_RECURSE _SRCS "src/*.cpp")

message(STATUS "Headers ${_HDRS}")
message(STATUS "Sources ${_SRCS}")

# Projects
add_executable(${PROJECT_NAME} "")
target_sources(${PROJECT_NAME} PRIVATE ${_HDRS} ${_SRCS})

target_include_directories(${PROJECT_NAME} PUBLIC
	${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
	${CMAKE_SOURCE_DIR}/external
	${CMAKE_SOURCE_DIR}/VehicleRouting/include
	${CMAKE_SOURCE_DIR}/ContainerLoading/ContainerLoading/include
	${CMAKE_SOURCE_DIR}/ContainerLoading/CommonBasics/include
)

# Gurobi
find_package(GUROBI REQUIRED)

message(STATUS "Gurobi include: ${GUROBI_INCLUDE_DIRS}")
message(STATUS "Gurobi lib: ${GUROBI_LIBRARY}")
message(STATUS "Gurobi CXX: ${GUROBI_CXX_LIBRARY}")
message(STATUS "Gurobi CXX Debug: ${GUROBI_CXX_DEBUG_LIBRARY}")

if(CXX)
	target_link_libraries(${PROJECT_NAME} PUBLIC optimized ${GUROBI_CXX_LIBRARY}
		debug ${GUROBI_CXX_LIBRARY})
endif()

target_link_libraries(${PROJECT_NAME} PRIVATE VehicleRouting ContainerLoading CommonBasics)
