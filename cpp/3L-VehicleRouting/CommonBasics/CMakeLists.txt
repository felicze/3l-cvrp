# https://cliutils.gitlab.io/modern-cmake/
# for CI: https://raymii.org/s/articles/Github_Actions_cpp_boost_cmake_speedup.html

cmake_minimum_required(VERSION 3.18)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

project(
  CommonBasics
  VERSION 0.9
  LANGUAGES CXX)
message(STATUS "project: ${PROJECT_NAME}")
message(STATUS "version: ${PROJECT_VERSION}")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# # specify the C++ standard
# add_library(CommonBasicsCompilerFlags INTERFACE)
# target_compile_features(CommonBasicsCompilerFlags INTERFACE cxx_std_20)

# # add compiler warning flags just when building this project via
# # the BUILD_INTERFACE genex
# set(gcc_like_cxx "$<COMPILE_LANG_AND_ID:CXX,ARMClang,AppleClang,Clang,GNU,LCC>")
# set(msvc_cxx "$<COMPILE_LANG_AND_ID:CXX,MSVC>")
# target_compile_options(CommonBasicsCompilerFlags INTERFACE
# "$<${gcc_like_cxx}:$<BUILD_INTERFACE:-Wall;-Wextra;-Wshadow;-Wformat=2;-Wunused>>"
# "$<${msvc_cxx}:$<BUILD_INTERFACE:-W3>>"
# )
option(CXX "enable C++ compilation" ON)

if(CXX)
  enable_language(CXX)
endif()

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

# Visual Studio compiler with static runtime libraries
if(MSVC AND MT)
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")
  set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /MT")
  set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} /MTd")
endif()

file(GLOB_RECURSE _HDRS "include/${PROJECT_NAME}/*.h")
file(GLOB_RECURSE _SRCS "src/*.cpp")

message(STATUS "Headers ${_HDRS}")
message(STATUS "Sources ${_SRCS}")

# Gurobi
find_package(GUROBI REQUIRED)

# OR-Tools
# list(APPEND CMAKE_PREFIX_PATH "C:/Program Files/or-tools")
# set(ORTOOLS_ROOT "C:/Program Files/or-tools")
# list(APPEND CMAKE_PREFIX_PATH ${ORTOOLS_ROOT})
find_package(ortools CONFIG REQUIRED)

if(${ortools_FOUND})
  message(STATUS "ORTOOLS: ${ortools_FOUND}")
endif()

# Projects
add_library(${PROJECT_NAME} INTERFACE "")
target_sources(${PROJECT_NAME} PRIVATE ${_HDRS} ${_SRCS})

message(STATUS "Gurobi include: ${GUROBI_INCLUDE_DIRS}")
message(STATUS "Gurobi lib: ${GUROBI_LIBRARY}")
message(STATUS "Gurobi CXX: ${GUROBI_CXX_LIBRARY}")
message(STATUS "Gurobi CXX Debug: ${GUROBI_CXX_DEBUG_LIBRARY}")

target_include_directories(${PROJECT_NAME} INTERFACE
  ${PROJECT_SOURCE_DIR}/include

  # $<INSTALL_INTERFACE:include>
  ${GUROBI_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} INTERFACE ortools::ortools ${GUROBI_LIBRARY})

if(CXX)
  # set(CMAKE_CXX_STANDARD 11)
  target_link_libraries(${PROJECT_NAME} INTERFACE optimized ${GUROBI_CXX_LIBRARY}
    debug ${GUROBI_CXX_LIBRARY})
endif()

if(UNIX)
  target_link_libraries(${PROJECT_NAME} INTERFACE tbb)
  message(STATUS "Linking tbb")
endif()
