cmake_minimum_required(VERSION 3.18)

project(3L-VehicleRouting VERSION 0.9 LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# include(GNUInstallDirs)

# After a normal build, we can specify the location of various outputs of the
# build. We put executables and static libraries in the build directory in
# bin/ and lib/, respectively.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

if(UNIX)
    set(ORTOOLS_ROOT "${ORTOOLS_ROOT_UNIX}")
    set(BOOST_ROOT "${BOOST_ROOT_UNIX}")
    set(GUROBI_DIR "${GUROBI_ROOT_UNIX}")
else()
    set(ORTOOLS_ROOT "${ORTOOLS_ROOT_WIN}")
    set(BOOST_ROOT "${BOOST_ROOT_WIN}")
    set(GUROBI_DIR "${GUROBI_ROOT_WIN}")
endif()

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${ORTOOLS_ROOT})
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${BOOST_ROOT})
message(STATUS "CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH}")

# OR-Tools has only a Release lib
if(CXX AND MSVC)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDLL")
    add_compile_options(/MD)
endif()

# Build Dependencies
add_subdirectory(CommonBasics)
add_subdirectory(ContainerLoading)
add_subdirectory(VehicleRouting)
add_subdirectory(3L-VehicleRoutingApplication)
