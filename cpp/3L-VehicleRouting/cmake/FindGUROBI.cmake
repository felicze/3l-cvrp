# if the GUROBI_HOME refers to another verson of Gurobi (as 10.0.3),
# you have to set the GUROBI_DIR to the correct path, see ${workspaceFolder}/CMakeLists.txt
# Alternative: Adjust gurobi version below to the GUROBI_HOME version
if(NOT DEFINED GUROBI_DIR OR GUROBI_DIR STREQUAL "")
    set(GUROBI_DIR $ENV{GUROBI_HOME})
endif()

find_path(GUROBI_INCLUDE_DIRS
    NAMES gurobi_c.h
    HINTS ${GUROBI_DIR}
    PATH_SUFFIXES include)

find_library(GUROBI_LIBRARY
    NAMES gurobi gurobi100
    HINTS ${GUROBI_DIR}
    PATH_SUFFIXES lib)

if(CXX)
    message(STATUS "GUROBI CXX ${GUROBI_DIR} ENV GUROBI HOME $ENV{GUROBI_HOME}")

    if(MSVC)
        set(MSVC_YEAR "2017")

        if(MT)
            set(M_FLAG "mt")
        else()
            set(M_FLAG "md")
        endif()

        find_library(GUROBI_CXX_LIBRARY
            NAMES gurobi_c++${M_FLAG}${MSVC_YEAR}
            HINTS ${GUROBI_DIR}
            PATH_SUFFIXES lib)
        find_library(GUROBI_CXX_DEBUG_LIBRARY
            NAMES gurobi_c++${M_FLAG}d${MSVC_YEAR}
            HINTS ${GUROBI_DIR}
            PATH_SUFFIXES lib)
    else()
        find_library(GUROBI_CXX_LIBRARY
            NAMES gurobi_c++
            HINTS ${GUROBI_DIR}
            PATH_SUFFIXES lib)

        # set(GUROBI_CXX_DEBUG_LIBRARY ${GUROBI_CXX_LIBRARY})
    endif()
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GUROBI DEFAULT_MSG GUROBI_LIBRARY)
