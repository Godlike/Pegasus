# Copyright (C) 2017 by Godlike
# This code is licensed under the MIT license (MIT)
# (http://opensource.org/licenses/MIT)

cmake_minimum_required(VERSION 3.1)

project(Pegasus)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

option(PEGASUS_BUILD_DEMOS "Build Pegasus demo." ON)

message(STATUS "${PROJECT_NAME} configuration:")
message(STATUS "-- PEGASUS_BUILD_DEMOS: ${PEGASUS_BUILD_DEMOS}")

set(PEGASUS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}" CACHE STRING "Pegasus root directory.")

include(PegasusConfig)

if (NOT CMAKE_ARCHIVE_OUTPUT_DIRECTORY)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/output/lib)
endif()

if (NOT CMAKE_LIBRARY_OUTPUT_DIRECTORY)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/output/lib)
endif()

if (NOT CMAKE_RUNTIME_OUTPUT_DIRECTORY)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/output)
endif()

add_subdirectory(Pegasus)

if (${PEGASUS_BUILD_DEMOS})
    add_subdirectory(demo)
endif()

set(PEGASUS_PHYSICS_LIB ${PEGASUS_LIB} PARENT_SCOPE)
