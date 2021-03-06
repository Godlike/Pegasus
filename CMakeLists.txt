# Copyright (C) 2017 by Godlike
# This code is licensed under the MIT license (MIT)
# (http://opensource.org/licenses/MIT)

cmake_minimum_required(VERSION 3.1)

project(Pegasus)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

option(PEGASUS_BUILD_DEMOS "Build Pegasus demo."  ON)
option(PEGASUS_BUILD_TESTS "Build Pegasus tests." ON)
option(PEGASUS_BUILD_DEBUG "Build Pegasus debug." ON)

message(STATUS "${PROJECT_NAME} ${CMAKE_BUILD_TYPE} configuration:")
message(STATUS "-- PEGASUS_BUILD_DEMOS: ${PEGASUS_BUILD_DEMOS}")
message(STATUS "-- PEGASUS_BUILD_TESTS: ${PEGASUS_BUILD_TESTS}")
message(STATUS "-- PEGASUS_BUILD_DEBUG: ${PEGASUS_BUILD_DEBUG}")

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
add_definitions(${PEGASUS_DEFINITIONS})
message(STATUS "Pegasus definitions:${PEGASUS_DEFINITIONS}")

if (${PEGASUS_BUILD_DEMOS})
    add_subdirectory(demo)
endif()

if (PEGASUS_BUILD_TESTS)
    enable_testing()
    set(CATCH_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/third_party/Catch)
    add_subdirectory(test)
endif()

get_directory_property(HAS_PARENT PARENT_DIRECTORY)
if (HAS_PARENT)
    set(PEGASUS_PHYSICS_LIB ${PEGASUS_LIB} PARENT_SCOPE)
endif()
