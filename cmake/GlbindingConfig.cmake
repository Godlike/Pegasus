# Copyright (C) 2017 by Godlike
# This code is licensed under the MIT license (MIT)
# (http://opensource.org/licenses/MIT)

set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build shared instead of static libraries.")
set(OPTION_SELF_CONTAINED OFF CACHE BOOL "Create a self-contained install with all dependencies.")
set(OPTION_BUILD_TESTS OFF CACHE BOOL "Build tests.")
set(OPTION_BUILD_GPU_TESTS OFF CACHE BOOL "Build tests that require an OpenGL context.")
set(OPTION_BUILD_DOCS OFF CACHE BOOL "Build documentation.")
set(OPTION_BUILD_TOOLS OFF CACHE BOOL "Build tools.")
set(OPTION_BUILD_EXAMPLES OFF CACHE BOOL "Build examples.")
set(OPTION_BUILD_WITH_BOOST_THREAD OFF CACHE BOOL "Use boost::thread instead of std::thread.")

set(GLBINDING_SOURCE_DIR "${PEGASUS_ROOT}/third_party/glbinding" CACHE STRING "Path to glbinding root directory")
set(GLBINDING_BINARY_DIR "${CMAKE_BINARY_DIR}/glbinding" CACHE STRING "Path to glbinding binary directory")
set(GLBINDING_LIB "glbinding" CACHE STRING "Name of glbinding library")
set(GLBINDING_INCLUDE_DIR
    "${GLBINDING_SOURCE_DIR}/source/glbinding/include"
    CACHE LIST "List of GLFW include directories")
