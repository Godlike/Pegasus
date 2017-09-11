# Copyright (C) 2017 by Godlike
# This code is licensed under the MIT license (MIT)
# (http://opensource.org/licenses/MIT)

set(FREEGLUT_BUILD_DEMOS OFF CACHE BOOL "Build FreeGLUT demos.")
set(FREEGLUT_BUILD_SHARED_LIBS OFF CACHE BOOL "Build FreeGLUT shared library.")
set(FREEGLUT_PRINT_ERRORS OFF CACHE BOOL "Lib prints errors to stderr")
set(FREEGLUT_PRINT_WARNINGS OFF CACHE BOOL "Lib prints warnings to stderr")

set(GLUT_SOURCE_DIR "${PEGASUS_ROOT}/third_party/FreeGLUT/freeglut/freeglut")
set(GLUT_BINARY_DIR "${CMAKE_BINARY_DIR}/FreeGLUT")
set(GLUT_INCLUDE_DIR ${GLUT_SOURCE_DIR}/include)
set(GLUT_LIBRARIES freeglut_static)
