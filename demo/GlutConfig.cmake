cmake_minimum_required (VERSION 3.1)

#Configure GLUT
set(FREEGLUT_BUILD_DEMOS OFF CACHE BOOL "Build FreeGLUT demos.")
set(FREEGLUT_BUILD_SHARED_LIBS OFF CACHE BOOL "Build FreeGLUT shared library.")
set(FREEGLUT_PRINT_ERRORS OFF CACHE BOOL "Lib prints errors to stderr")
set(FREEGLUT_PRINT_WARNINGS OFF CACHE BOOL "Lib prints warnings to stderr")

#Set GLUT variables
set(GLUT_PATH
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/FreeGLUT/freeglut/freeglut
)
set(GLUT_LIBRARY_DIRS
    ${CMAKE_CURRENT_BINARY_DIR}/third_party/FreeGLUT/freeglut/freeglut
)
set(GLUT_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/FreeGLUT/freeglut/freeglut/include
)
