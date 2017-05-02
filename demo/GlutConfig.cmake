cmake_minimum_required (VERSION 3.1)

#Configure GLUT
set(FREEGLUT_BUILD_DEMOS OFF CACHE BOOL "Build FreeGLUT demos.")
set(FREEGLUT_BUILD_SHARED_LIBS OFF CACHE BOOL "Build FreeGLUT shared library.")
set(FREEGLUT_PRINT_ERRORS OFF CACHE BOOL "Lib prints errors to stderr")
set(FREEGLUT_PRINT_WARNINGS OFF CACHE BOOL "Lib prints warnings to stderr")

#Set GLUT variables
set(GLUT_INCLUDE_DIRS
    ${GLUT_SOURCE_DIR}/include
)

set(GLUT_LIBRARIES 
    freeglut_static
)
