# Copyright (C) 2017 by Godlike
# This code is licensed under the MIT license (MIT)
# (http://opensource.org/licenses/MIT)

cmake_minimum_required (VERSION 3.1)

project(PegasusDemo)
set(CMAKE_CXX_STANDARD 14)

#FreeGLUT
include(GlutConfig)
add_subdirectory(${GLUT_SOURCE_DIR} ${GLUT_BINARY_DIR})
include_directories(${GLUT_INCLUDE_DIR})

#OpenGL
find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
add_definitions(${OpenGL_DEFINITIONS})
if(NOT OPENGL_FOUND)
    message(ERROR " OPENGL not found!")
endif(NOT OPENGL_FOUND)

#Target
include(PegasusConfig)
include_directories (
    ${PEGASUS_INCLUDE_DIR}
)
set(DEMO_HEADERS
    app.hpp
    timing.hpp
    random.hpp
    ogl_headers.hpp
)
set(DEMO_SOURCES
    app.cpp
    timing.cpp
    random.cpp
    falling.cpp
    main.cpp
)

add_executable (${PROJECT_NAME} 
    ${DEMO_SOURCES} 
    ${DEMO_HEADERS}
)
target_link_libraries(${PROJECT_NAME} 
    ${OPENGL_LIBRARIES} 
    ${GLUT_LIBRARIES}
    ${PEGASUS_LIB}
)
