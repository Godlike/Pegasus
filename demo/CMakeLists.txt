# Copyright (C) 2017 by Godlike
# This code is licensed under the MIT license (MIT)
# (http://opensource.org/licenses/MIT)

cmake_minimum_required (VERSION 3.1)

project(PegasusDemo)

#GLBINDING
include(GlbindingConfig)
add_subdirectory(${GLBINDING_SOURCE_DIR} ${GLBINDING_BINARY_DIR})
include_directories(${GLBINDING_INCLUDE_DIR})

#GLFW
include(GlfwConfig)
add_subdirectory(${GLFW_SOURCE_DIR} ${GLFW_BINARY_DIR})
include_directories(${GLFW_INCLUDE_DIR})

#Target
include(PegasusConfig)
include_directories (
    ${PEGASUS_INCLUDE_DIR}
)
set(DEMO_HEADERS
    Demo.hpp
    Render.hpp
    Bunny.hpp
)
set(DEMO_SOURCES
    Demo.cpp
    Render.cpp
    Main.cpp
)

add_executable (${PROJECT_NAME}
    ${DEMO_SOURCES}
    ${DEMO_HEADERS}
)
target_link_libraries(${PROJECT_NAME}
    ${OPENGL_LIBRARIES}
    ${GLBINDING_LIB}
    ${GLFW_LIB}
    ${PEGASUS_LIB}
)
