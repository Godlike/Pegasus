# Copyright (C) 2018 by Godlike
# This code is licensed under the MIT license (MIT)
# (http://opensource.org/licenses/MIT)

cmake_minimum_required (VERSION 3.1)

project(PegasusDemo_imgui)

#GLBINDING
include(GlbindingConfig)
add_subdirectory(${GLBINDING_SOURCE_DIR} ${GLBINDING_BINARY_DIR})
include_directories(${GLBINDING_INCLUDE_DIR})

#GLFW
include(GlfwConfig)
add_subdirectory(${GLFW_SOURCE_DIR} ${GLFW_BINARY_DIR})
include_directories(${GLFW_INCLUDE_DIR})

#Imgui
include(ImguiConfig)
include_directories(${IMGUI_INCLUDE_DIR})

add_library(${PROJECT_NAME} STATIC
    ${PEGASUS_ROOT}/demo/imgui/imgui.cpp
    ${PEGASUS_ROOT}/demo/imgui/imgui_draw.cpp
    ${PEGASUS_ROOT}/demo/imgui/imgui_impl_glfw_gl3.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${OPENGL_LIBRARIES}
    ${GLBINDING_LIB}
    ${GLFW_LIB}
)
