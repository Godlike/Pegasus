# Copyright (C) 2017 by Godlike
# This code is licensed under the MIT license (MIT)
# (http://opensource.org/licenses/MIT)

set(IMGUI_INCLUDE_DIR "${PEGASUS_ROOT}/third_party/imgui")
set(IMGUI_SOURCE_DIR "${PEGASUS_ROOT}/third_party/imgui")
set(IMGUI_BINARY_DIR "${CMAKE_BINARY_DIR}/imgui" CACHE STRING "Path to IMGUI binary directory")

configure_file(${IMGUI_INCLUDE_DIR}/examples/opengl3_example/imgui_impl_glfw_gl3.h ${IMGUI_INCLUDE_DIR} COPYONLY)
configure_file(${IMGUI_INCLUDE_DIR}/examples/opengl3_example/imgui_impl_glfw_gl3.cpp ${IMGUI_INCLUDE_DIR} COPYONLY)

set(IMGUI_SOURCES
	${IMGUI_INCLUDE_DIR}/imgui.cpp
	${IMGUI_INCLUDE_DIR}/imgui_draw.cpp
	${IMGUI_INCLUDE_DIR}/imgui_impl_glfw_gl3.cpp
)
