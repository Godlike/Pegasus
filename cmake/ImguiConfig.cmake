# Copyright (C) 2017 by Godlike
# This code is licensed under the MIT license (MIT)
# (http://opensource.org/licenses/MIT)

set(IMGUI_INCLUDE_DIR "${PEGASUS_ROOT}/third_party/imgui" "${PEGASUS_ROOT}/demo/imgui")
set(IMGUI_SOURCE_DIR "${PEGASUS_ROOT}/third_party/imgui" "${PEGASUS_ROOT}/demo/imgui")
set(IMGUI_BINARY_DIR "${CMAKE_BINARY_DIR}/imgui" CACHE STRING "Path to IMGUI binary directory")

set(IMGUI_SOURCES
	${PEGASUS_ROOT}/demo/imgui/imgui.cpp
	${PEGASUS_ROOT}/demo/imgui/imgui_draw.cpp
	${PEGASUS_ROOT}/demo/imgui/imgui_impl_glfw_gl3.cpp
)
