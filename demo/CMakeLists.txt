# Copyright (C) 2017 by Godlike
# This code is licensed under the MIT license (MIT)
# (http://opensource.org/licenses/MIT)

cmake_minimum_required (VERSION 3.1)

project(PegasusDemo)

#Target
include(PegasusConfig)

set(PEGASUS_DEMO_INCLUDE_DIR
    ${PEGASUS_ROOT}/demo
)

include_directories (
    ${PEGASUS_INCLUDE_DIR}
    ${EPONA_INCLUDE_DIR}
    ${ARION_INCLUDE_DIR}
    ${PEGASUS_DEMO_INCLUDE_DIR}
)

#IMGUI
include(ImguiConfig)
add_subdirectory(imgui)
include_directories(${IMGUI_INCLUDE_DIR})

include(PegasusBuildFlags)

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
    PegasusDemo_imgui
    ${PEGASUS_LIB}
    ${EPONA_LIB}
    ${ARION_LIB}
)
