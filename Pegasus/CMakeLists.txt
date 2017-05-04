# Copyright (C) 2017 by Godlike
# This code is licensed under the MIT license (MIT)
# (http://opensource.org/licenses/MIT)

cmake_minimum_required (VERSION 3.1)

if(NOT DEFINED PEGASUS_NAME)
    set(PEGASUS_NAME "PegasusPhysics" CACHE STRING "Pegasus project name.")
endif ()
if(NOT DEFINED PEGASUS_LIB)
    set(PEGASUS_LIB ${PEGASUS_NAME} CACHE STRING "Pegasus library name.")
endif ()

project(${PEGASUS_NAME})
set(CMAKE_CXX_STANDARD 14)

include_directories(
    ${PEGASUS_INCLUDE_DIR}
)
set(PEGASUS_HEADERS
    include/core.hpp
    include/particle.hpp
    include/particleforcegenerator.hpp
    include/particlecontacts.hpp
    include/particlelinks.hpp
    include/particleworld.hpp
    include/math.hpp
)
set(PEGASUS_SOURCES
    sources/core.cpp
    sources/particle.cpp
    sources/particleforcegenerator.cpp
    sources/particlecontacts.cpp
    sources/particlelinks.cpp
    sources/particleworld.cpp
    sources/math.cpp
)

add_library(${PEGASUS_LIB}
    ${PEGASUS_HEADERS}
    ${PEGASUS_SOURCES}
)
