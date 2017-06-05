# Copyright (C) 2017 by Godlike
# This code is licensed under the MIT license (MIT)
# (http://opensource.org/licenses/MIT)

cmake_minimum_required (VERSION 3.1)

include(PegasusConfig)
if(NOT DEFINED PEGASUS_NAME)
    set(PEGASUS_NAME "PegasusPhysics" CACHE STRING "Pegasus project name.")
endif ()
if(NOT DEFINED PEGASUS_LIB)
    set(PEGASUS_LIB ${PEGASUS_NAME} CACHE STRING "Pegasus library name.")
endif ()

project(${PEGASUS_NAME})
set(CMAKE_CXX_STANDARD 14)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

#Eigen
include(EigenConfig)
add_subdirectory(${EIGEN_SOURCE_DIR})
include_directories(${EIGEN_INCLUDE_DIR})

#Target
include_directories(
    ${PEGASUS_INCLUDE_DIR}
)
set(PEGASUS_HEADERS
    include/particle.hpp
    include/particleforcegenerator.hpp
    include/particlecontacts.hpp
    include/particlelinks.hpp
    include/particleworld.hpp
    include/geometry.hpp
    include/mechanics.hpp
    include/math.hpp
    include/boundingvolumes.hpp
)
set(PEGASUS_SOURCES
    sources/particle.cpp
    sources/particleforcegenerator.cpp
    sources/particlecontacts.cpp
    sources/particlelinks.cpp
    sources/particleworld.cpp
    sources/geometry.cpp
    sources/mechanics.cpp
    sources/math.cpp
    sources/boundingvolumes.cpp
)

add_library(${PEGASUS_LIB}
    ${PEGASUS_HEADERS}
    ${PEGASUS_SOURCES}
)
target_link_libraries(${PEGASUS_LIB})
