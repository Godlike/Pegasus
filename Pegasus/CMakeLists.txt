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

#GLM
add_subdirectory(${GLM_SOURCE_DIR})

#Eigen
add_subdirectory(${EIGEN_SOURCE_DIR})

#Target
include_directories(
    ${PEGASUS_INCLUDE_DIR}
)
set(PEGASUS_HEADERS
    include/Particle.hpp
    include/ParticleForceGenerator.hpp
    include/ParticleContacts.hpp
    include/ParticleLinks.hpp
    include/ParticleWorld.hpp
    include/Geometry.hpp
    include/Mechanics.hpp
    include/Math.hpp
    include/BoundingVolumes.hpp
)
set(PEGASUS_SOURCES
    sources/Particle.cpp
    sources/ParticleForceGenerator.cpp
    sources/ParticleContacts.cpp
    sources/ParticleLinks.cpp
    sources/ParticleWorld.cpp
    sources/Geometry.cpp
    sources/Mechanics.cpp
    sources/Math.cpp
    sources/BoundingVolumes.cpp
)

add_library(${PEGASUS_LIB}
    ${PEGASUS_HEADERS}
    ${PEGASUS_SOURCES}
)
target_link_libraries(${PEGASUS_LIB})
