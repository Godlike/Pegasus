# Copyright (C) 2017 by Godlike
# This code is licensed under the MIT license (MIT)
# (http://opensource.org/licenses/MIT)

cmake_minimum_required (VERSION 3.1)

include(PegasusConfig)

project(${PEGASUS_NAME})
set(CMAKE_CXX_STANDARD 14)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build (Debug or Release)" FORCE)
endif()

if (NOT GLM_FOUND)
    add_subdirectory(${GLM_ROOT_DIR})
endif()

#Target
include_directories(
    ${PEGASUS_INCLUDE_DIR}
)

message(STATUS "Version: ${PEGASUS_VERSION}")

set(PEGASUS_HEADERS
    include/pegasus/Math.hpp
    include/pegasus/Object.hpp
    include/pegasus/Integration.hpp
    include/pegasus/Collision.hpp
    include/pegasus/ParticleForceGenerator.hpp
    include/pegasus/Scene.hpp
    include/pegasus/BoundingVolumes.hpp
    include/pegasus/Geometry.hpp
)
set(PEGASUS_SOURCES
    sources/Math.cpp
    sources/Object.cpp
    sources/Integration.cpp
    sources/Collision.cpp
    sources/ParticleForceGenerator.cpp
    sources/Scene.cpp
    sources/BoundingVolumes.cpp
    sources/Geometry.cpp
)

set(PEGASUS_EXTRA)

if (MSVC)
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/PegasusVersionInfo.rc.in
        ${CMAKE_CURRENT_BINARY_DIR}/PegasusVersionInfo.rc

        @ONLY
    )

    set(PEGASUS_EXTRA
        ${PEGASUS_EXTRA}
        ${CMAKE_CURRENT_BINARY_DIR}/PegasusVersionInfo.rc
    )
endif()

add_library(${PEGASUS_LIB}
    ${PEGASUS_HEADERS}
    ${PEGASUS_SOURCES}
    ${PEGASUS_EXTRA}
)

set_target_properties(
    ${PEGASUS_LIB}
    PROPERTIES
        VERSION ${PEGASUS_VERSION}
        SOVERSION ${PEGASUS_SOVERSION}
)

target_link_libraries(${PEGASUS_LIB})

install( DIRECTORY ${GLM_INCLUDE_DIR}/glm
    DESTINATION ${PEGASUS_INSTALL_INCLUDE_DIR}
    COMPONENT pegasus_extra
    FILES_MATCHING
        PATTERN "*.hpp"
            PERMISSIONS OWNER_WRITE OWNER_READ GROUP_READ WORLD_READ
        PATTERN "*.h"
            PERMISSIONS OWNER_WRITE OWNER_READ GROUP_READ WORLD_READ
        PATTERN "*.inl"
            PERMISSIONS OWNER_WRITE OWNER_READ GROUP_READ WORLD_READ
)

install( TARGETS ${PEGASUS_LIB}
    COMPONENT pegasus_libraries
    RUNTIME DESTINATION ${PEGASUS_INSTALL_LIBRARY_DIR}
    LIBRARY DESTINATION ${PEGASUS_INSTALL_LIBRARY_DIR}
    ARCHIVE DESTINATION ${PEGASUS_INSTALL_LIBRARY_DIR}
)

install( DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
    DESTINATION ${PEGASUS_INSTALL_INCLUDE_DIR}
    COMPONENT pegasus_headers
    FILES_MATCHING PATTERN "*.hpp"
    PERMISSIONS OWNER_WRITE OWNER_READ GROUP_READ WORLD_READ
)

include(CMakePackageConfigHelpers)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/PegasusPhysicsConfigVersion.cmake
    VERSION ${PEGASUS_VERSION}
    COMPATIBILITY SameMajorVersion
)

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/PegasusPhysicsConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/PegasusPhysicsConfig.cmake

    @ONLY
)

install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/PegasusPhysicsConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/PegasusPhysicsConfigVersion.cmake
    DESTINATION ${PEGASUS_INSTALL_LIBRARY_DIR}
    COMPONENT pegasus_headers
)
