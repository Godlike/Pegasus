# Copyright (C) 2017-2018 by Godlike
# This code is licensed under the MIT license (MIT)
# (http://opensource.org/licenses/MIT)

cmake_minimum_required (VERSION 3.1)

include(PegasusConfig)
include(PegasusBuildFlags)

project(${PEGASUS_NAME})

set(PEGASUS_LIB_ALIAS Pegasus::Physics)

add_library(${PROJECT_NAME} "")
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
add_library(${PEGASUS_LIB_ALIAS} ALIAS ${PROJECT_NAME})

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build (Debug or Release)" FORCE)
endif()

if (NOT GLM_FOUND)
    add_subdirectory(${GLM_ROOT_DIR})
endif()

if (NOT ARION_FOUND)
    set(ARION_BUILD_DEMOS OFF CACHE BOOL "Build Arion demo.")
    set(ARION_BUILD_TESTS OFF CACHE BOOL "Build Arion tests.")
    set(ARION_BUILD_DEBUG ON  CACHE BOOL "Build Arion debug.")
    add_subdirectory(${PEGASUS_ROOT}/Pegasus/godlike/Arion)
    add_definitions(${ARION_DEFINITIONS})
endif()

#Target
include_directories(
    ${PEGASUS_INCLUDE_DIR}
)

message(STATUS "Version: ${PEGASUS_VERSION}")

set(PEGASUS_HEADERS
    include/pegasus/Debug.hpp
    include/pegasus/debug/DebugDummy.hpp
    include/pegasus/debug/DebugImplementation.hpp
    include/pegasus/Body.hpp
    include/pegasus/Force.hpp
    include/pegasus/Integration.hpp
    include/pegasus/Asset.hpp
    include/pegasus/AssetManager.hpp
    include/pegasus/Scene.hpp
    include/pegasus/Primitives.hpp
    include/pegasus/Material.hpp
    include/pegasus/Contact.hpp
    include/pegasus/CollisionResolver.hpp
    include/pegasus/CollisionDetector.hpp
)
set(PEGASUS_SOURCES
    sources/DebugDummy.cpp
    sources/DebugImplementation.cpp
    sources/Body.cpp
    sources/Force.cpp
    sources/Integration.cpp
    sources/Asset.cpp
    sources/Scene.cpp
    sources/Primitives.cpp
    sources/Material.cpp
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

target_sources(${PROJECT_NAME}
    PRIVATE
        ${PEGASUS_HEADERS}
        ${PEGASUS_SOURCES}
        ${PEGASUS_EXTRA}
)

target_compile_definitions(${PEGASUS_LIB}
    PUBLIC
        ${PEGASUS_DEFINITIONS}
)

target_include_directories(${PEGASUS_LIB}
    PUBLIC
        $<BUILD_INTERFACE:${PEGASUS_INCLUDE_DIR}>
)

set_target_properties(
    ${PROJECT_NAME}
    PROPERTIES
        VERSION ${PEGASUS_VERSION}
        SOVERSION ${PEGASUS_SOVERSION}
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        Arion::Collision
)

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
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    VERSION ${PEGASUS_VERSION}
    COMPATIBILITY SameMajorVersion
)

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}Config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake.gen

    @ONLY
)

file(GENERATE
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    INPUT ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake.gen
)

install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    DESTINATION ${PEGASUS_INSTALL_LIBRARY_DIR}
    COMPONENT pegasus_headers
)
