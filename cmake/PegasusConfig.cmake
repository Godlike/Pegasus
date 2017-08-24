# Copyright (C) 2017 by Godlike
# This code is licensed under the MIT license (MIT)
# (http://opensource.org/licenses/MIT)

set(PEGASUS_NAME "PegasusPhysics" CACHE STRING "Pegasus project name.")

if(NOT DEFINED PEGASUS_ROOT)
    set(PEGASUS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}" CACHE STRING "Pegasus root directory.")
endif()

list(APPEND CMAKE_MODULE_PATH "${PEGASUS_ROOT}/Pegasus/cmake")

include(GlmConfig)

#GLM
find_package(GLM)

set(PEGASUS_INCLUDE_DIR
    ${PEGASUS_ROOT}
    ${PEGASUS_ROOT}/Pegasus/include
    ${GLM_INCLUDE_DIR}
    CACHE LIST "Pegasus include directories."
)

if (NOT DEFINED INSTALL_INCLUDE_DIR)
    set(INSTALL_INCLUDE_DIR "include/godlike" CACHE STRING "Path to directory holding headers")
endif()

if (NOT DEFINED INSTALL_LIBRARY_DIR)
    set(INSTALL_LIBRARY_DIR "lib" CACHE STRING "Path to directory holding libraries")
endif()

set(PEGASUS_LIB ${PEGASUS_NAME} CACHE STRING "Pegasus library name.")

set(PEGASUS_INSTALL_INCLUDE_DIR ${INSTALL_INCLUDE_DIR})
set(PEGASUS_INSTALL_LIBRARY_DIR ${INSTALL_LIBRARY_DIR}/${PEGASUS_NAME})

set(PEGASUS_VERSION_MAJOR 0)
set(PEGASUS_VERSION_MINOR 1)
set(PEGASUS_VERSION_PATCH 0)

set(PEGASUS_VERSION "${PEGASUS_VERSION_MAJOR}.${PEGASUS_VERSION_MINOR}.${PEGASUS_VERSION_PATCH}")
set(PEGASUS_SOVERSION "${PEGASUS_VERSION_MAJOR}.${PEGASUS_VERSION_MINOR}")

if (BUILD_SHARED_LIBS)
    add_definitions(-DPEGASUS_SHARED)
endif()
