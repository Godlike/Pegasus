# Copyright (C) 2017 by Godlike
# This code is licensed under the MIT license (MIT)
# (http://opensource.org/licenses/MIT)

set(PEGASUS_NAME "PegasusPhysics" CACHE STRING "Pegasus project name.")

if(NOT DEFINED PEGASUS_ROOT)
    set(PEGASUS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}" CACHE STRING "Pegasus root directory.")
endif()

list(APPEND CMAKE_MODULE_PATH "${PEGASUS_ROOT}/Pegasus/cmake")

include(GlmConfig)
    
set(PEGASUS_INCLUDE_DIR
    ${PEGASUS_ROOT}
    ${GLM_INCLUDE_DIR}
    CACHE LIST "Pegasus include directories."
)
set(PEGASUS_LIB ${PEGASUS_NAME} CACHE STRING "Pegasus library name.")
