# Copyright (C) 2017 by Godlike
# This code is licensed under the MIT license (MIT)
# (http://opensource.org/licenses/MIT)

set(PEGASUS_NAME "PegasusPhysics" CACHE STRING "Pegasus project name.")

if(NOT DEFINED PEGASUS_ROOT)
    set(PEGASUS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}" CACHE STRING "Pegasus root directory.")
endif()
    
set(PEGASUS_INCLUDE_DIR ${PEGASUS_ROOT} CACHE STRING "Pegasus include directory.")
set(PEGASUS_LIB ${PEGASUS_NAME} CACHE STRING "Pegasus library name.")
