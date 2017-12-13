# Copyright (C) 2017 by Godlike
# This code is licensed under the MIT license (MIT)
# (http://opensource.org/licenses/MIT)

cmake_minimum_required(VERSION 3.0)
cmake_policy(VERSION 3.0)

project(PegasusTests)

include_directories(
    ${PEGASUS_INCLUDE_DIR}
    ${CATCH_ROOT}/single_include
)

function(pegasus_add_test)
    set(options )
    set(oneValueArgs NAME)
    set(multiValueArgs SOURCE ARGS DEPENDS)
    cmake_parse_arguments(
        pegasus_add_test
        "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN}
    )

    add_executable(
        ${pegasus_add_test_NAME}_test
        ${pegasus_add_test_SOURCE}
    )

    if (pegasus_add_test_DEPENDS)
        target_link_libraries(${pegasus_add_test_NAME}_test ${pegasus_add_test_DEPENDS})
    endif()

    add_test(
        NAME ${pegasus_add_test_NAME}
        COMMAND ${pegasus_add_test_NAME}_test ${pegasus_add_test_ARGS}
    )
endfunction()

pegasus_add_test(NAME Integration
    SOURCE IntegrationTest.cpp
    DEPENDS ${PEGASUS_LIB}
)
