# Copyright (C) 2017 by Godlike
# This code is licensed under the MIT license (MIT) 
# (http://opensource.org/licenses/MIT)

set(GLM_INSTALL_ENABLE OFF CACHE BOOL "Flag to override default GLM_INSTALL_ENABLE value")
set(GLM_TEST_ENABLE OFF CACHE BOOL "Flag to override default GLM_TEST_ENABLE value")

set(GLM_SOURCE_DIR ${PEGASUS_ROOT}/Pegasus/third_party/glm CACHE STRING "Path to GLM root directory")
set(GLM_INCLUDE_DIR "${GLM_SOURCE_DIR}" CACHE STRING "Path to GLM include directory")
