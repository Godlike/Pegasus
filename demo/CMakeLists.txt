cmake_minimum_required (VERSION 3.1)
project (PegasDemo)

set (CMAKE_CXX_STANDARD 17)

#########################################################
# ADD FreeGLUT
#########################################################
set(FREEGLUT_BUILD_DEMOS OFF)
set(FREEGLUT_BUILD_SHARED_LIBS OFF)
add_subdirectory(
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/FreeGLUT/freeglut/freeglut 
    ${CMAKE_CURRENT_BINARY_DIR}/third_party/FreeGLUT/freeglut/freeglut
)
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/FreeGLUT/freeglut/freeglut/include
)

#########################################################
# FIND OPENGL
#########################################################
find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
add_definitions(${OpenGL_DEFINITIONS})
if(NOT OPENGL_FOUND)
    message(ERROR " OPENGL not found!")
endif(NOT OPENGL_FOUND)

#########################################################
# Include Files
#########################################################

include_directories (
    ${CMAKE_CURRENT_SOURCE_DIR}
    ../../.
)

set(HEADERS
    ../include/core.hpp
    ../include/particle.hpp
    ../include/particleforcegenerator.hpp
    ../include/particlecontacts.hpp
    ../include/particlelinks.hpp
    ../include/particleworld.hpp
    ../include/math.hpp
    ../demo/app.hpp
    ../demo/timing.hpp
    ../demo/random.hpp
    ../demo/ogl_headers.hpp
)

set(SOURCES
    ../src/core.cpp
    ../src/particle.cpp
    ../src/particleforcegenerator.cpp
    ../src/particlecontacts.cpp
    ../src/particlelinks.cpp
    ../src/particleworld.cpp
    ../src/math.cpp
    ../demo/app.cpp
    ../demo/timing.cpp
    ../demo/random.cpp
    ../demo/blob.cpp
    ./main.cpp
)

########################################################
# Linking
#########################################################
add_executable (${PROJECT_NAME} ${SOURCES} ${HEADERS})

target_link_libraries(${PROJECT_NAME} 
    ${OPENGL_LIBRARIES} 
    freeglut_static
)
