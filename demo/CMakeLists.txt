cmake_minimum_required (VERSION 3.1)
project (PegasusDemo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

#########################################################
# INCLUDE FreeGLUT
#########################################################
include(GlutConfig)
include_directories(${GLUT_INCLUDE_DIRS})
add_subdirectory(${GLUT_PATH} ${GLUT_LIBRARY_DIRS})
set(GLUT_LIBRARIES freeglut_static)

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
set(PEGASUS_INCLUDE_DIRS
    ../../.
)

include_directories (
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PEGASUS_INCLUDE_DIRS}
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
    ${GLUT_LIBRARIES}
)
