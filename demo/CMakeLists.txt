cmake_minimum_required (VERSION 3.1)
project (PegasDemo)

set (CMAKE_CXX_STANDARD 11)

#########################################################
# FIND GLUT
#########################################################
find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})
add_definitions(${GLUT_DEFINITIONS})
if(NOT GLUT_FOUND)
    message(ERROR " GLUT not found!")
endif(NOT GLUT_FOUND)

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
# FIND EIGEN
#########################################################
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
if(NOT EIGEN3_FOUND)
    message(ERROR " EIGEN3 not found!")
endif(NOT EIGEN3_FOUND)


#########################################################
# Include Files
#########################################################
include_directories (
    ${CMAKE_CURRENT_SOURCE_DIR}
    ../..
)

set(HEADERS
    ../include/core.hpp
    ../include/particle.hpp
    ../include/math.hpp
    ../include/obb.hpp
    ../demo/app.hpp
    ../demo/ogl_headers.hpp
    ../demo/timing.hpp
)

set(SOURCES
    ../src/core.cpp
    ../src/particle.cpp
    ../src/math.cpp
    ../src/obb.cpp
    ../demo/app.cpp
    ../demo/timing.cpp
    ../demo/ballistic.cpp
    ../demo/main.cpp
)

########################################################
# Linking
#########################################################
add_executable (${PROJECT_NAME} ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})
