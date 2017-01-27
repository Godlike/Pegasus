cmake_minimum_required (VERSION 3.1)
project (PegasDemo)

set (CMAKE_CXX_STANDARD 11)

include_directories (
	${CMAKE_CURRENT_SOURCE_DIR} 
	../..
)

set(HEADERS
        ../include/core.hpp
        ../include/particle.hpp
        ../include/math.hpp
)

set(SOURCES
	../src/core.cpp
	../src/particle.cpp
	../src/math.cpp
	./main.cpp
)

add_executable (${PROJECT_NAME} ${SOURCES} ${HEADERS})
