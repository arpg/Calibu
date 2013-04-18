project("Calibu")

cmake_minimum_required( VERSION 2.8 )
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")

option( BUILD_APPLICATIONS "Build Applications" ON )

set( CMAKE_CXX_FLAGS "-std=c++0x -Wall ${CMAKE_CXX_FLAGS}" )

string( TOLOWER ${PROJECT_NAME} LIBRARY_NAME )

add_subdirectory( src )

if( BUILD_APPLICATIONS )
    add_subdirectory( applications )
endif()

