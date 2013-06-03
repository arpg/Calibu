project("Calibu")

cmake_minimum_required( VERSION 2.8 )
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")

option( BUILD_APPLICATIONS "Build Applications" ON )

if(NOT ANDROID)
    option(BUILD_SHARED_LIBS "Build Shared Library" ON)
else()
    set(BUILD_SHARED_LIBS OFF)
    set( CMAKE_CXX_FLAGS "-llog ${CMAKE_CXX_FLAGS}" )
endif()

set( CMAKE_CXX_FLAGS "-std=c++0x -Wall ${CMAKE_CXX_FLAGS}" )

# Overide with cmake -DCMAKE_BUILD_TYPE=Debug {dir}
if( NOT CMAKE_BUILD_TYPE )
   set( CMAKE_BUILD_TYPE Release )
endif()

string( TOLOWER ${PROJECT_NAME} LIBRARY_NAME )

add_subdirectory( src )

if( BUILD_APPLICATIONS )
    add_subdirectory( applications )
    add_subdirectory( examples )
endif()

