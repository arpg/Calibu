project(Calibu)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/)
include(def_library)
include(export_package)

set(CALIBU_VERSION_MAJOR 0)
set(CALIBU_VERSION_MINOR 1)
set(CALIBU_VERSION ${CALIBU_VERSION_MAJOR}.${CALIBU_VERSION_MINOR})

option(BUILD_APPLICATIONS "Build Applications" ON)
option(BUILD_SHARED_LIBS "Build Shared Library" ON)

set(CMAKE_CXX_FLAGS "-std=c++0x -Wall ${CMAKE_CXX_FLAGS}")
if(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
endif()

# Overide with cmake -DCMAKE_BUILD_TYPE=Debug {dir}
if( NOT CMAKE_BUILD_TYPE )
   set( CMAKE_BUILD_TYPE Release )
endif()

string(TOLOWER ${PROJECT_NAME} LIBRARY_NAME)
add_subdirectory(src)

include_directories(${EXPORT_CALIBU_INC})

def_library(calibu
  SOURCES ${CALIBU_HEADERS} ${CALIBU_SOURCES}
  LINK_LIBS ${CALIBU_LINK_LIBS}
  PACKAGE Calibu
  )

export_package(Calibu
  TARGETS ${LIBRARY_NAME}
  VERSION ${CALIBU_VERSION}
  LIBRARY ${LIBRARY_NAME}
  LIBRARIES ${LIBRARY_NAME}
  INCLUDE_DIRS ${EXPORT_CALIBU_INC}
  )

if( BUILD_APPLICATIONS )
  set(Calibu_DIR ${Calibu_BINARY_DIR})
  add_subdirectory( applications )
  add_subdirectory( examples )
endif()

