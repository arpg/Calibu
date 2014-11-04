project( APRIL )
cmake_minimum_required(VERSION 2.8)

list( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules/" )

set( CMAKE_CXX_FLAGS "-O3 -std=c++11" )

#find_package( Calibu REQUIRED )
#find_package( EIGEN3 REQUIRED )
find_package( HAL REQUIRED )
find_package( apriltags REQUIRED )

include_directories(
#    ${Calibu_INCLUDE_DIRS}
    ${HAL_INCLUDE_DIRS}
    ${apriltags_INCLUDE_DIRS}
    )

add_executable( ${PROJECT_NAME} main.cpp )

target_link_libraries( ${PROJECT_NAME}
#  ${Calibu_LIBRARIES}
  ${HAL_LIBRARIES}
  ${apriltags_LIBRARIES}
  )
