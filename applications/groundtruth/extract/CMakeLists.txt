project( extract )
cmake_minimum_required(VERSION 2.8)

list( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules/" )

set( CMAKE_CXX_FLAGS "-std=c++11" )

#find_package( Calibu REQUIRED )
find_package( EIGEN3 REQUIRED )
find_package( HAL 0.1 REQUIRED )
find_package(PbMsgs 2.0 QUIET)

find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIR})


include_directories(
#    ${Calibu_INCLUDE_DIRS}
    ${PbMsgs_INCLUDE_DIRS}
    ${HAL_INCLUDE_DIRS}
    )

add_executable( ${PROJECT_NAME} main.cpp )

target_link_libraries( ${PROJECT_NAME}
#  ${Calibu_LIBRARIES}
  ${HAL_LIBRARIES}
  ${PbMsgs_LIBRARIES}
  ${PROTOBUF_LIBRARY}
  )
