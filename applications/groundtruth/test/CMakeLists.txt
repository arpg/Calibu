project( Synthetic )
cmake_minimum_required(VERSION 2.8)

list( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules/" )

set( CMAKE_CXX_FLAGS "-std=c++11" )

find_package( EIGEN3 REQUIRED )
find_package( HAL 0.1 REQUIRED )
find_package( SceneGraph REQUIRED )
find_package( PbMsgs 2.0 QUIET )
find_package( Pangolin REQUIRED )
find_package( Protobuf REQUIRED)
find_package( Calibu REQUIRED )

include_directories(
    ${PROTOBUF_INCLUDE_DIR}
    ${SceneGraph_INCLUDE_DIRS}
    ${PbMsgs_INCLUDE_DIRS}
    ${HAL_INCLUDE_DIRS}
    ${Calibu_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
    )

add_executable( ${PROJECT_NAME} main.cpp codes.h )

target_link_libraries( ${PROJECT_NAME}
  ${HAL_LIBRARIES}
  ${Pangolin_LIBRARIES}
  ${SceneGraph_LIBRARIES}
  ${PbMsgs_LIBRARIES}
  ${PROTOBUF_LIBRARY}
  ${Calibu_LIBRARIES}
  )
