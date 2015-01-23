project( GroundTruth )
cmake_minimum_required(VERSION 2.8)
set( MACOSX_RPATH OFF )
list( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules/" )

set( CMAKE_CXX_FLAGS "-O3 -std=c++11 -lprotobuf" )

#set( CMAKE_C_FLAGS "-g -Wall -Wextra" )
#set( CMAKE_CXX_FLAGS "-g -Wall -Wextra -std=c++11" )
#set( CMAKE_C_FLAGS "-O3 -funroll-loops -msse4 -mavx -ffast-math -Wall -Wextra" )
#set( CMAKE_CXX_FLAGS "-O3 -funroll-loops -msse4 -mavx -ffast-math -Wall -Wextra -std=c++11" )
add_subdirectory( ImageAlign )

find_package( PbMsgs REQUIRED )
find_package( Calibu REQUIRED )
find_package( EIGEN3 REQUIRED )
find_package( OpenCV2 REQUIRED )
find_package( Ceres REQUIRED )
find_package( gflags REQUIRED )
find_package( Sophus REQUIRED )
find_package( Protobuf REQUIRED )
find_package( SceneGraph REQUIRED )
find_package( HAL REQUIRED )
find_package( Pangolin REQUIRED )

find_package( apriltags REQUIRED )

include_directories(
    ${SceneGraph_INCLUDE_DIRS}
    ${PbMsgs_INCLUDE_DIRS}
    ${Calibu_INCLUDE_DIRS}
    ${Sophus_INCLUDE_DIR}
    ${PROTOBUF_INCLUDE_DIR}
    ${GFlags_INCLUDE_DIRS} 
    ${OpenCV2_INCLUDE_DIR} 
    ${EIGEN3_INCLUDE_DIR} 
    ${CERES_INCLUDE_DIRS}
    ${HAL_INCLUDE_DIRS}
    ${apriltags_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
    )

add_executable( ${PROJECT_NAME} main.cpp measurements.h
                                math.h usage.h tags.h usage.h
                                dense_ceres.h dtrack.h hess.h
                                pfromh.h ceres_cost_functions.h
                                compute_homography.h )

target_link_libraries( ${PROJECT_NAME}
  ${PbMsgs_LIBRARIES}
  ${OpenCV2_LIBRARIES}
  ${CERES_LIBRARIES}
  ${GFlags_LIBRARIES}
  ${Calibu_LIBRARIES}
  ${HAL_LIBRARIES}
  ${apriltags_LIBRARIES}
  ${Pangolin_LIBRARIES}
  ${SceneGraph_LIBRARIES}
  ${PROTOBUF_LIBRARY}
  )
target_link_libraries( ${PROJECT_NAME} ImageAlign )
