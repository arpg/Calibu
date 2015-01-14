cmake_minimum_required(VERSION 2.8)
project( ImageAlign )

LIST( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules/" )

find_package( OpenCV2 REQUIRED )
find_package( Ceres REQUIRED )
include_directories( ${OpenCV2_INCLUDE_DIRS} )
include_directories( ${CERES_INCLUDE_DIRS} )
INCLUDE_DIRECTORIES( "/Users/faradazerage/code/ThirdParty/vlfeat" )
list( APPEND SRCS compute_homography.cpp )
list( APPEND HDRS compute_homography.h )

add_library( ImageAlign SHARED ${SRCS} ${HDRS} )

target_link_libraries( ImageAlign ${OpenCV2_LIBRARIES} )
target_link_libraries( ImageAlign ${CERES_LIBRARIES} )
target_link_libraries( ImageAlign
  "/Users/faradazerage/code/ThirdParty/vlfeat/bin/maci64/libvl.dylib")


