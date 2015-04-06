project(Calibu)
cmake_minimum_required(VERSION 2.8)

if(POLICY CMP0042)
  cmake_policy(SET CMP0042 NEW) # MACOSX_RPATH
endif()


set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/)
include( install_package )

set(CALIBU_VERSION_MAJOR 0)
set(CALIBU_VERSION_MINOR 2)
set(CALIBU_VERSION ${CALIBU_VERSION_MAJOR}.${CALIBU_VERSION_MINOR})

option(BUILD_APPLICATIONS "Build Applications" ON)
option(BUILD_MATLAB "Build Matlab Wrappers" OFF)
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
 
#add_subdirectory(src)
#######################################################
## Sources
set( INC_DIR ${Calibu_SOURCE_DIR}/include/calibu )
set(HEADERS
  ${INC_DIR}/Calibu.h
  ${INC_DIR}/Platform.h
  ${INC_DIR}/calib/AutoDiffArrayCostFunction.h
  ${INC_DIR}/calib/Calibrator.h
  ${INC_DIR}/calib/CostFunctionAndParams.h
  ${INC_DIR}/calib/LocalParamSe3.h
  ${INC_DIR}/cam/camera_crtp.h
  ${INC_DIR}/cam/camera_models_crtp.h
  ${INC_DIR}/cam/camera_crtp_interop.h
  ${INC_DIR}/cam/CameraModel.h
  ${INC_DIR}/cam/CameraModelInterface.h
  ${INC_DIR}/cam/CameraModelT.h
  ${INC_DIR}/cam/CameraRig.h
  ${INC_DIR}/cam/CameraUtils.h
  ${INC_DIR}/cam/CameraXml.h
  ${INC_DIR}/cam/DistortionModel.h
  ${INC_DIR}/cam/ProjectionModel.h
  ${INC_DIR}/cam/ProjectionKannalaBrandt.h
  ${INC_DIR}/cam/Rectify.h
  ${INC_DIR}/cam/rectify_crtp.h
  ${INC_DIR}/cam/StereoRectify.h
  ${INC_DIR}/conics/Conic.h
  ${INC_DIR}/conics/ConicFinder.h
  ${INC_DIR}/conics/FindConics.h
  ${INC_DIR}/gl/Drawing.h
  ${INC_DIR}/image/AdaptiveThreshold.h
  ${INC_DIR}/image/Gradient.h
  ${INC_DIR}/image/ImageProcessing.h
  ${INC_DIR}/image/IntegralImage.h
  ${INC_DIR}/image/Label.h
  ${INC_DIR}/pose/Ransac.h
  ${INC_DIR}/target/Hungarian.h
  ${INC_DIR}/target/LineGroup.h
  ${INC_DIR}/target/RandomGrid.h
  ${INC_DIR}/target/Target.h
  ${INC_DIR}/target/TargetGridDot.h
  ${INC_DIR}/target/TargetRandomDot.h
  ${INC_DIR}/utils/Rectangle.h
  ${INC_DIR}/utils/Range.h
  ${INC_DIR}/utils/Utils.h
  ${INC_DIR}/utils/PlaneBasis.h
  ${INC_DIR}/utils/StreamOperatorsEigen.h
  ${INC_DIR}/utils/StreamOperatorsSophus.h
  ${INC_DIR}/utils/Xml.h
  )

set(SRC_DIR ${Calibu_SOURCE_DIR}/src)
SET(SOURCES
  ${SRC_DIR}/cam/CameraXml.cpp
  ${SRC_DIR}/cam/Rectify.cpp
  ${SRC_DIR}/cam/rectify_crtp.cpp
  ${SRC_DIR}/cam/StereoRectify.cpp
  ${SRC_DIR}/conics/Conic.cpp
  ${SRC_DIR}/conics/ConicFinder.cpp
  ${SRC_DIR}/conics/FindConics.cpp
  ${SRC_DIR}/image/ImageProcessing.cpp
  ${SRC_DIR}/image/Label.cpp
  ${SRC_DIR}/target/Hungarian.cpp
  ${SRC_DIR}/target/RandomGrid.cpp
  ${SRC_DIR}/target/TargetGridDot.cpp
  ${SRC_DIR}/target/TargetRandomDot.cpp
  ${SRC_DIR}/utils/Utils.cpp
  ${SRC_DIR}/utils/Xml.cpp
  )

#######################################################
## Find required dependencies
find_package( Eigen3 REQUIRED )
find_package( Sophus REQUIRED )

## Apply project include directories
list( APPEND CALIBU_INC "${CMAKE_SOURCE_DIR}/include;${CMAKE_CURRENT_BINARY_DIR}/include" )
list( APPEND USER_INC   ${EIGEN3_INCLUDE_DIR} )
list( APPEND USER_INC   ${Sophus_INCLUDE_DIR} )

message( STATUS "Sophus_INCLUDE_DIR ${Sophus_INCLUDE_DIR}" )

#######################################################
## Find optional dependencies

# OpenCV is required for PNP methods and calibration application
find_package( OpenCV2 REQUIRED )
if( OpenCV2_FOUND )
    set( HAVE_OPENCV 1 )
    list( APPEND LINK_LIBS  ${OpenCV2_LIBRARIES})
    list( APPEND USER_INC ${OpenCV2_INCLUDE_DIRS} )
    list( APPEND HEADERS ${INC_DIR}/pose/Pnp.h ${INC_DIR}/pose/Tracker.h )
    list( APPEND SOURCES ${SRC_DIR}/pose/Pnp.cpp ${SRC_DIR}/pose/Tracker.cpp )
endif()

#######################################################
## Setup and configure library
## Generate symbol export helper header on MSVC
IF(MSVC)
    string(TOUPPER ${LIBRARY_NAME} LIBRARY_NAME_CAPS)
    include(GenerateExportHeader)
    GENERATE_EXPORT_HEADER( ${LIBRARY_NAME}
        BASE_NAME ${LIBRARY_NAME_CAPS}
        EXPORT_MACRO_NAME ${LIBRARY_NAME_CAPS}_EXPORT
        EXPORT_FILE_NAME "${CMAKE_CURRENT_BINARY_DIR}/include/${LIBRARY_NAME}/${LIBRARY_NAME}_export.h"
        STATIC_DEFINE ${LIBRARY_NAME_CAPS}_BUILT_AS_STATIC
    )
ENDIF()

#######################################################
## Create configure file for inclusion in library

# Platform configuration vars
## Compiler configuration
include(SetPlatformVars)
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/config.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/include/${LIBRARY_NAME}/config.h"
)
#list( APPEND HEADERS "${CMAKE_CURRENT_BINARY_DIR}/include/${LIBRARY_NAME}/config.h" )

# Include source/build directories first to preempt any installed Calibu
include_directories( ${CALIBU_INC} ${USER_INC} ${CMAKE_CURRENT_BINARY_DIR}/include )

# build calibu library
add_library( calibu ${HEADERS} ${SOURCES} )
target_link_libraries( calibu ${LINK_LIBS} )

set(INSTALL_HEADERS
  Calibu.h
  Platform.h
  calib/AutoDiffArrayCostFunction.h
  calib/Calibrator.h
  calib/CostFunctionAndParams.h
  calib/LocalParamSe3.h
  cam/camera_crtp.h
  cam/camera_models_crtp.h
  cam/camera_crtp_interop.h
  cam/CameraModel.h
  cam/CameraModelInterface.h
  cam/CameraModelT.h
  cam/CameraRig.h
  cam/CameraUtils.h
  cam/CameraXml.h
  cam/DistortionModel.h
  cam/ProjectionModel.h
  cam/ProjectionKannalaBrandt.h
  cam/Rectify.h
  cam/rectify_crtp.h
  cam/StereoRectify.h
  conics/Conic.h
  conics/ConicFinder.h
  conics/FindConics.h
  gl/Drawing.h
  image/AdaptiveThreshold.h
  image/Gradient.h
  image/ImageProcessing.h
  image/IntegralImage.h
  image/Label.h
  pose/Ransac.h
  target/Hungarian.h
  target/LineGroup.h
  target/RandomGrid.h
  target/Target.h
  target/TargetGridDot.h
  target/TargetRandomDot.h
  utils/Rectangle.h
  utils/Range.h
  utils/Utils.h
  utils/PlaneBasis.h
  utils/StreamOperatorsEigen.h
  utils/StreamOperatorsSophus.h
  utils/Xml.h
  )


# install everything
install_package(
    PKG_NAME Calibu
    LIB_NAME calibu
    VERSION 0.2
    DESCRIPTION "Computer vision camera model library."
    INSTALL_HEADERS ${INSTALL_HEADERS}
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/Calibu
    INCLUDE_DIRS ${USER_INC} ${EXPORT_CALIBY_INC}
    LINK_LIBS ${LINK_LIBS} calibu
    )


if( BUILD_APPLICATIONS )
  set( Calibu_DIR ${Calibu_BINARY_DIR} )
#  set( Calibu_INCLUDE_DIRS ${USER_INC} ${CALIBU_INC} )
#  set( Calibu_LIBRARIES ${LINK_LIBS} calibu )

  add_subdirectory( applications )
#  add_subdirectory( examples )
endif()

if( BUILD_MATLAB )
#  add_subdirectory( matlab )
endif()

