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
  ${INC_DIR}/image/Threshold.h
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
  ${INC_DIR}/target/TargetViconGridDot.h
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
  ${SRC_DIR}/target/TargetViconGridDot.cpp
  ${SRC_DIR}/utils/Utils.cpp
  ${SRC_DIR}/utils/Xml.cpp
  )

#######################################################
## Find required dependencies

find_package( Eigen3 REQUIRED )

# https://github.com/gwu-robotics/Sophus.git
find_package( Sophus REQUIRED )

## Apply project include directories
list( APPEND CALIBU_INC "${CMAKE_CURRENT_SOURCE_DIR}/../include;${CMAKE_CURRENT_BINARY_DIR}/include" )
list( APPEND USER_INC   ${EIGEN3_INCLUDE_DIR} )
list( APPEND USER_INC   ${Sophus_INCLUDE_DIR} )

#######################################################
## Find optional dependencies

# OpenCV is required for PNP methods and calibration application
find_package(OpenCV2)
if( OpenCV2_FOUND )
    set( HAVE_OPENCV 1 )
    list( APPEND LINK_LIBS  ${OpenCV2_LIBRARIES})
    list( APPEND CALIBU_INC ${OpenCV2_INCLUDE_DIRS} )
    list( APPEND HEADERS ${INC_DIR}/pose/Pnp.h ${INC_DIR}/pose/Tracker.h )
    list( APPEND SOURCES ${SRC_DIR}/pose/Pnp.cpp ${SRC_DIR}/pose/Tracker.cpp )
endif()

#######################################################
## Setup and configure library

# Include source/build directories first to preempt any installed Calibu
include_directories( ${CALIBU_INC} )
include_directories( ${USER_INC} )

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

list(APPEND CALIBU_INC ${CMAKE_CURRENT_BINARY_DIR}/include/)

# Configure the include directories for export_package
set(EXPORT_CALIBU_INC ${CALIBU_INC} ${USER_INC} PARENT_SCOPE)
set(CALIBU_HEADERS ${HEADERS} PARENT_SCOPE)
set(CALIBU_SOURCES ${SOURCES} PARENT_SCOPE)
set(CALIBU_LINK_LIBS ${LINK_LIBS} PARENT_SCOPE)

#######################################################
## Install headers

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/include/${LIBRARY_NAME}/config.h"
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${LIBRARY_NAME}
)
install(DIRECTORY ${INC_DIR}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include
)
