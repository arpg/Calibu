#######################################################
## Sources

set( INCDIR "../include/fiducials")

set(
    HEADERS
    ${INCDIR}/calib/AutoDiffArrayCostFunction.h
    ${INCDIR}/calib/Calibrator.h
    ${INCDIR}/calib/CostFunctionAndParams.h
    ${INCDIR}/calib/EigenCeresJetNumTraits.h
    ${INCDIR}/calib/LocalParamSe3.h
    ${INCDIR}/cam/CameraModel.h
    ${INCDIR}/cam/CameraModelBase.h
    ${INCDIR}/cam/DistortionModel.h
    ${INCDIR}/cam/ProjectionModel.h
    ${INCDIR}/conics/Conic.h
    ${INCDIR}/conics/ConicFinder.h
    ${INCDIR}/conics/FindConics.h
    ${INCDIR}/gl/Drawing.h
    ${INCDIR}/image/AdaptiveThreshold.h
    ${INCDIR}/image/Gradient.h
    ${INCDIR}/image/ImageProcessing.h
    ${INCDIR}/image/IntegralImage.h
    ${INCDIR}/image/Label.h
    ${INCDIR}/pose/Pnp.h
    ${INCDIR}/pose/Ransac.h
    ${INCDIR}/pose/Tracker.h
    ${INCDIR}/target/Hungarian.h
    ${INCDIR}/target/LineGroup.h
    ${INCDIR}/target/Target.h
    ${INCDIR}/target/TargetGridDot.h
    ${INCDIR}/target/TargetRandomDot.h
    ${INCDIR}/utils/Rectangle.h
    ${INCDIR}/utils/Utils.h
)

SET(
    SOURCES
    conics/Conic.cpp
    conics/ConicFinder.cpp
    conics/FindConics.cpp
    gl/Drawing.cpp
    image/ImageProcessing.cpp
    image/Label.cpp
    pose/Pnp.cpp
    pose/Tracker.cpp
    target/Hungarian.cpp
    target/TargetGridDot.cpp
    target/TargetRandomDot.cpp
    utils/Utils.cpp
)

#######################################################
## Find required dependencies

find_package(OpenGL REQUIRED QUIET)
find_package(Boost COMPONENTS thread filesystem REQUIRED QUIET)

## Apply required dependencies settings
list(APPEND USER_INC  "${PROJECT_SOURCE_DIR}/include;${CMAKE_CURRENT_BINARY_DIR}/include" )
list(APPEND USER_INC  "${OPENGL_INCLUDE_DIR};${Boost_INCLUDE_DIRS};${PROJECT_BINARY_DIR}" )
list(APPEND LINK_LIBS "${OPENGL_LIBRARIES};${Boost_LIBRARIES}" )
list(APPEND LIB_DIRS  "${OPENGL_LIBRARY_DIR};${Boost_LIBRARY_DIRS}" )

# git://github.com/stevenlovegrove/Sophus.git
find_package(Sophus REQUIRED)
list(APPEND LINK_LIBS ${Sophus_LIBRARIES} )
list(APPEND USER_INC  ${Sophus_INCLUDE_DIR} )
list(APPEND LIB_DIRS  ${Sophus_LIBRARY_DIRS} )

find_package(OpenCV REQUIRED)
list(APPEND LINK_LIBS ${OpenCV2_LIBRARIES} )
list(APPEND INTERNAL_INC ${OpenCV2_INCLUDE_DIRS} )

find_package(Eigen3 REQUIRED)
list(APPEND USER_INC  ${EIGEN3_INCLUDE_DIR} )

include_directories( ${INTERNAL_INC} )
include_directories( ${USER_INC} )
link_libraries(${LINK_LIBS})
link_directories(${LIB_DIRS})

add_library(${LIBRARY_NAME} SHARED ${SOURCES})

#######################################################
## Create configure file for inclusion in library

IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
   SET(_OSX_ 1)
ENDIF()

configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/config.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/include/${LIBRARY_NAME}/config.h"
)

#######################################################
## Export config file for easy inclusion into other projects: FIND_PACKAGE( Fiducials )
get_target_property( LIB_LOC ${LIBRARY_NAME} LOCATION )
CONFIGURE_FILE( ${CMAKE_CURRENT_SOURCE_DIR}/FiducialsConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/FiducialsConfig.cmake @ONLY IMMEDIATE )
EXPORT( PACKAGE Fiducials )
