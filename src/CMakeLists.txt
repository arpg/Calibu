#######################################################
## Sources

set( INCDIR "../include/calibu" )

set(
    HEADERS
    ${INCDIR}/calib/AutoDiffArrayCostFunction.h
    ${INCDIR}/calib/Calibrator.h
    ${INCDIR}/calib/CostFunctionAndParams.h
    ${INCDIR}/calib/CeresExtra.h
    ${INCDIR}/calib/LocalParamSe3.h
    ${INCDIR}/cam/CameraUtils.h
    ${INCDIR}/cam/CameraModelInterface.h
    ${INCDIR}/cam/CameraModelT.h
    ${INCDIR}/cam/CameraModel.h
    ${INCDIR}/cam/CameraRig.h
    ${INCDIR}/cam/CameraXml.h
    ${INCDIR}/cam/DistortionModel.h
    ${INCDIR}/cam/ProjectionModel.h
    ${INCDIR}/cam/StereoRectify.h
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
    ${INCDIR}/target/RandomGrid.h
    ${INCDIR}/target/Target.h
    ${INCDIR}/target/TargetGridDot.h
    ${INCDIR}/target/TargetRandomDot.h
    ${INCDIR}/utils/Rectangle.h
    ${INCDIR}/utils/Range.h
    ${INCDIR}/utils/Utils.h
    ${INCDIR}/utils/PlaneBasis.h
    ${INCDIR}/utils/StreamOperatorsEigen.h
    ${INCDIR}/utils/StreamOperatorsSophus.h
    ${INCDIR}/utils/Xml.h
)

SET(
    SOURCES
    cam/CameraXml.cpp
    cam/Rectify.cpp
    cam/StereoRectify.cpp
    conics/Conic.cpp
    conics/ConicFinder.cpp
    conics/FindConics.cpp
    image/ImageProcessing.cpp
    image/Label.cpp
    pose/Pnp.cpp
    pose/Tracker.cpp
    target/Hungarian.cpp
    target/RandomGrid.cpp
    target/TargetGridDot.cpp
    target/TargetRandomDot.cpp
    utils/Utils.cpp
)

#######################################################
## Find required dependencies

# git://github.com/stevenlovegrove/Sophus.git
find_package( Eigen3 REQUIRED )
find_package( Sophus REQUIRED )
find_package( OpenCV REQUIRED )
find_package( ZLIB REQUIRED )

## Apply project include directories
list( APPEND USER_INC  "${PROJECT_SOURCE_DIR}/include;${CMAKE_CURRENT_BINARY_DIR}/include" )
list( APPEND USER_INC   ${EIGEN3_INCLUDE_DIR} )
list( APPEND USER_INC   ${Sophus_INCLUDE_DIR} )

list( APPEND LINK_LIBS  ${OpenCV2_LIBRARIES} ${ZLIB_LIBRARIES} )
list( APPEND INTERNAL_INC ${OpenCV2_INCLUDE_DIRS} ${ZLIB_INCLUDE_DIR} )

# Include source/build directories first to preempt any installed Calibu
include_directories( ${USER_INC} )
include_directories( ${INTERNAL_INC} )

add_library( ${LIBRARY_NAME} ${HEADERS} ${SOURCES} )
target_link_libraries( ${LIBRARY_NAME} ${LINK_LIBS} )

#######################################################
## Create configure file for inclusion in library

IF( ${CMAKE_SYSTEM_NAME} MATCHES "Darwin" )
   SET( _OSX_ 1 )
ENDIF()

configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/config.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/include/${LIBRARY_NAME}/config.h"
)

#######################################################
## Install headers / targets

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/include/${LIBRARY_NAME}/config.h"
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${LIBRARY_NAME}
)
install(DIRECTORY ${INCDIR}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include
)
install(TARGETS ${LIBRARY_NAME}
  EXPORT CalibuTargets
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

#######################################################
# Figure out the relative path from the installed Config.cmake file to the
# install prefix. This relative path will be configured into CalibuConfig.cmake.
set( CMAKECONFIG_INSTALL_DIR lib/cmake/Calibu )
file( RELATIVE_PATH REL_INCLUDE_DIR
    "${CMAKE_INSTALL_PREFIX}/${CMAKECONFIG_INSTALL_DIR}"
    "${CMAKE_INSTALL_PREFIX}/include" )

# Add package to CMake package registery for use from the build tree
export( TARGETS ${LIBRARY_NAME}
        FILE "${CMAKE_CURRENT_BINARY_DIR}/CalibuTargets.cmake" )

if(NOT ANDROID)
        # Add package to CMake package registery for use from the build tree
        export( PACKAGE Calibu )
endif()

# Install the export set for use with the install tree
install( EXPORT CalibuTargets DESTINATION ${CMAKECONFIG_INSTALL_DIR} )

## Export config file for easy inclusion into other projects: FIND_PACKAGE( Calibu )
# ... for the build tree
CONFIGURE_FILE( ${CMAKE_CURRENT_SOURCE_DIR}/CalibuConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/CalibuConfig.cmake @ONLY IMMEDIATE )
# ... for the install tree
set( USER_INC "\${CALIBU_CMAKE_DIR}/${REL_INCLUDE_DIR}" )
configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/CalibuConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/CalibuConfig.cmake @ONLY )

INSTALL(FILES "${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/CalibuConfig.cmake"
        DESTINATION ${CMAKECONFIG_INSTALL_DIR} )
