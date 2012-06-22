#######################################################
## Sources

SET(
  SOURCES
  hungarian.h hungarian.cpp
  camera.h camera.cpp
  conics.h conics.cpp
  find_conics.h
  ransac.h
  rectangle.h
  adaptive_threshold.h
  label.h label.cpp
  target.h target.cpp
  utils.h utils.cpp
  drawing.h drawing.cpp
  tracker.h tracker.cpp
  pnp.h pnp.cpp
)

#######################################################
## Find required dependencies

FIND_PACKAGE(Boost 1.40 COMPONENTS thread filesystem REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)

# git://github.com/stevenlovegrove/Sophus.git
FIND_PACKAGE(Sophus REQUIRED)
INCLUDE_DIRECTORIES( ${Sophus_INCLUDE_DIR} )
LINK_DIRECTORIES( ${Sophus_LIBRARY_DIRS} )
LINK_LIBRARIES(${Sophus_LIBRARIES})


LINK_DIRECTORIES(
  ${Boost_LIBRARY_DIRS}
)

INCLUDE_DIRECTORIES(
  ${OpenCV_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)

LINK_LIBRARIES(
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS}
  lapack toontag
)

ADD_LIBRARY(${LIBRARY_NAME} SHARED ${SOURCES})

# Export config file for easy inclusion into other projects: FIND_PACKAGE( Fiducials )
CONFIGURE_FILE( ${CMAKE_CURRENT_SOURCE_DIR}/FiducialsConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/FiducialsConfig.cmake @ONLY IMMEDIATE )
EXPORT( PACKAGE Fiducials )
