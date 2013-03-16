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
)

#######################################################
## Find required dependencies

FIND_PACKAGE(OpenGL REQUIRED QUIET)
FIND_PACKAGE(Boost COMPONENTS thread filesystem REQUIRED QUIET)

## Apply required dependencies settings
LIST(APPEND LINK_LIBS "${OPENGL_LIBRARIES};${Boost_LIBRARIES}" )
LIST(APPEND LIB_DIRS  "${OPENGL_LIBRARY_DIR};${Boost_LIBRARY_DIRS};${PROJECT_BINARY_DIR}/fiducials" )
LIST(APPEND USER_INC  "${OPENGL_INCLUDE_DIR};${Boost_INCLUDE_DIRS};${CMAKE_CURRENT_SOURCE_DIR}/..";${PROJECT_BINARY_DIR} )

# git://github.com/stevenlovegrove/Sophus.git
FIND_PACKAGE(Sophus REQUIRED)
LIST(APPEND LINK_LIBS ${Sophus_LIBRARIES} )
LIST(APPEND USER_INC  ${Sophus_INCLUDE_DIR} )
LIST(APPEND LIB_DIRS  ${Sophus_LIBRARY_DIRS} )

FIND_PACKAGE(OpenCV REQUIRED)
LIST(APPEND LINK_LIBS ${OpenCV2_LIBRARIES} )
LIST(APPEND INTERNAL_INC ${OpenCV2_INCLUDE_DIRS} )

FIND_PACKAGE(Eigen3 REQUIRED)
LIST(APPEND USER_INC  ${EIGEN3_INCLUDE_DIR} )

INCLUDE_DIRECTORIES( ${INTERNAL_INC} )
INCLUDE_DIRECTORIES( ${USER_INC} )
LINK_LIBRARIES(${LINK_LIBS})
LINK_DIRECTORIES(${LIB_DIRS})

ADD_LIBRARY(${LIBRARY_NAME} SHARED ${SOURCES})

#######################################################
## Create configure file for inclusion in library

IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
   SET(_OSX_ 1)
ENDIF()

CONFIGURE_FILE(
  "${CMAKE_CURRENT_SOURCE_DIR}/config.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/config.h"
)

#######################################################
## Export config file for easy inclusion into other projects: FIND_PACKAGE( Fiducials )

CONFIGURE_FILE( ${CMAKE_CURRENT_SOURCE_DIR}/FiducialsConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/FiducialsConfig.cmake @ONLY IMMEDIATE )
EXPORT( PACKAGE Fiducials )
