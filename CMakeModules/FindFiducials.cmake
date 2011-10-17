# - Try to find libFiducials
#
#  Fiducials_FOUND - system has libFiducials
#  Fiducials_INCLUDE_DIR - the libFiducials include directories
#  Fiducials_LIBRARY - link these to use libFiducials

FIND_PATH(
  Fiducials_INCLUDE_DIR_SRC
  NAMES fiducials/target.h
  PATHS
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/../fiducials
    ${CMAKE_SOURCE_DIR}/../Fiducials
    /usr/include
    /usr/local/include
)

FIND_LIBRARY(
  Fiducials_LIBRARY
  NAMES fiducials
  PATHS
    ${CMAKE_SOURCE_DIR}/../release/fiducials
    ${CMAKE_SOURCE_DIR}/../build/fiducials
    ${CMAKE_SOURCE_DIR}/../Fiducials/release/fiducials
    ${CMAKE_SOURCE_DIR}/../Fiducials/build/fiducials
    ${CMAKE_SOURCE_DIR}/../fiducials/release/fiducials
    ${CMAKE_SOURCE_DIR}/../fiducials/build/fiducials
    /usr/lib
    /usr/local/lib
) 

IF(Fiducials_INCLUDE_DIR_SRC AND Fiducials_LIBRARY)
  SET(Fiducials_INCLUDE_DIR ${Fiducials_INCLUDE_DIR_SRC})
  SET(Fiducials_FOUND TRUE)
ENDIF()

IF(Fiducials_FOUND)
   IF(NOT Fiducials_FIND_QUIETLY)
      MESSAGE(STATUS "Found Fiducials: ${Fiducials_LIBRARY}")
   ENDIF()
ELSE()
   IF(Fiducials_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find Fiducials")
   ENDIF()
ENDIF()
