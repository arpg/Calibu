#######################################################
## Nessy CMake build script

PROJECT("ScreenToCamera")

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")

# Overide with cmake -DCMAKE_BUILD_TYPE=Debug {dir}
IF( NOT CMAKE_BUILD_TYPE )
   MESSAGE("Build type not set (defaults to release)")
   MESSAGE("-DCMAKE_BUILD_TYPE=Debug for debug")
   SET(CMAKE_BUILD_TYPE Release)
ENDIF()

# Platform configuration vars
INCLUDE(SetPlatformVars)

#######################################################
## Sources

SET(
  SOURCES
  main.cpp
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
)

#######################################################
## Find required dependencies

FIND_PACKAGE(OpenGL REQUIRED)
FIND_PACKAGE(FREEGLUT REQUIRED)
FIND_PACKAGE(Pangolin REQUIRED)
FIND_PACKAGE(DC1394 REQUIRED)
FIND_PACKAGE(Boost 1.40 COMPONENTS thread filesystem REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)

LINK_DIRECTORIES(
  ${Boost_LIBRARY_DIRS}
)

INCLUDE_DIRECTORIES(
  ${OPENGL_INCLUDE_DIR}
  ${FREEGLUT_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIR}
)

LINK_LIBRARIES(
  ${OPENGL_LIBRARIES}
  ${FREEGLUT_LIBRARY}
  ${Pangolin_LIBRARY}
  ${DC1394_LIBRARY}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS}
  lapack cvd
)

ADD_EXECUTABLE( screentocam ${SOURCES})
