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

FIND_PACKAGE(OpenGL REQUIRED)
FIND_PACKAGE(FREEGLUT REQUIRED)
FIND_PACKAGE(Pangolin REQUIRED)
FIND_PACKAGE(CVD REQUIRED)
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
  ${CVD_INCLUDE_DIR}
)

LINK_LIBRARIES(
  ${OPENGL_LIBRARIES}
  ${FREEGLUT_LIBRARY}
  ${Pangolin_LIBRARY}
  ${DC1394_LIBRARY}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS}
  ${CVD_LIBRARY}
  lapack toontag
)

ADD_LIBRARY(${LIBRARY_NAME} SHARED ${SOURCES})
