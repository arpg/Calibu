SET(SAMPLE_NAME "calibgrid")

FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)

find_package(Fiducials REQUIRED)
include_directories(${Fiducials_INCLUDE_DIRS})
link_libraries(${LIBRARY_NAME})

# git://github.com/stevenlovegrove/Sophus.git
FIND_PACKAGE(Sophus REQUIRED)
INCLUDE_DIRECTORIES( ${Sophus_INCLUDE_DIR} )
LINK_DIRECTORIES( ${Sophus_LIBRARY_DIRS} )
LINK_LIBRARIES(${Sophus_LIBRARIES})

# git://github.com/stevenlovegrove/Pangolin.git
FIND_PACKAGE(Pangolin REQUIRED)
INCLUDE_DIRECTORIES(${Pangolin_INCLUDE_DIRS})
LINK_DIRECTORIES(${Pangolin_LIBRARY_DIRS})
LINK_LIBRARIES(${Pangolin_LIBRARIES})

# git clone https://ceres-solver.googlesource.com/ceres-solver
find_package( CeresSolver REQUIRED)
INCLUDE_DIRECTORIES(${CeresSolver_INCLUDE_DIRS})
LINK_LIBRARIES(${CeresSolver_LIBRARIES})


INCLUDE_DIRECTORIES(
  ${OpenCV_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${Sophus_INCLUDE_DIR}
)

LINK_LIBRARIES(
  ${OpenCV_LIBS}
)

ADD_EXECUTABLE(${SAMPLE_NAME} main.cpp)
