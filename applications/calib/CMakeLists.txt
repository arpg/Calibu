find_package( Calibu REQUIRED )
include_directories( ${Calibu_INCLUDE_DIRS} )
link_libraries( ${LIBRARY_NAME} )

# git://github.com/stevenlovegrove/Pangolin.git
find_package( Pangolin REQUIRED )
include_directories( ${Pangolin_INCLUDE_DIRS} )
link_directories( ${Pangolin_LIBRARY_DIRS} )
link_libraries( ${Pangolin_LIBRARIES} )

# git clone https://ceres-solver.googlesource.com/ceres-solver
find_package( CeresSolver REQUIRED )
include_directories( ${CeresSolver_INCLUDE_DIRS} )
link_libraries( ${CeresSolver_LIBRARIES} )

add_executable( calibgrid main.cpp )

install(TARGETS calibgrid
  EXPORT CalibuTargets
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
)
