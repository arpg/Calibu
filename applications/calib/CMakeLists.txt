#https://github.com/arpg/Calibu
find_package( CALIBU 0.1 REQUIRED )
include_directories( ${CALIBU_INCLUDE_DIRS} )

# git://github.com/stevenlovegrove/Pangolin.git
find_package( PANGOLIN 0.1 QUIET )
include_directories( ${PANGOLIN_INCLUDE_DIRS} )

# git clone https://ceres-solver.googlesource.com/ceres-solver
list(APPEND SEARCH_HEADERS ${EIGEN3_INCLUDE_DIR}) # Help Ceres find Eigen
find_package( Ceres 1.6.0 QUIET )
include_directories( ${CERES_INCLUDES} )

# Check that OPENCV is available
set(NO_OPENCV_GPU TRUE)
find_package( OpenCV QUIET )

option(BUILD_CALIBGRID OFF "Toggle build calibgrid.")
if( PANGOLIN_FOUND AND Ceres_FOUND AND OpenCV_FOUND AND BUILD_CALIBGRID)
    add_executable( calibgrid main.cpp )
    target_link_libraries( calibgrid ${CERES_LIBRARIES} )
    target_link_libraries( calibgrid ${PANGOLIN_LIBRARIES} )
    target_link_libraries( calibgrid ${CALIBU_LIBRARIES} )
    target_link_libraries( calibgrid ${CVARS_LIBRARY} )

    install(TARGETS calibgrid
        EXPORT CalibuTargets
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
    )
endif()
