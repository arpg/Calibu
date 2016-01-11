include_directories( ${Calibu_INCLUDE_DIRS} )

# git://github.com/stevenlovegrove/Pangolin.git
find_package( Pangolin 0.1 QUIET )
include_directories( ${Pangolin_INCLUDE_DIRS} )

# git clone https://ceres-solver.googlesource.com/ceres-solver
list(APPEND SEARCH_HEADERS ${EIGEN3_INCLUDE_DIR}) # Help Ceres find Eigen
find_package( Ceres 1.6.0 QUIET )
include_directories( ${CERES_INCLUDES} )

# Check that OPENCV is available
set(NO_OPENCV_GPU TRUE)
find_package( OpenCV QUIET )
find_package( CVars QUIET )

option(BUILD_CALIBGRID OFF "Toggle build calibgrid.")
if( Pangolin_FOUND AND Ceres_FOUND AND OpenCV_FOUND AND CVars_FOUND AND BUILD_CALIBGRID)
    add_executable( calibgrid main.cpp )
    target_link_libraries( calibgrid
        ${CERES_LIBRARIES}
        ${Pangolin_LIBRARIES}
        ${Calibu_LIBRARIES}
        ${CVars_LIBRARIES}
        calibu )

    install(TARGETS calibgrid EXPORT CalibuTargets RUNTIME
            DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
elseif(BUILD_CALIBGRID)
    #report what's missing
    set(WARNING_MSG "calibgrid dependencies not met:")
    foreach(dep Pangolin Ceres OpenCV CVars)
        if(NOT ${dep}_FOUND)
            set(WARNING_MSG "${WARNING_MSG} ${dep}")
        endif() 
    endforeach()
    set(WARNING_MSG "${WARNING_MSG}. Skipping calibgrid during build.")
    message(WARNING ${WARNING_MSG})
endif()
