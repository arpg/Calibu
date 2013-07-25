find_package( Calibu 0.1 REQUIRED )
include_directories( ${Calibu_INCLUDE_DIRS} )
link_libraries( ${LIBRARY_NAME} )

# git://github.com/stevenlovegrove/Pangolin.git
find_package( Pangolin 0.1 QUIET )
include_directories( ${Pangolin_INCLUDE_DIRS} )
link_libraries( ${Pangolin_LIBRARIES} )

# Make sure Ceres knows where to find Eigen
list(APPEND SEARCH_HEADERS ${EIGEN3_INCLUDE_DIR})

# git clone https://ceres-solver.googlesource.com/ceres-solver
find_package( Ceres 1.6.0 QUIET )
include_directories( ${CERES_INCLUDES} )
link_libraries( ${CERES_LIBRARIES} )

if( Pangolin_FOUND AND CERES_FOUND )
    add_executable( calibgrid main.cpp )

    install(TARGETS calibgrid
        EXPORT CalibuTargets
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
    )
endif()
