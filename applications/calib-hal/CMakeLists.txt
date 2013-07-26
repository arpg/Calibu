#https://github.com/gwu-robotics/Calibu
find_package( Calibu 0.1 REQUIRED )
include_directories( ${Calibu_INCLUDE_DIRS} )

# git://github.com/stevenlovegrove/Pangolin.git
find_package( Pangolin 0.1 QUIET )
include_directories( ${Pangolin_INCLUDE_DIRS} )

#https://github.com/gwu-robotics/HAL
find_package( HAL 0.1 QUIET )
include_directories( ${HAL_INCLUDE_DIRS} )

# git clone https://ceres-solver.googlesource.com/ceres-solver
list(APPEND SEARCH_HEADERS ${EIGEN3_INCLUDE_DIR}) # Help Ceres find Eigen
find_package( Ceres 1.6.0 QUIET )
include_directories( ${CERES_INCLUDES} )

if( Pangolin_FOUND AND Ceres_FOUND AND HAL_FOUND )
    add_executable( calibgrid-hal main.cpp )
    target_link_libraries( calibgrid-hal ${CERES_LIBRARIES} )
    target_link_libraries( calibgrid-hal ${Pangolin_LIBRARIES} )
    target_link_libraries( calibgrid-hal ${Calibu_LIBRARIES} )
    target_link_libraries( calibgrid-hal ${HAL_LIBRARIES} )

    install(TARGETS calibgrid-hal
        EXPORT CalibuTargets
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
    )
endif()
