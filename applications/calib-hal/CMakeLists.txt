#https://github.com/arpg/Calibu
find_package( Calibu 0.1 REQUIRED )
include_directories( ${Calibu_INCLUDE_DIRS} )

#https://github.com/arpg/miniglog
find_package( MINIGLOG )

# git://github.com/stevenlovegrove/Pangolin.git
find_package( Pangolin 0.1 QUIET )
include_directories( ${Pangolin_INCLUDE_DIRS} )

#https://github.com/arpg/HAL
find_package( HAL 0.1 QUIET )
find_package( PbMsgs )
include_directories( ${HAL_INCLUDE_DIRS} )

# git clone https://ceres-solver.googlesource.com/ceres-solver
list(APPEND SEARCH_HEADERS ${EIGEN3_INCLUDE_DIR}) # Help Ceres find Eigen
find_package( Ceres 1.6.0 QUIET )
include_directories( ${CERES_INCLUDES} )

find_package( OpenCV2 QUIET )
include_directories( ${OpenCV2_INCLUDES} )

find_package( CVARS )
include_directories( ${CVARS_INCLUDE_DIR} )

# This depends on HAL, which in turns depends on Calibu, so we disable this by default
set(BUILD_CALIBGRID_HAL OFF CACHE BOOL "To build or not build calibgrid-hal")

if(BUILD_CALIBGRID_HAL AND Pangolin_FOUND AND Ceres_FOUND AND HAL_FOUND AND OpenCV2_FOUND)
    add_executable( calibgrid-hal main.cpp )
    target_link_libraries( calibgrid-hal ${CERES_LIBRARIES} )
    target_link_libraries( calibgrid-hal ${Pangolin_LIBRARIES} )
    target_link_libraries( calibgrid-hal ${Calibu_LIBRARIES} )
    target_link_libraries( calibgrid-hal ${HAL_LIBRARIES} )
    target_link_libraries( calibgrid-hal ${CVARS_LIBRARY} )
    target_link_libraries( calibgrid-hal ${OpenCV2_LIBRARIES} )
    target_link_libraries( calibgrid-hal ${PbMsgs_LIBRARY} )

    install(TARGETS calibgrid-hal
        EXPORT CalibuTargets
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
    )
endif()
