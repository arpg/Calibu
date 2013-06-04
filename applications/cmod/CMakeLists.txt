find_package( MVL QUIET )

if(MVL_FOUND)
    include_directories( ${MVL_INCLUDE_DIRS} )
    link_libraries( ${MVL_LIBRARIES} )

    find_package( Calibu REQUIRED )
    include_directories( ${Calibu_INCLUDE_DIRS} )
    link_libraries( ${LIBRARY_NAME} )

    set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_MODULE_PATH} )
    find_package( OpenCV2 REQUIRED )
    include_directories( ${OpenCV2_INCLUDE_DIRS} )
    link_libraries( ${OpenCV2_LIBRARIES} )


    # git://github.com/stevenlovegrove/Pangolin.git
    find_package( Pangolin REQUIRED )
    include_directories( ${Pangolin_INCLUDE_DIRS} )
    link_libraries( ${Pangolin_LIBRARIES} )    
    
    # git clone https://ceres-solver.googlesource.com/ceres-solver
    find_package( CeresSolver REQUIRED )
    include_directories( ${CeresSolver_INCLUDE_DIRS} )
    link_libraries( ${CeresSolver_LIBRARIES} )
    
    add_executable( cmod main.cpp )
    
    install(TARGETS cmod RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
endif()
