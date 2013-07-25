find_package( MVL QUIET )

if(MVL_FOUND)
    include_directories( ${MVL_INCLUDE_DIRS} )
    link_libraries( ${MVL_LIBRARIES} )

    find_package( Calibu 0.1 REQUIRED )
    include_directories( ${Calibu_INCLUDE_DIRS} )
    link_libraries( ${LIBRARY_NAME} )

    # git://github.com/stevenlovegrove/Pangolin.git
    find_package( Pangolin 0.1 QUIET )
    include_directories( ${Pangolin_INCLUDE_DIRS} )
    link_libraries( ${Pangolin_LIBRARIES} )    
    
    # git clone https://ceres-solver.googlesource.com/ceres-solver
    find_package( Ceres 1.6.0 QUIET )
    include_directories( ${CERES_INCLUDE_DIRS} )
    link_libraries( ${CERES_LIBRARIES} )
    
    if( Pangolin_FOUND AND CERES_FOUND )
        add_executable( cmod main.cpp )
    
        install(TARGETS cmod RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
    endif()
endif()
