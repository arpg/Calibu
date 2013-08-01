#find_package( MVL QUIET )

#if(MVL_FOUND)
#    include_directories( ${MVL_INCLUDE_DIRS} )
#    link_libraries( ${MVL_LIBRARIES} )

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

    if( Pangolin_FOUND AND Ceres_FOUND )

        add_executable( cmod main.cpp
                        MVL/CameraModel.h
                        MVL/camera.h
                        MVL/camera.cpp
                        MVL/camera_transform_cf.h
                        MVL/camera_transform_cf.cpp
                        MVL/camera_3d_to_2d.h
                        MVL/camera_3d_to_2d.cpp
                        MVL/camera_2d_to_3d.h
                        MVL/camera_2d_to_3d.cpp
                        MVL/matrix.h
                        MVL/matrix.cpp
                        MVL/kinematics.h
                        MVL/kinematics.cpp
                        MVL/conversions.h
                        MVL/conversions.cpp
                        MVL/camera_convert.h
                        MVL/camera_convert.cpp
                        MVL/camera_model_io.h
                        MVL/camera_model_io.cpp
                        MVL/camera_model_lut.h
                        MVL/camera_model_lut.cpp
                        MVL/camera_model_linear.h
                        MVL/camera_model_linear.cpp
                        MVL/camera_model_warped.h
                        MVL/camera_model_warped.cpp
                        MVL/mvl_tinyxml.h
                        MVL/mvl_tinyxml.cpp
                        MVL/mvl_tinyxmlparser.cpp
                        MVL/mvl_tinyxmlerror.cpp
        )

        install(TARGETS cmod RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
    endif()
#endif()
