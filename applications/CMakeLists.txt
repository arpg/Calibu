if(NOT ANDROID)
    add_subdirectory( calib )
    add_subdirectory( cmod )
    find_package( HAL 0.1 QUIET )
    if( HAL_FOUND )
#        add_subdirectory( calib-hal )
    endif()
endif()

