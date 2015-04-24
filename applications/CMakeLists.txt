if(NOT ANDROID)
    add_subdirectory( calib )
    add_subdirectory( cmod )

    include_directories(${Calibu_INCLUDE_DIRS})
    add_executable( grid_gen grid-gen/main.cpp )
    target_link_libraries( grid_gen ${Calibu_LIBRARIES} calibu )
endif()
