find_package( Calibu 0.2 REQUIRED )
include_directories(${Calibu_INCLUDE_DIRS})
add_executable( grid_gen main.cpp )
target_link_libraries( grid_gen ${Calibu_LIBRARIES} )

