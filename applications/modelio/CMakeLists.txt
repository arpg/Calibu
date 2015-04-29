include_directories( ${Calibu_INCLUDE_DIRS} )

add_executable( modelio main.cpp )
target_link_libraries( modelio ${Calibu_LIBRARIES} calibu )
