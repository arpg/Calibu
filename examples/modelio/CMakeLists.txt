find_package( CALIBU 0.1 REQUIRED )
include_directories( ${CALIBU_INCLUDE_DIRS} )

add_executable( modelio main.cpp )
target_link_libraries( modelio ${CALIBU_LIBRARIES} )
