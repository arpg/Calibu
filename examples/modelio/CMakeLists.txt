find_package( Calibu 0.1 REQUIRED )
include_directories( ${Calibu_INCLUDE_DIRS} )
LINK_LIBRARIES( ${LIBRARY_NAME} )

ADD_EXECUTABLE( modelio main.cpp )
