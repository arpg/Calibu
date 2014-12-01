#https://github.com/arpg/Calibu
find_package( Calibu 0.1 REQUIRED )
include_directories( ${Calibu_INCLUDE_DIRS} )

#https://github.com/arpg/miniglog
find_package( MINIGLOG )
include_directories( ${MINIGLOG_INCLUDE_DIRS} )
#https://github.com/arpg/HAL
find_package( HAL 0.1 QUIET )
find_package( PbMsgs )
include_directories( ${HAL_INCLUDE_DIRS} )

find_package( OpenCV2 QUIET )
include_directories( ${OpenCV2_INCLUDES} )

# This depends on HAL, which in turns depends on Calibu, so we disable this by default
set(BUILD_RECTIFY OFF CACHE BOOL "To build or not build rectify")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g ")
 
if(BUILD_RECTIFY AND HAL_FOUND AND OpenCV2_FOUND)
    add_executable( rectify main.cpp )
    target_link_libraries( rectify ${Calibu_LIBRARIES} )
    target_link_libraries( rectify ${HAL_LIBRARIES} )
    target_link_libraries( rectify ${CVARS_LIBRARY} )
    target_link_libraries( rectify ${OpenCV2_LIBRARIES} )
    target_link_libraries( rectify ${PbMsgs_LIBRARY} )
    target_link_libraries( rectify protobuf )		     
    target_link_libraries( rectify dl )
    install(TARGETS rectify
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
    )
endif()
