find_package( Boost QUIET )
find_package( ZLIB QUIET )

if( Boost_FOUND AND ZLIB_FOUND )
    include_directories( ${Calibu_INCLUDE_DIRS} )
    link_libraries( ${Calibu_LIBRARIES} )

    include_directories( ${Boost_INCLUDE_DIRS} )
    link_libraries( ${Boost_LIBRARIES} )

    include_directories( ${ZLIB_INCLUDE_DIR} )
    link_libraries( ${ZLIB_LIBRARIES} )

    # MVL source for old camera models
    file( GLOB mvlsrcs "MVL/*.c*")
    file( GLOB mvlhdrs "MVL/*.h")

    add_executable( cmod main.cpp ${mvlsrcs} ${mvlhdrs} )

    install(TARGETS cmod RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
endif()

