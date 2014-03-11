find_package( Calibu 0.1 REQUIRED )

find_package( ZLIB QUIET )

if( ZLIB_FOUND )
    include_directories( ${Calibu_INCLUDE_DIRS} )
    link_libraries( ${LIBRARY_NAME} )

    link_libraries( ${ZLIB_LIBRARIES} )
    include_directories( ${ZLIB_INCLUDE_DIR} )

    # MVL source for old camera models
    file( GLOB mvlsrcs "MVL/*.c*")
    file( GLOB mvlhdrs "MVL/*.h")

    add_executable( cmod main.cpp ${mvlsrcs} ${mvlhdrs} )

    install(TARGETS cmod RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
endif()

