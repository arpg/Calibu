find_package( Calibu 0.2 REQUIRED )
find_package( ZLIB QUIET )

if( ZLIB_FOUND )
  include_directories( ${Calibu_INCLUDE_DIRS} ${ZLIB_INCLUDE_DIR} )
  link_libraries( ${Calibu_LIBRARIES} ${ZLIB_LIBRARIES} )

    # grab the old MVL source for old camera models
    file( GLOB mvlsrcs "MVL/*.c*")
    file( GLOB mvlhdrs "MVL/*.h")

    add_executable( cmod main.cpp ${mvlsrcs} ${mvlhdrs} )

    install(TARGETS cmod RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
endif()

