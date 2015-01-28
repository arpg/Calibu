function(link_android target)
  if(NOT ANDROID)
    return()
  endif()

  find_library(GNUSTL_SHARED_LIBRARY gnustl_shared)

  if(NOT GNUSTL_SHARED_LIBRARY)
    message(FATAL_ERROR "Could not find required GNU STL shared library.")
  endif()
  target_link_libraries(${target} PRIVATE log android z -pthread ${GNUSTL_SHARED_LIBRARY})
endfunction()