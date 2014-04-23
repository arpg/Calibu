if(NOT ANDROID_PACKAGE_NAME)
  set(ANDROID_PACKAGE_NAME "edu.gwu.robotics")
endif()

# Configure build environment to automatically generate APK's instead of executables.
if(ANDROID)
  find_library(GNUSTL_SHARED_LIBRARY gnustl_shared)

  if(NOT GNUSTL_SHARED_LIBRARY)
    message(FATAL_ERROR "Could not find required GNU STL shared library.")
  endif()

  # Reset output directories to be in binary folder (rather than source)
  set(LIBRARY_OUTPUT_PATH_ROOT ${CMAKE_CURRENT_BINARY_DIR})
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${LIBRARY_OUTPUT_PATH_ROOT}/libs/${ANDROID_NDK_ABI_NAME})
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${LIBRARY_OUTPUT_PATH_ROOT}/libs/${ANDROID_NDK_ABI_NAME})
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${LIBRARY_OUTPUT_PATH_ROOT}/bin/${ANDROID_NDK_ABI_NAME})

  macro(add_library lib_name)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/libs/${ANDROID_NDK_ABI_NAME})
    _add_library(${lib_name} SHARED ${ARGN})

    # Add required link libs for android
    target_link_libraries(${lib_name} log android z -pthread ${GNUSTL_SHARED_LIBRARY})
  endmacro()
endif()
