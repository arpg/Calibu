include(CMakeParseArguments)
include(FindGTest)
include(link_android)

function(def_test test)

  string(TOUPPER ${test} TEST)

  set(TEST_OPTIONS CUDA)
  set(TEST_SINGLE_ARGS)
  set(TEST_MULTI_ARGS SOURCES DEPENDS CONDITIONS LINK_LIBS)
  cmake_parse_arguments(test
    "${TEST_OPTIONS}"
    "${TEST_SINGLE_ARGS}"
    "${TEST_MULTI_ARGS}"
    "${ARGN}"
    )

  if(NOT test_SOURCES)
    message(FATAL_ERROR "def_test for ${TEST} has an empty source list.")
  endif()

  set(cache_var BUILD_${TEST})
  set(${cache_var} ON CACHE BOOL "Enable ${TEST} compilation.")

  if(test_CONDITIONS)
    foreach(cond ${test_CONDITIONS})
      if(NOT ${cond})
	set(${cache_var} OFF)
	message("${cache_var} is false because ${cond} is false.")
	return()
      endif()
    endforeach()
  endif()

  if(test_DEPENDS)
    foreach(dep ${test_DEPENDS})
      if(NOT TARGET ${dep})
	set(${cache_var} OFF)
	message("${cache_var} is false because ${dep} is not being built.")
	return()
      endif()
    endforeach()
  endif()

  if(${cache_var})
    if (test_CUDA)
      cuda_add_executable(${test} ${test_SOURCES})
    else()
      add_executable(${test} ${test_SOURCES})
    endif()
    gtest_add_tests(${test} "" ${test_SOURCES})

    # Always build tests debug so they can be debugged!
    set_target_properties(${test} PROPERTIES
      COMPILE_FLAGS "${CMAKE_CXX_FLAGS_DEBUG}")

    target_link_libraries(${test} PRIVATE
      gtest gtest_main
      ${test_DEPENDS}
      ${test_LINK_LIBS})

    if(ANDROID)
      link_android(${test})
    endif()
  endif()
endfunction()