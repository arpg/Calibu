enable_testing()

# find packages

list(APPEND REQUIRED_INCLUDE_DIRS ${USER_INC})
list(APPEND REQUIRED_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/include)
list(APPEND REQUIRED_INCLUDE_DIRS ${CMAKE_BINARY_DIR}/include)
list(APPEND REQUIRED_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR})
list(APPEND REQUIRED_INCLUDE_DIRS ${CMAKE_CURRENT_BINARY_DIR})
list(APPEND REQUIRED_LIBRARIES calibu)

find_package(GTest REQUIRED)
list(APPEND REQUIRED_INCLUDE_DIRS ${GTEST_INCLUDE_DIRS})
list(APPEND REQUIRED_LIBRARIES ${GTEST_LIBRARIES})
list(APPEND REQUIRED_LIBRARIES pthread)

# define c++ sources

set(CPP_SOURCES
  base64_test.cpp
  pcalib_xml_test.cpp
)

# build executable

include_directories(${REQUIRED_INCLUDE_DIRS})
add_executable(tests tests.cpp ${CPP_SOURCES})
target_link_libraries(tests ${REQUIRED_LIBRARIES})