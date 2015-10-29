cmake_minimum_required(VERSION 2.8.8)

find_package(Calibu REQUIRED) # Find Calibu.
include(mex.cmake) # Load add_mex macro.

# List external library dependencies.
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall ${CMAKE_CXX_FLAGS}")
set(MEX_LDFLAGS "-Wl,-rpath,${Calibu_DIR} -L${Calibu_DIR}")
set(MEX_LINK_LIBS "${Calibu_LIBRARIES};calibu" 
    CACHE STRING "External libraries to link to this mex file."
    FORCE)

# Invoke macro to generate calibu wrapper.
add_mex(calibu_mex calibu_mex.cpp)

#add_dependencies(calibu_mex calibu)

