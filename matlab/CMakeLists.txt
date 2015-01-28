cmake_minimum_required(VERSION 2.8.8)

# Find Calibu.
find_package(Calibu REQUIRED)

# Load add_mex macro.
include(mex.cmake)

# List external library dependencies.
set(MEX_EXTERNAL_LIBS "-Wl,-rpath,${Calibu_DIR} -L${Calibu_DIR} -l${Calibu_LIBRARIES}"
    CACHE STRING "External libraries to link to this mex file."
    FORCE)

# Invoke macro to generate calibu wrapper.
add_mex(calibu_mex calibu_mex.cpp)

add_dependencies(calibu_mex calibu)
