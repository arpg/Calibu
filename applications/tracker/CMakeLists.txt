find_package(Fiducials REQUIRED)
include_directories(${Fiducials_INCLUDE_DIRS})
LINK_LIBRARIES( ${LIBRARY_NAME} )

# git://github.com/stevenlovegrove/Pangolin.git
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})
LINK_DIRECTORIES(${Pangolin_LIBRARY_DIRS})
LINK_LIBRARIES(${Pangolin_LIBRARIES})

ADD_EXECUTABLE(tracker main.cpp)
