# CMake entry point.
# Note that this CMakeLists.txt in fully independent from the one in the parent directory
# (since that's the goal of this tutorial !)

# For an introduction to CMake, see
# http://www.cmake.org/cmake/help/cmake_tutorial.html (at least steps 1 and 2)

cmake_minimum_required (VERSION 3.10)
set(name "KinectSimple")
project ( ${name} )

# cmake module
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

# c++ version
set (CMAKE_CXX_STANDARD 11)

#opencv 
if ( WIN32 )
	set(OpenCV_DIR D:/libs/opencv/x64/vc14/lib)
	set(OpenCV_CONFIG_PATH D:/libs/opencv)
endif()
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#Kinect
# Find Kinect SDK v2
find_package( KinectSDK2 REQUIRED )

# Settings Kinect SDK v2
if(KinectSDK2_FOUND)
  include_directories( ${KinectSDK2_INCLUDE_DIRS} )
  link_directories( ${KinectSDK2_LIBRARY_DIRS} )
  #add_custom_command( TARGET project POST_BUILD ${KinectSDK2_COMMANDS} )
endif()

set(MY_HEADER_FILES
    KinectSimple.h
)

set(MY_SOURCE_FILES 
    KinectSimple.cpp
    main.cpp
)
include_directories(
	${PROJECT_SOURCE_DIR}	
)

add_executable(${name}
	${MY_HEADER_FILES}
	${MY_SOURCE_FILES}
	)

target_link_libraries(${name}
    ${OpenCV_LIBS}
    ${KinectSDK2_LIBRARIES}
	)