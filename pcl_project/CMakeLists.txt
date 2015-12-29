cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
PROJECT(pcl_project)

find_package(PCL 1.6 REQUIRED)
find_package(OpenCV REQUIRED)

INCLUDE_DIRECTORIES( ${OPENCV_INCLUDE_DIR} )
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRAccRY_DIRS})
add_definitions(${PCL_DEFINITIONS})

ADD_EXECUTABLE(pcl_project main.cpp)
target_link_libraries(pcl_project ${PCL_LIBRARIES})
