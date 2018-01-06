cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(SegGraph)

#head file path  
INCLUDE_DIRECTORIES(include)  
  
#source directory  
AUX_SOURCE_DIRECTORY(src DIR_SRCS)  
  


find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (SegGraph  ${DIR_SRCS})
target_link_libraries (SegGraph ${PCL_LIBRARIES})