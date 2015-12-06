cmake_minimum_required(VERSION 2.8.3)
project(image_test)

find_package(catkin_simple REQUIRED)
find_package(catkin REQUIRED COMPONENTS
	pcl_chen
	arm_planning_lib
)

include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EiGEN_DEFINITIONS})

catkin_simple()

# example boost usage
find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries
# cs_add_library(my_lib src/my_lib.cpp)
find_package(cwru_pcl_utils REQUIRED)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${cwru_pcl_utils_INCLUDE_DIRS}
  ${pcl_chen_INCLUDE_DIRS}
)

# Executables
# cs_add_executable(example src/example.cpp)
cs_add_executable(image_test src/fp.cpp)
cs_add_executable(pcl_emotion_test src/pcl_emotion_test.cpp)
# target_link_library(example my_lib)
target_link_libraries(image_test arm_planning_lib pcl_chen)
target_link_libraries(pcl_emotion_test arm_planning_lib pcl_chen)

cs_install()
cs_export()
    
