cmake_minimum_required(VERSION 2.8.3)
project(arm_planning_lib)

find_package(catkin_simple REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cwru_action
  baxter_traj_streamer
  gripper_move
)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PROJECT_NAME}/include)
include_directories(${gripper_move_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES 
  #CATKIN_DEPENDS catkin_simple roscpp std_msgs
  #DEPENDS system_lib
)

catkin_simple()

# example boost usage
find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
 SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries
# cs_add_library(my_lib src/my_lib.cpp)   
cs_add_library(arm_planning_lib src/arm_planning_lib.cpp) 
cs_add_library(arm_planning_lib_moveit src/arm_planning_lib_moveit.cpp) 

# Executables
cs_add_executable(arm_planning_lib_test src/arm_planning_lib_test.cpp)
target_link_libraries(arm_planning_lib_test arm_planning_lib baxter_traj_streamer gripper_move arm_planning_lib_moveit ${catkin_LIBRARIES})

#add_dependencies(arm_planning_lib_test ${gripper_move_EXPORTED_TARGETS})
#add_dependencies(arm_planning_lib ${gripper_move_EXPORTED_TARGETS})

cs_install()
cs_export()
    
