# Baxter grasp blocks


This repository is for EECS-600 final project

Group Gamma (3rd)

# Color Movements

Red: pick up and drop on the right

Blue: pick up, take a look and put it back

White: pick up and drop on the left

Black: pick up and put it back

Green: pick up, take a look and drop on the right

Wood: pick up, take a look and drop on the left

# Depandencies

This branch is using moveit as its planning interface, so in order to compile it, you have to get moveit first:

`sudo apt-get install ros-indigo-moveit-full`

You also need to have config for Baxter with you:

`git clone https://github.com/TuZZiX/baxter_moveit_config.git`

# Moveit planning:

Enable *#define MOVEIT* in overall_executer/scr/fp.cpp

`roslaunch overall_executer kinect_gripper.launch`

`roslaunch arm_planning_lib moveit_planning_lib.launch`

`rosrun overall_executer overall_executer`


# Classic planning:

comment *#define MOVEIT* in overall_executer/scr/fp.cpp

Change the #define in arm_planning_lib and pcl_chen to *REAL_WORLD*

`roslaunch overall_executer kinect_gripper.launch`

`roslaunch overall_executer overall_executer.launch`


# In gazebo:

Change the #define in arm_planning_lib and pcl_chen to *GAZEBO*

`roslaunch cwru_baxter_sim baxter_world.launch`

`roslaunch overall_executer overall_executer.launch`


# Troubleshooting:

If could not open device when running *overall_executer.launch* you have to give permission to the gripper USB connector by:

`sudo chmod -R 777 /dev/ttyUSB0`

# Collaborators:

Shipei Tian: Arm motion planning lib, team leader

Zhiang Chen: Block, surface and hand recognizing based on PCL

Alex DeFiore: Main program and HMI

Qian Wang: Gripper