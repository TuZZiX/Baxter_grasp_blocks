# Baxter grasp blocks


This repository is for EECS-600 final project

Group Gamma (3rd)

#To run code:

**In gazebo:**

`roslaunch cwru_baxter_sim baxter_world.launch`

`roslaunch overall_executer overall_executer.launch`

change the #define in arm_planning_lib and pcl_chen to *GAZEBO*

**On real robot:**

`roslaunch overall_executer kinect_gripper.launch`

`roslaunch overall_executer overall_executer.launch`

change the #define in arm_planning_lib and pcl_chen to *REAL_WORLD*

#Troubleshooting:

If could not open device when running *overall_executer.launch* you have to give permission to the gripper USB connector by:

`sudo chmod -R 777 /dev/ttyUSB0`

#Collaborators:

Shipei Tian: Arm motion planning lib, team leader

Zhiang Chen: Block, surface and hand recognizing based on PCL

Alex DeFiore: Main program and HMI

Qian Wang: Gripper