# Baxter grasp blocks


This repository is for the robotice final project

#Collaborators:

	Shipei Tian: Arm motion planning lib, team leader

	Zhiang Chen: Block, surface and hand recognizing based on PCL

	Alex DeFiore: Main program

	Qian Wang: Gripper

#To run code:

**To run in gazebo:**

'roslaunch cwru_baxter_sim baxter_world.launch'

'roslaunch overall_executer overall_executer.launch'

change the #define in arm_planning_lib and pcl_chen to GAZEBO

**To run in real baxter:**

'roslaunch overall_executer kinect_gripper.launch'

'roslaunch overall_executer overall_executer.launch'

change the #define in arm_planning_lib and pcl_chen to REAL_WORLD

