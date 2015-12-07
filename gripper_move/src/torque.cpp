#include <ros/ros.h> 
#include <gripper_move/grippermove.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "torque"); 
    ros::NodeHandle nh;
    GripperMove grippertest(&nh);


    while (ros::ok()) {
        
    	grippertest.gripper_close();
    	ros::Duration(5).sleep(); 
    	grippertest.gripper_open();
    	ros::Duration(10).sleep(); 
        ros::spinOnce();
    }
}

    
