// node intended to test the gripper to open and close by sending position
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <math.h>

int cmd = 0; //initialize gripper to open command
void testCmdCB(const std_msgs::Bool& positon_cmd) 
{ 
  cmd=goal->close_or_open;
  if (cmd == 0)
  {
     ROS_INFO("gripper open");
  }
  else if (cmd == 1)
  {
     ROS_INFO("gripper close");
  }

 
} 


int main(int argc, char **argv) {
    ros::init(argc, argv, "gripper_test"); 
    ros::NodeHandle n; 

    int motor_id=1; 
    
    sprintf("dynamixel_motor%d_cmd",motor_id);
    
    ros::Publisher position_publisher = n.advertise<std_msgs::Int16>(dynamixel_motor1_cmd, 1);

    ros::Subscriber manual_cmd_subscriber = n.subscribe("gripper_cmd",1,manualCmdCB);     

    std_msgs::Int16 position_command;
    
    std_msgs::Int16 gripper_position
   double dt = 0.02; 
   ros::Rate naptime(1/dt); 

     while (ros::ok()) 
    {

	position_command.data = cmd;

  	if (position_command.data == 0)
  	{
   	  ROS_INFO("sending open command");
	 gripper_command.data = 3000; 
  	}

  	if (position_command.data == 1)
  	{
   	  ROS_INFO("sending close command");
	  gripper_position.data = 3999; 
  	}

	position_publisher.publish(gripper_postion); // publish the operation mode (position/torque) to the motor node
      
	ros::spinOnce();	
	naptime.sleep();


    }
}

