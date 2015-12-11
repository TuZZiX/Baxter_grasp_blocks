#include <gripper_move/grippermove.h>

#define TORQUE


#ifdef WSN

int curpo=0;
GripperMove::GripperMove(ros::NodeHandle* nodehandle) : nh_(*nodehandle){
	static ros::Publisher position_pub = nh_.advertise<std_msgs::Int16>("dynamixel_motor1_cmd", 1);
	position_publisher = &position_pub;
	std_msgs::Int16 position;
	//opens the hand on start up to know the position
	position.data = 3000;
	(*position_publisher).publish(position);
	//keep track of current position
	curpo = 3000;
}
void GripperMove::gripper_open(){
	std_msgs::Int16 position;
	position.data = 3000;
	(*position_publisher).publish(position);
	/*
	std_msgs::Int16 position;
	for(int i=curpo;i>=3000; i-=10){
		position.data=i;
		(*position_publisher).publish(position);
		ros::Duration(0.002).sleep(); 
	};
	curpo=3000;*/
}


void GripperMove::gripper_close(){
	std_msgs::Int16 position;
	position.data = 3999;
	(*position_publisher).publish(position);
/*
	std_msgs::Int16 position;
	for(int i = curpo; i<=4000; i+=10){
		position.data = i;
		(*position_publisher).publish(position);
		ros::Duration(0.002).sleep(); 
	}

	curpo = 4000;*/
}
#endif

#ifdef TORQUE

GripperMove::GripperMove(ros::NodeHandle* nodehandle) : nh_(*nodehandle){
	static ros::Publisher dyn_puba = nh_.advertise<std_msgs::Int16>("/dynamixel_motor1_cmd", 1);
	dyn_pub = &dyn_puba;
    static ros::Publisher torque_togglea = nh_.advertise<std_msgs::Bool>("/dynamixel_motor1_mode", 1);
    torque_toggle = &torque_togglea;
    std_msgs::Int16 cmd_topic_nameaaa;
    cmd_topic_nameaaa.data=3000;
    std_msgs::Bool cmd_topic_toggleaaa;
    cmd_topic_toggleaaa.data=0;
    (*torque_toggle).publish(cmd_topic_toggleaaa);
    ros::Duration(0.5).sleep();
    (*dyn_pub).publish(cmd_topic_nameaaa); 

}

void GripperMove::gripper_open(){
	std_msgs::Int16 cmd_topic_nameaaa;
	cmd_topic_nameaaa.data=3200;
	std_msgs::Bool cmd_topic_toggleaaa;
	cmd_topic_toggleaaa.data=0;
	(*torque_toggle).publish(cmd_topic_toggleaaa);
	ros::Duration(0.5).sleep();
    (*dyn_pub).publish(cmd_topic_nameaaa); 


}


void GripperMove::gripper_close(){
	std_msgs::Int16 cmd_topic_nameaaa;
	cmd_topic_nameaaa.data=80;
	std_msgs::Bool cmd_topic_toggleaaa;
	cmd_topic_toggleaaa.data=1;
	(*torque_toggle).publish(cmd_topic_toggleaaa);
	ros::Duration(0.5).sleep();
    (*dyn_pub).publish(cmd_topic_nameaaa); 

}
#endif
