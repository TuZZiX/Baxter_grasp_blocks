#include <gripper_move/grippermove.h>

#define WSN


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
	position.data = 3200;
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

#ifdef MARC

GripperMove::GripperMove(ros::NodeHandle* nodehandle) : nh_(*nodehandle){

}

void GripperMove::gripper_open(){

}


void GripperMove::gripper_close(){

}
#endif