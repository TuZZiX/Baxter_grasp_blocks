#include <gripper_move/grippermove.h>


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
	for(int i=curpo;i>=3000;i--){
		position.data=i;
		(*position_publisher).publish(position);
		ros::Duration(.0005).sleep(); 
	};
	curpo=3000;
}


void GripperMove::gripper_close(){
	std_msgs::Int16 position;
	for(int i = curpo; i<=3900; i++){
		position.data = i;
		(*position_publisher).publish(position);
		ros::Duration(.0005).sleep(); 
	}

	curpo = 3900;
}
