#include <arm_planning_lib/arm_planning_lib.h>

ArmPlanningInterface::ArmPlanningInterface(ros::NodeHandle* nodehandle): nh_(*nodehandle),
cart_move_action_client_("cartMoveActionServer", true), gripper(&nh_) { // constructor
	// attempt to connect to the server:
	ROS_INFO("waiting for server: ");
		bool server_exists = false;
	while ((!server_exists)&&(ros::ok())) {
		server_exists = cart_move_action_client_.waitForServer(ros::Duration(0.5)); //
		ros::spinOnce();
		ros::Duration(0.5).sleep();
		//       ROS_INFO("retrying...");
	}
	ROS_INFO("connected to action server"); // if here, then we connected to the server;
	collision_offset << 0, 0, 0.32870648723;
	gripper_offset << 0, 0, 0.17665179565;
	//	arm_back_joints << -0.20628314186274055, -1.0357409957184627, -0.5780814380569512, 1.3172378913039648, 1.7037512511889998, 1.2374043903744254, 1.6265004725741392;
	arm_back_pose.pose.position.x = 0.628566212076;
	arm_back_pose.pose.position.y = -0.347307547123;
	arm_back_pose.pose.position.z = 0.350523968221;
	arm_back_pose.pose.orientation.x = 0.630141382934;
	arm_back_pose.pose.orientation.y = 0.775598336984;
	arm_back_pose.pose.orientation.z = 0.000519772148981;
	arm_back_pose.pose.orientation.w = 0.0369971217577;
	
	//drop_offset_left << -0.627811922278, -0.37037794008, -0.01957099178;
	drop_offset_left << -0.1, -0.2, 0;
	//drop_offset_right << -0.02005453231, 0.29488994572, 0.00129911747;
	drop_offset_right << 0.1, 0.2, 0;
	//	take_look_pose << -0.18503156094537968, 0.44527797626583865, 0.36021593694382065, 1.827325037963473, 1.0199002550686511, 1.0137709796128629, 1.2701787563643496;
	take_look_pose.pose.position.x = 0.52542198926;
	take_look_pose.pose.position.y = -0.19065849761;
	take_look_pose.pose.position.z = 0.154626356725;
	take_look_pose.pose.orientation.x = -0.249821685981;
	take_look_pose.pose.orientation.y = -0.335221027398;
	take_look_pose.pose.orientation.z = -0.504147162678;
	take_look_pose.pose.orientation.w = 0.755679579166;
	
	pre_grab_pose.pose.position.x = 0.62630899645;
	pre_grab_pose.pose.position.y = -0.00805334156627;
	pre_grab_pose.pose.position.z = 0.206422664913;
	pre_grab_pose.pose.orientation.x = 0.629827329351;
	pre_grab_pose.pose.orientation.y = 0.776484996568;
	pre_grab_pose.pose.orientation.z = 0.0187988533019;
	pre_grab_pose.pose.orientation.w = -0.00593198287789;
	
	grab_pose.pose.position.x = 0.627811922278;
	grab_pose.pose.position.y = -0.00340972086974;
	grab_pose.pose.position.z = 0.0543679733262;
	grab_pose.pose.orientation.x = 0.629826654045;
	grab_pose.pose.orientation.y = 0.776525376337;
	grab_pose.pose.orientation.z = 0.0171547560392;
	grab_pose.pose.orientation.w = -0.0056956215282;
	
	default_orientation.x() = 0.535374791616;
	default_orientation.y() = 0.844190102515;
	default_orientation.z() = 0.0264940675637;
	default_orientation.w() = 0.00386881152953;

	take_look_joints << -0.18503156094537968, 0.44527797626583865, 0.36021593694382065, 1.827325037963473, 1.0199002550686511, 1.0137709796128629, 1.2701787563643496;


	global_pose_offset.pose.position.x = 0;
	global_pose_offset.pose.position.y = 0;
	global_pose_offset.pose.position.z = 0;
	global_pose_offset.pose.orientation.x = 0;
	global_pose_offset.pose.orientation.y = 0;
	global_pose_offset.pose.orientation.z = 0;
	global_pose_offset.pose.orientation.w = 0;

	global_joints_offset << 0,0,0,0,0,0,0;

}

// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server
//int g_return_code=0;
void ArmPlanningInterface::doneCb_(const actionlib::SimpleClientGoalState& state,
const cwru_action::cwru_baxter_cart_moveResultConstPtr& result) {
	//	ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
	//	ROS_INFO("got return value= %d", result->return_code);
	cart_result_=*result;
}
////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////
bool ArmPlanningInterface::moveArmsBack(void) {
	/*
	//    ROS_INFO("requesting a joint-space motion plan");
	cart_goal_.command_code = cwru_action::cwru_baxter_cart_moveGoal::RT_ARM_PLAN_JSPACE_PATH_CURRENT_TO_PRE_POSE;
	cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmPlanningInterface::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
	finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
	//    ROS_INFO("return code: %d",cart_result_.return_code);
	if (!finished_before_timeout_ || cart_result_.return_code==cwru_action::cwru_baxter_cart_moveResult::RT_ARM_PATH_NOT_VALID || cart_result_.return_code!=cwru_action::cwru_baxter_cart_moveResult::SUCCESS) {
	return false;
	}
	computed_arrival_time_= cart_result_.computed_arrival_time; //action_client.get_computed_arrival_time();
	//    ROS_INFO("computed move time: %f",computed_arrival_time_);
	return true;
	*/
	gripper.gripper_open();
	if(planPath(arm_back_joints)){
		if(!executePath()) {
			return false;
		}
	} else {
		return false;
	}
	return true;
}

bool ArmPlanningInterface::planPath(geometry_msgs::Pose pose) {
	geometry_msgs::PoseStamped stamp_pose;
	stamp_pose.pose = pose;
	return planPath(stamp_pose);
}

bool ArmPlanningInterface::planPath(geometry_msgs::PoseStamped pose) {
	
	ROS_INFO("requesting a cartesian-space motion plan");
	pose = addPose(pose, global_pose_offset);

	cart_goal_.command_code = cwru_action::cwru_baxter_cart_moveGoal::RT_ARM_PLAN_PATH_CURRENT_TO_GOAL_POSE;
	cart_goal_.des_pose_gripper_right = pose;
	cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmPlanningInterface::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
	finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
	    ROS_INFO("return code: %d",cart_result_.return_code);
	if (!finished_before_timeout_ || cart_result_.return_code==cwru_action::cwru_baxter_cart_moveResult::RT_ARM_PATH_NOT_VALID || cart_result_.return_code!=cwru_action::cwru_baxter_cart_moveResult::SUCCESS) {
		return false;
	}
	computed_arrival_time_= cart_result_.computed_arrival_time; //action_client.get_computed_arrival_time();
	//    ROS_INFO("computed move time: %f",computed_arrival_time_);
	return true;
}

bool ArmPlanningInterface::planPath(Vector7d joints) {
	ROS_INFO("requesting a joint-space motion plan");
	joints = joints + global_joints_offset;

	cart_goal_.command_code = cwru_action::cwru_baxter_cart_moveGoal::RT_ARM_PLAN_JSPACE_PATH_CURRENT_TO_QGOAL;
	cart_goal_.q_goal_right.resize(7);
	for (int i=0;i<7;i++)
	{
		cart_goal_.q_goal_right[i] = joints[i]; //specify the goal js pose
	}
	cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmPlanningInterface::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
	finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
		ROS_INFO("return code: %d",cart_result_.return_code);
	if (!finished_before_timeout_ || cart_result_.return_code==cwru_action::cwru_baxter_cart_moveResult::RT_ARM_PATH_NOT_VALID || cart_result_.return_code!=cwru_action::cwru_baxter_cart_moveResult::SUCCESS) {
		return false;
	}
	computed_arrival_time_= cart_result_.computed_arrival_time; //action_client.get_computed_arrival_time();
	//    ROS_INFO("computed move time: %f",computed_arrival_time_);
	return true;
}

bool ArmPlanningInterface::planPath(Vector3f plane_normal, Vector3f major_axis, Vector3f centroid) {
	geometry_msgs::PoseStamped pose;
	Affine3d Affine_des_gripper;
	Vector3d xvec_des,yvec_des,zvec_des,origin_des;
	
	Matrix3d Rmat;
	for (int i=0;i<3;i++) {
		origin_des[i] = centroid[i]; // convert to double precision
		zvec_des[i] = -plane_normal[i]; //want tool z pointing OPPOSITE surface normal
		xvec_des[i] = major_axis[i];
	}
//	origin_des[2]+=0.02; //raise up 2cm
	yvec_des = zvec_des.cross(xvec_des); //construct consistent right-hand triad
	Rmat.col(0)= xvec_des;
	Rmat.col(1)= yvec_des;
	Rmat.col(2)= zvec_des;
	Affine_des_gripper.linear()=Rmat;
	Affine_des_gripper.translation()=origin_des;
	
	//convert des pose from Affine to geometry_msgs::PoseStamped
	pose.pose = transformEigenAffine3dToPose(Affine_des_gripper);
	
	return planPath(pose);
}

bool ArmPlanningInterface::executePath(double timeout) {
	//    ROS_INFO("requesting execution of planned path");
	cart_goal_.command_code = cwru_action::cwru_baxter_cart_moveGoal::RT_ARM_EXECUTE_PLANNED_PATH;
	cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmPlanningInterface::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
	if (timeout == 0) {
		finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(computed_arrival_time_+2.0));
	} else {
		finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(timeout));
	}
	
	if (!finished_before_timeout_ || cart_result_.return_code!=cwru_action::cwru_baxter_cart_moveResult::SUCCESS) {
		//        ROS_WARN("did not complete move in expected time");
		return false;
	}
	return true;
}

//send goal command to request right-arm joint angles; these will be stored in internal variable
Vector7d ArmPlanningInterface::getJointAngles(void) {
	Vector7d joints;
	//   ROS_INFO("requesting right-arm joint angles");
	cart_goal_.command_code = cwru_action::cwru_baxter_cart_moveGoal::RT_ARM_GET_Q_DATA;
	cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmPlanningInterface::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
	finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
	for (int i = 0; i < 7; i++) {
		joints[i] = cart_result_.q_arm_right[i];
	}
	return joints;
}
geometry_msgs::PoseStamped ArmPlanningInterface::getGripperPose(void) {
	// debug: compare this to output of:
	//rosrun tf tf_echo torso yale_gripper_framev
	geometry_msgs::PoseStamped gripper_pose;
	//    ROS_INFO("requesting right-arm tool pose");
	cart_goal_.command_code = cwru_action::cwru_baxter_cart_moveGoal::RT_ARM_GET_TOOL_POSE;
	cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmPlanningInterface::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
	finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
	if (!finished_before_timeout_ || cart_result_.return_code!=cwru_action::cwru_baxter_cart_moveResult::SUCCESS) {
		//        ROS_WARN("did not respond within timeout");
		return gripper_pose;
	}
	gripper_pose = cart_result_.current_pose_gripper_right;
	//        ROS_INFO("move returned success; right arm tool pose: ");
	//        ROS_INFO("origin w/rt torso = %f, %f, %f ",tool_pose_stamped_.pose.position.x,
	//                tool_pose_stamped_.pose.position.y,tool_pose_stamped_.pose.position.z);
	//        ROS_INFO("quaternion x,y,z,w: %f, %f, %f, %f",tool_pose_stamped_.pose.orientation.x,
	//                tool_pose_stamped_.pose.orientation.y,tool_pose_stamped_.pose.orientation.z,
	//                tool_pose_stamped_.pose.orientation.w);
	return gripper_pose;
}

#define EXECUTE()	if(planPath(next)){ if(!executePath()) {	return false;} } else 	return false
bool ArmPlanningInterface::colorMovement(string color, geometry_msgs::PoseStamped block_pose) {
	ROS_INFO("executing motion for %s block", color.c_str());

	geometry_msgs::PoseStamped next = block_pose;

	if (color.compare("red")==0) {
		next = addPosOffset(block_pose, collision_offset);
		EXECUTE();
		ros::Duration(5.0).sleep();
		next = addPosOffset(block_pose, gripper_offset);
		EXECUTE();
		ros::Duration(5.0).sleep();
		
		gripper.gripper_close();
		next = addPosOffset(block_pose, collision_offset);
		EXECUTE();
		ros::Duration(5.0).sleep();
		next = addPosOffset(next, drop_offset_left);
		EXECUTE();
		ros::Duration(5.0).sleep();
		next = subPosOffset(next, collision_offset);
		EXECUTE();
		ros::Duration(5.0).sleep();
		
		gripper.gripper_open();
		ros::Duration(5.0).sleep();
	} else if (color.compare("blue")==0) {
		next = addPosOffset(block_pose, collision_offset);
		EXECUTE();
		ros::Duration(5.0).sleep();
		next = addPosOffset(block_pose, gripper_offset);
		EXECUTE();
		ros::Duration(5.0).sleep();
		gripper.gripper_close();
		next = addPosOffset(block_pose, collision_offset);
		EXECUTE();
		ros::Duration(5.0).sleep();
		takeALook();
		ros::Duration(5.0).sleep();
		next = addPosOffset(block_pose, collision_offset);
		EXECUTE();
		ros::Duration(5.0).sleep();
		next = addPosOffset(block_pose, gripper_offset);
		EXECUTE();
		ros::Duration(5.0).sleep();
		gripper.gripper_open();
		ros::Duration(5.0).sleep();
	} else if (color.compare("white")==0) {
		next = addPosOffset(block_pose, collision_offset);
		EXECUTE();
		ros::Duration(5.0).sleep();
		next = addPosOffset(block_pose, gripper_offset);
		EXECUTE();
		ros::Duration(5.0).sleep();
		gripper.gripper_close();
		ros::Duration(5.0).sleep();
		next = addPosOffset(block_pose, collision_offset);
		EXECUTE();
		ros::Duration(5.0).sleep();
		next = addPosOffset(next, drop_offset_right);
		EXECUTE();
		ros::Duration(5.0).sleep();
		next = subPosOffset(next, collision_offset);
		EXECUTE();
		ros::Duration(5.0).sleep();
		gripper.gripper_open();
		ros::Duration(5.0).sleep();
	} else if (color.compare("black")==0) {
		next = addPosOffset(block_pose, collision_offset);
		EXECUTE();
		ros::Duration(5.0).sleep();
		next = addPosOffset(block_pose, gripper_offset);
		EXECUTE();
		ros::Duration(5.0).sleep();
		gripper.gripper_close();
		ros::Duration(5.0).sleep();
		next = addPosOffset(block_pose, collision_offset);
		EXECUTE();
		ros::Duration(5.0).sleep();
		next = addPosOffset(block_pose, gripper_offset);
		EXECUTE();
		ros::Duration(5.0).sleep();
		gripper.gripper_open();
		ros::Duration(5.0).sleep();
	} else if (color.compare("green")==0) {
		next = addPosOffset(block_pose, collision_offset);
		EXECUTE();
		ros::Duration(5.0).sleep();
		next = addPosOffset(block_pose, gripper_offset);
		EXECUTE();
		ros::Duration(5.0).sleep();
		gripper.gripper_close();
		ros::Duration(5.0).sleep();
		next = addPosOffset(block_pose, collision_offset);
		EXECUTE();
		ros::Duration(5.0).sleep();
		takeALook();
		ros::Duration(5.0).sleep();
		next = addPosOffset(block_pose, collision_offset);
		EXECUTE();
		ros::Duration(5.0).sleep();
		next = addPosOffset(next, drop_offset_left);
		EXECUTE();
		ros::Duration(5.0).sleep();
		next = subPosOffset(next, collision_offset);
		EXECUTE();
		ros::Duration(5.0).sleep();
		gripper.gripper_open();
		ros::Duration(5.0).sleep();
	} else if (color.compare("wood")==0) {
		next = addPosOffset(block_pose, collision_offset);
		EXECUTE();
		ros::Duration(5.0).sleep();
		next = addPosOffset(block_pose, gripper_offset);
		EXECUTE();
		ros::Duration(5.0).sleep();
		gripper.gripper_close();
		next = addPosOffset(block_pose, collision_offset);
		EXECUTE();
		ros::Duration(5.0).sleep();
		next = addPosOffset(block_pose, gripper_offset);
		EXECUTE();
		ros::Duration(5.0).sleep();
		gripper.gripper_open();
		ros::Duration(5.0).sleep();
	}
	return false;
}
bool ArmPlanningInterface::colorMovement(string color, geometry_msgs::Pose block_pose) {
	geometry_msgs::PoseStamped stamp_pose;
	stamp_pose.pose = block_pose;
	return planPath(stamp_pose);
}
void ArmPlanningInterface::convToStampPose(std::vector<geometry_msgs::PoseStamped> &pose_seq, std::vector<Vector3f> &position_seq, Quaterniond &orientation) {
	int size = (int)pose_seq.size();
	for (int i = 0; i < size; i++) {
		(pose_seq[i]).pose.position.x = (position_seq[i])[0];
		(pose_seq[i]).pose.position.y = (position_seq[i])[1];
		(pose_seq[i]).pose.position.z = (position_seq[i])[2];
		(pose_seq[i]).pose.orientation.x = orientation.x();
		(pose_seq[i]).pose.orientation.y = orientation.y();
		(pose_seq[i]).pose.orientation.z = orientation.z();
		(pose_seq[i]).pose.orientation.w = orientation.w();
	}
	
}

geometry_msgs::Pose ArmPlanningInterface::transformEigenAffine3dToPose(Affine3d e) {
	Vector3d Oe;
	Matrix3d Re;
	geometry_msgs::Pose pose;
	Oe = e.translation();
	Re = e.linear();
	
	Quaterniond q(Re); // convert rotation matrix Re to a quaternion, q
	pose.position.x = Oe(0);
	pose.position.y = Oe(1);
	pose.position.z = Oe(2);
	
	pose.orientation.x = q.x();
	pose.orientation.y = q.y();
	pose.orientation.z = q.z();
	pose.orientation.w = q.w();
	
	return pose;
}

geometry_msgs::PoseStamped ArmPlanningInterface::convToStampPose(Vector3f plane_normal, Vector3f major_axis, Vector3f centroid) {
	geometry_msgs::PoseStamped pose;
	Affine3d Affine_des_gripper;
	Vector3d xvec_des,yvec_des,zvec_des,origin_des;
	
	Matrix3d Rmat;
	for (int i=0;i<3;i++) {
		origin_des[i] = centroid[i]; // convert to double precision
		zvec_des[i] = -plane_normal[i]; //want tool z pointing OPPOSITE surface normal
		xvec_des[i] = major_axis[i];
	}
//	origin_des[2]+=0.02; //raise up 2cm
	yvec_des = zvec_des.cross(xvec_des); //construct consistent right-hand triad
	Rmat.col(0)= xvec_des;
	Rmat.col(1)= yvec_des;
	Rmat.col(2)= zvec_des;
	Affine_des_gripper.linear()=Rmat;
	Affine_des_gripper.translation()=origin_des;
	
	//convert des pose from Affine to geometry_msgs::PoseStamped
	pose.pose = transformEigenAffine3dToPose(Affine_des_gripper);
	return pose;
}
geometry_msgs::Pose ArmPlanningInterface::convToPose(Vector3f plane_normal, Vector3f major_axis, Vector3f centroid) {
	geometry_msgs::PoseStamped stamp_pose;
	stamp_pose = convToStampPose(plane_normal, major_axis, centroid);
	return stamp_pose.pose;
}
geometry_msgs::PoseStamped ArmPlanningInterface::addPosOffset(geometry_msgs::PoseStamped pose, Vector3d offset) {
	geometry_msgs::PoseStamped result;
	result.pose.position.x=pose.pose.position.x+offset[0];
	result.pose.position.y=pose.pose.position.y+offset[1];
	result.pose.position.z=pose.pose.position.z+offset[2];
	result.pose.orientation.x=pose.pose.orientation.x;
	result.pose.orientation.y=pose.pose.orientation.y;
	result.pose.orientation.z=pose.pose.orientation.z;
	result.pose.orientation.w=pose.pose.orientation.w;
	return result;
}
geometry_msgs::PoseStamped ArmPlanningInterface::subPosOffset(geometry_msgs::PoseStamped pose, Vector3d offset) {
	geometry_msgs::PoseStamped result;
	result.pose.position.x=pose.pose.position.x-offset[0];
	result.pose.position.y=pose.pose.position.y-offset[1];
	result.pose.position.z=pose.pose.position.z-offset[2];
	result.pose.orientation.x=pose.pose.orientation.x;
	result.pose.orientation.y=pose.pose.orientation.y;
	result.pose.orientation.z=pose.pose.orientation.z;
	result.pose.orientation.w=pose.pose.orientation.w;
	return result;
}
geometry_msgs::PoseStamped ArmPlanningInterface::addPose(geometry_msgs::PoseStamped pose_a, geometry_msgs::PoseStamped pose_b) {
	geometry_msgs::PoseStamped result;
	result.pose.position.x=pose_a.pose.position.x+pose_b.pose.position.x;
	result.pose.position.y=pose_a.pose.position.y+pose_b.pose.position.y;
	result.pose.position.z=pose_a.pose.position.z+pose_b.pose.position.z;
	result.pose.orientation.x=pose_a.pose.orientation.x+pose_b.pose.orientation.x;
	result.pose.orientation.y=pose_a.pose.orientation.y+pose_b.pose.orientation.y;
	result.pose.orientation.z=pose_a.pose.orientation.z+pose_b.pose.orientation.z;
	result.pose.orientation.w=pose_a.pose.orientation.w+pose_b.pose.orientation.w;
	return result;
}