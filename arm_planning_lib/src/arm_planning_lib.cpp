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
		       ROS_INFO("retrying...");
	}
	ROS_INFO("connected to action server"); // if here, then we connected to the server;
	collision_offset << 0, 0, 0.4;
	gripper_offset << -0.030, 0, 0.12;
	/*gripper_pose.pose.position.x = ;
	gripper_pose.pose.position.y =;
	gripper_pose.pose.position.z =;
	gripper_pose.pose.orientation.x =;
	gripper_pose.pose.orientation.y =;
	gripper_pose.pose.orientation.z =;
	gripper_pose.pose.orientation.w =;
*/
	arm_back_joints << -0.687578798801038, -1.2286859555710585, 1.7230128939734446, 1.4960121051119204, -0.3406034304607237, 1.6922049512515933, -2.6852473873167133;
	arm_back_pose.pose.position.x = 0.48336029291;
	arm_back_pose.pose.position.y = -0.345984422306;
	arm_back_pose.pose.position.z = 0.442497286433;
	arm_back_pose.pose.orientation.x = 1;
	arm_back_pose.pose.orientation.y = 0;
	arm_back_pose.pose.orientation.z = 0;
	arm_back_pose.pose.orientation.w = 0;
	
	//drop_offset_left << -0.627811922278, -0.37037794008, -0.01957099178;
	drop_offset_left << -0.1, -0.25, 0;
	//drop_offset_right << -0.02005453231, 0.29488994572, 0.00129911747;
	drop_offset_right << 0.1, 0.25, 0;
	take_look_joints << -0.6706028380738552, 0.3385839239076704, -3.0484307475016825, -1.1967584710123835, -0.29302672221528303, 1.4968265397170557, 1.7967659166798897;
	take_look_pose.pose.position.x = 0.54666548495;
	take_look_pose.pose.position.y = -0.302305563037;
	take_look_pose.pose.position.z = 0.366209878016;
	take_look_pose.pose.orientation.x = -0.547922120529;
	take_look_pose.pose.orientation.y = -0.0239951476795;
	take_look_pose.pose.orientation.z = -0.0745942473519;
	take_look_pose.pose.orientation.w = 0.832851295841;
	

	/*
pre_grab
pose: 
  position: 
    x: 0.62630899645
    y: -0.00805334156627
    z: 0.206422664913
  orientation: 
    x: 0.629827329351
    y: 0.776484996568
    z: 0.0187988533019
    w: -0.00593198287789

grab
pose: 
  position: 
    x: 0.627811922278
    y: -0.00340972086974
    z: 0.0543679733262
  orientation: 
    x: 0.629826654045
    y: 0.776525376337
    z: 0.0171547560392
    w: -0.0056956215282

drop_left
  position: 
    x: 0.618217876627
    y: -0.373787660953
    z: 0.0347969815439
  orientation: 
    x: 0.629429968147
    y: 0.776821296576
    z: 0.0183006089831
    w: -0.00562815199012

drop_right
  position: 
    x: 0.607757389961
    y: 0.29148022485
    z: 0.0556670907964
  orientation: 
    x: 0.629136911805
    y: 0.777105813518
    z: 0.0147144639623
    w: -0.00876272515292

gripper at table top
  position: 
    x: 0.575521702564
    y: -0.22915244597
    z: 0.0475048224968
  orientation: 
    x: 0.62945411565
    y: 0.776795019467
    z: 0.0183889186523
    w: -0.00623391483899

hand at table top
pose: 
  position: 
    x: 0.578822113329
    y: -0.222901535766
    z: -0.129146973155
  orientation: 
    x: 0.629548500816
    y: 0.776861010357
    z: 0.00458504421153
    w: 0.0116031494482
take a look
pose: 
  position: 
    x: 0.52542198926
    y: -0.19065849761
    z: 0.154626356725
  orientation: 
    x: -0.249821685981
    y: -0.335221027398
    z: -0.504147162678
    w: 0.755679579166

bakc to start
pose: 
  position: 
    x: 0.628566212076
    y: -0.347307547123
    z: 0.350523968221
  orientation: 
    x: 0.630141382934
    y: 0.775598336984
    z: 0.000519772148981
    w: 0.0369971217577
*/
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

#ifdef REAL_WORLD
	global_pose_offset.pose.position.x = 0;
	global_pose_offset.pose.position.y = 0;
	global_pose_offset.pose.position.z = 0;
	global_pose_offset.pose.orientation.x = 0;
	global_pose_offset.pose.orientation.y = 0;
	global_pose_offset.pose.orientation.z = 0;
	global_pose_offset.pose.orientation.w = 0;
#endif

#ifdef GAZEBO
	global_pose_offset.pose.position.x = 0;
	global_pose_offset.pose.position.y = 0;
	global_pose_offset.pose.position.z = -0.1;
	global_pose_offset.pose.orientation.x = 0;
	global_pose_offset.pose.orientation.y = 0;
	global_pose_offset.pose.orientation.z = 0;
	global_pose_offset.pose.orientation.w = 0;
#endif

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
	if(planPath(arm_back_pose)){
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
	   //ROS_INFO("return code: %d",cart_result_.return_code);
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
		//ROS_INFO("return code: %d",cart_result_.return_code);
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
	Vector3d pick_up_offset;
	pick_up_offset << 0, 0, 0.01;
	geometry_msgs::PoseStamped next = block_pose;

	if (color.compare("red")==0) {
		next = addPosOffset(block_pose, collision_offset);
		EXECUTE();
		ros::Duration(MOTION_TIME).sleep();
		next = addPosOffset(block_pose, gripper_offset);
		EXECUTE();
		ros::Duration(MOTION_TIME).sleep();
		
		gripper.gripper_close();
		//ros::Duration(1.0).sleep();
		//next = addPosOffset(next, pick_up_offset);
		//EXECUTE();
		ros::Duration(GRIPPER_TIME).sleep();
		next = addPosOffset(block_pose, collision_offset);
		EXECUTE();
		ros::Duration(MOTION_TIME).sleep();
		next = addPosOffset(next, drop_offset_left);
		EXECUTE();
		ros::Duration(MOTION_TIME).sleep();
		next = subPosOffset(next, collision_offset);
		EXECUTE();
		ros::Duration(MOTION_TIME).sleep();
		
		gripper.gripper_open();
		ros::Duration(MOTION_TIME).sleep();
	} else if (color.compare("blue")==0) {
		next = addPosOffset(block_pose, collision_offset);
		EXECUTE();
		ros::Duration(MOTION_TIME).sleep();
		next = addPosOffset(block_pose, gripper_offset);
		EXECUTE();
		ros::Duration(MOTION_TIME).sleep();
		gripper.gripper_close();
		ros::Duration(GRIPPER_TIME).sleep();
		next = addPosOffset(block_pose, collision_offset);
		EXECUTE();
		ros::Duration(MOTION_TIME).sleep();
		takeALook();
		ros::Duration(MOTION_TIME).sleep();
		next = addPosOffset(block_pose, collision_offset);
		EXECUTE();
		ros::Duration(MOTION_TIME).sleep();
		next = addPosOffset(block_pose, gripper_offset);
		EXECUTE();
		ros::Duration(MOTION_TIME).sleep();
		gripper.gripper_open();
		ros::Duration(MOTION_TIME).sleep();
	} else if (color.compare("white")==0) {
		next = addPosOffset(block_pose, collision_offset);
		EXECUTE();
		ros::Duration(MOTION_TIME).sleep();
		next = addPosOffset(block_pose, gripper_offset);
		EXECUTE();
		ros::Duration(MOTION_TIME).sleep();
		gripper.gripper_close();
		ros::Duration(GRIPPER_TIME).sleep();
		next = addPosOffset(block_pose, collision_offset);
		EXECUTE();
		ros::Duration(MOTION_TIME).sleep();
		next = addPosOffset(next, drop_offset_right);
		EXECUTE();
		ros::Duration(MOTION_TIME).sleep();
		next = subPosOffset(next, collision_offset);
		EXECUTE();
		ros::Duration(MOTION_TIME).sleep();
		gripper.gripper_open();
		ros::Duration(MOTION_TIME).sleep();
	} else if (color.compare("black")==0) {
		next = addPosOffset(block_pose, collision_offset);
		EXECUTE();
		ros::Duration(MOTION_TIME).sleep();
		next = addPosOffset(block_pose, gripper_offset);
		EXECUTE();
		ros::Duration(MOTION_TIME).sleep();
		gripper.gripper_close();
		ros::Duration(GRIPPER_TIME).sleep();
		next = addPosOffset(block_pose, collision_offset);
		EXECUTE();
		ros::Duration(MOTION_TIME).sleep();
		next = addPosOffset(block_pose, gripper_offset);
		EXECUTE();
		ros::Duration(MOTION_TIME).sleep();
		gripper.gripper_open();
		ros::Duration(MOTION_TIME).sleep();
	} else if (color.compare("green")==0) {
		next = addPosOffset(block_pose, collision_offset);
		EXECUTE();
		ros::Duration(MOTION_TIME).sleep();
		next = addPosOffset(block_pose, gripper_offset);
		EXECUTE();
		ros::Duration(MOTION_TIME).sleep();
		gripper.gripper_close();
		ros::Duration(GRIPPER_TIME).sleep();
		next = addPosOffset(block_pose, collision_offset);
		EXECUTE();
		ros::Duration(MOTION_TIME).sleep();
		takeALook();
		ros::Duration(MOTION_TIME).sleep();
		next = addPosOffset(block_pose, collision_offset);
		EXECUTE();
		ros::Duration(MOTION_TIME).sleep();
		next = addPosOffset(next, drop_offset_left);
		EXECUTE();
		ros::Duration(MOTION_TIME).sleep();
		next = subPosOffset(next, collision_offset);
		EXECUTE();
		ros::Duration(MOTION_TIME).sleep();
		gripper.gripper_open();
		ros::Duration(MOTION_TIME).sleep();
	} else if (color.compare("wood")==0) {
		next = addPosOffset(block_pose, collision_offset);
		EXECUTE();
		ros::Duration(MOTION_TIME).sleep();
		next = addPosOffset(block_pose, gripper_offset);
		EXECUTE();
		ros::Duration(MOTION_TIME).sleep();
		gripper.gripper_close();
		ros::Duration(GRIPPER_TIME).sleep();
		next = addPosOffset(block_pose, collision_offset);
		EXECUTE();
		ros::Duration(MOTION_TIME).sleep();
		next = addPosOffset(block_pose, gripper_offset);
		EXECUTE();
		ros::Duration(MOTION_TIME).sleep();
		gripper.gripper_open();
		ros::Duration(MOTION_TIME).sleep();
	}
	return true;
}
bool ArmPlanningInterface::colorMovement(string color, geometry_msgs::Pose block_pose) {
	geometry_msgs::PoseStamped stamp_pose;
	stamp_pose.pose = block_pose;
	return colorMovement(color,stamp_pose);
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