#include <arm_planning_lib/arm_planning_lib_moveit.h>
MoveitPlanningInterface::MoveitPlanningInterface(ros::NodeHandle* nodehandle): nh_(*nodehandle), 
gripper(&nh_), spinner(1), right_arm("right_arm"), left_arm("left_arm"), 
display_publisher(nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true)) { // constructor
	// attempt to connect to the server:
	spinner.start();

	// We can print the name of the reference frame for this robot.
	ROS_INFO("Reference robot frame: %s", right_arm.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
	ROS_INFO("Reference end-effector frame: %s", right_arm.getEndEffectorLink().c_str());

	right_arm.setPlanningTime(1.0);
/*
	moveit_msgs::CollisionObject collision_object;
	collision_object.header.frame_id = "torso";

	collision_object.id = "kinect";

	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 0.3;
	primitive.dimensions[1] = 0.29;
	primitive.dimensions[2] = 0.06;

	geometry_msgs::Pose box_pose;
	box_pose.orientation.w = 1.0;
	box_pose.position.x =  0;
	box_pose.position.y = 0.20;
	box_pose.position.z =  0.74;


	collision_object.primitives.push_back(primitive);
	collision_object.primitive_poses.push_back(box_pose);
	collision_object.operation = collision_object.ADD;

	std::vector<moveit_msgs::CollisionObject> collision_objects;  
	collision_objects.push_back(collision_object);  

	ROS_INFO("Add an kinect into the world");  
	planning_scene_interface.addCollisionObjects(collision_objects);*/

	collision_offset << -0.02, 0.03, 0.4;
	gripper_offset << -0.02, 0.03, 0.175;
	/*gripper_pose.position.x = ;
	gripper_pose.position.y =;
	gripper_pose.position.z =;
	gripper_pose.orientation.x =;
	gripper_pose.orientation.y =;
	gripper_pose.orientation.z =;
	gripper_pose.orientation.w =;
*/
	arm_back_joints << -0.97578798801038, -1.2286859555710585, 1.7230128939734446, 1.4960121051119204, -0.3406034304607237, 1.6922049512515933, -2.6852473873167133;
	arm_back_pose.position.x = 0.48336029291;
	arm_back_pose.position.y = -0.345984422306;
	arm_back_pose.position.z = 0.442497286433;
	arm_back_pose.orientation.x = 1;
	arm_back_pose.orientation.y = 0;
	arm_back_pose.orientation.z = 0;
	arm_back_pose.orientation.w = 0;
	
	left_arm_back_pose.position.x = 0.356899870469;
	left_arm_back_pose.position.y = 0.553228163753;
	left_arm_back_pose.position.z = -0.333650371585;
	left_arm_back_pose.orientation.x = 1;
	left_arm_back_pose.orientation.y = 0;
	left_arm_back_pose.orientation.z = 0;
	left_arm_back_pose.orientation.w = 0;

	drop_offset_left << -0.15, -0.25, 0;
	drop_offset_right << -0.15, 0.25, 0;

	take_look_joints << -0.6706028380738552, 0.3385839239076704, -3.0484307475016825, -1.1967584710123835, -0.29302672221528303, 1.4968265397170557, 1.7967659166798897;
	take_look_pose.position.x = 0.54666548495;
	take_look_pose.position.y = -0.102305563037;
	take_look_pose.position.z = 0.366209878016;
	take_look_pose.orientation.x = -0.547922120529;
	take_look_pose.orientation.y = -0.0239951476795;
	take_look_pose.orientation.z = -0.0745942473519;
	take_look_pose.orientation.w = 0.832851295841;

	take_look_joints << -0.18503156094537968, 0.44527797626583865, 0.36021593694382065, 1.827325037963473, 1.0199002550686511, 1.0137709796128629, 1.2701787563643496;

#ifdef REAL_WORLD
	global_pose_offset.position.x = 0.02;
	global_pose_offset.position.y = 0.01;
	global_pose_offset.position.z = 0;
	global_pose_offset.orientation.x = 0.13;
	global_pose_offset.orientation.y = -0.20;
	global_pose_offset.orientation.z = 0;
	global_pose_offset.orientation.w = 0.01;
#endif

/*
pose: 
  position: 
    x: 0.748721050657
    y: 0.0345305831626
    z: 0.0631545313051
  orientation: 
    x: 0.774185882473
    y: 0.632461597908
    z: 0.020930375697
    w: 0.0138009392546


  position: 
    x: 0.763820648866
    y: 0.0499207273826
    z: 0.0627710565632
  orientation: 
    x: 0.907066743129
    y: 0.42032783865
    z: 0.0224578714723
    w: 0.00707640972079

*/
#ifdef GAZEBO
	global_pose_offset.position.x = 0;
	global_pose_offset.position.y = 0;
	global_pose_offset.position.z = -0.1;
	global_pose_offset.orientation.x = 0;
	global_pose_offset.orientation.y = 0;
	global_pose_offset.orientation.z = 0;
	global_pose_offset.orientation.w = 0;
#endif

	global_joints_offset << 0,0,0,0,0,0,0;

}


void MoveitPlanningInterface::displayPath() {
	ROS_INFO("Visualizing planed Path");    
	display_trajectory.trajectory_start = my_plan.start_state_;
	display_trajectory.trajectory.push_back(my_plan.trajectory_);
	display_publisher.publish(display_trajectory);
    /* Sleep to give Rviz time to visualize the plan. */
	sleep(5.0);
}

bool MoveitPlanningInterface::moveArmsBack(void) {
	gripper.gripper_open();

	left_arm.setPoseTarget(left_arm_back_pose);
	left_arm.move();
	if(planPath(arm_back_joints)){
		if(!executePath()) {
			return false;
		}
	} else {
		return false;
	}
	return true;
}

bool MoveitPlanningInterface::planPath(geometry_msgs::Pose pose) {
	ROS_INFO("requesting a cartesian-space motion plan");
	pose = addPose(pose, global_pose_offset);

	right_arm.setPoseTarget(pose);
	bool success = right_arm.plan(my_plan);
	return success;
}

bool MoveitPlanningInterface::planPath(Vector7d joints) {
	ROS_INFO("requesting a joint-space motion plan");
	joints = joints + global_joints_offset;

	std::vector<double> group_variable_values(7);

	for (int i = 0; i < 7; ++i)
	{
		group_variable_values[i] = joints[i];
	}

	right_arm.setJointValueTarget(group_variable_values);
	bool success = right_arm.plan(my_plan);

	return success;
}

bool MoveitPlanningInterface::planPath(Vector3f plane_normal, Vector3f major_axis, Vector3f centroid) {
	geometry_msgs::Pose pose;
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
	pose = transformEigenAffine3dToPose(Affine_des_gripper);
	
	return planPath(pose);
}

bool MoveitPlanningInterface::executePath() {
	right_arm.move();
	return true;
}

bool MoveitPlanningInterface::executeAsync() {
	right_arm.asyncMove();
	return true;
}

void MoveitPlanningInterface::stop() {
	right_arm.stop();
}

//send goal command to request right-arm joint angles; these will be stored in internal variable
Vector7d MoveitPlanningInterface::getJointAngles(void) {
	Vector7d joints;
	//   ROS_INFO("requesting right-arm joint angles");
	std::vector<double> group_variable_values;
	right_arm.getCurrentState()->copyJointGroupPositions(right_arm.getCurrentState()->getRobotModel()->getJointModelGroup(right_arm.getName()), group_variable_values);
	for (int i = 0; i < 7; ++i)
	{
		joints[i] = group_variable_values[i];
	}
	return joints;
}
geometry_msgs::Pose MoveitPlanningInterface::getGripperPose(void) {
	// debug: compare this to output of:
	//rosrun tf tf_echo torso yale_gripper_framev
	moveit::planning_interface::MoveGroup *group; 
	group = new moveit::planning_interface::MoveGroup("right_arm");
	group->setEndEffectorLink("hand_left");
	geometry_msgs::PoseStamped gripper_pose = group->getCurrentPose();
	geometry_msgs::Pose pose = gripper_pose.pose;
	return pose;
}

#define EXECUTE()	if(planPath(next)){ if(!executePath()) {	return false;} } else 	return false

bool MoveitPlanningInterface::colorMovement(string color, geometry_msgs::Pose block_pose) {
	ROS_WARN("executing motion for %s block", color.c_str());
	geometry_msgs::Pose next = block_pose;

	if (color.compare("red")==0) {
		next = addPosOffset(block_pose, collision_offset);
		EXECUTE();
		ros::Duration(MOTION_TIME_MOVEIT).sleep();
		next = addPosOffset(block_pose, gripper_offset);
		EXECUTE();
		ros::Duration(GRIPPER_TIME_MOVEIT).sleep();
		
		gripper.gripper_close();
		ros::Duration(GRIPPER_TIME_MOVEIT).sleep();
		next = addPosOffset(block_pose, collision_offset);
		EXECUTE();
		ros::Duration(MOTION_TIME_MOVEIT).sleep();
		next = addPosOffset(next, drop_offset_left);
		EXECUTE();
		ros::Duration(MOTION_TIME_MOVEIT).sleep();
		next = subPosOffset(next, collision_offset);
		EXECUTE();
		ros::Duration(MOTION_TIME_MOVEIT).sleep();
		
		gripper.gripper_open();
		ros::Duration(MOTION_TIME_MOVEIT).sleep();
	} else if (color.compare("blue")==0) {
		next = addPosOffset(block_pose, collision_offset);
		EXECUTE();
		ros::Duration(MOTION_TIME_MOVEIT).sleep();
		next = addPosOffset(block_pose, gripper_offset);
		EXECUTE();
		ros::Duration(GRIPPER_TIME_MOVEIT).sleep();
		gripper.gripper_close();
		ros::Duration(GRIPPER_TIME_MOVEIT).sleep();
		next = addPosOffset(block_pose, collision_offset);
		EXECUTE();
		ros::Duration(MOTION_TIME_MOVEIT).sleep();
		takeALook();
		ros::Duration(MOTION_TIME_MOVEIT).sleep();
		next = addPosOffset(block_pose, collision_offset);
		EXECUTE();
		ros::Duration(MOTION_TIME_MOVEIT).sleep();
		next = addPosOffset(block_pose, gripper_offset);
		EXECUTE();
		ros::Duration(MOTION_TIME_MOVEIT).sleep();
		gripper.gripper_open();
		ros::Duration(MOTION_TIME_MOVEIT).sleep();
	} else if (color.compare("white")==0) {
		next = addPosOffset(block_pose, collision_offset);
		EXECUTE();
		ros::Duration(MOTION_TIME_MOVEIT).sleep();
		next = addPosOffset(block_pose, gripper_offset);
		EXECUTE();
		ros::Duration(GRIPPER_TIME_MOVEIT).sleep();
		gripper.gripper_close();
		ros::Duration(GRIPPER_TIME_MOVEIT).sleep();
		next = addPosOffset(block_pose, collision_offset);
		EXECUTE();
		ros::Duration(MOTION_TIME_MOVEIT).sleep();
		next = addPosOffset(next, drop_offset_right);
		EXECUTE();
		ros::Duration(MOTION_TIME_MOVEIT).sleep();
		next = subPosOffset(next, collision_offset);
		EXECUTE();
		ros::Duration(MOTION_TIME_MOVEIT).sleep();
		gripper.gripper_open();
		ros::Duration(MOTION_TIME_MOVEIT).sleep();
	} else if (color.compare("black")==0) {
		next = addPosOffset(block_pose, collision_offset);
		EXECUTE();
		ros::Duration(MOTION_TIME_MOVEIT).sleep();
		next = addPosOffset(block_pose, gripper_offset);
		EXECUTE();
		ros::Duration(GRIPPER_TIME_MOVEIT).sleep();
		gripper.gripper_close();
		ros::Duration(GRIPPER_TIME_MOVEIT).sleep();
		next = addPosOffset(block_pose, collision_offset);
		EXECUTE();
		ros::Duration(MOTION_TIME_MOVEIT).sleep();
		next = addPosOffset(block_pose, gripper_offset);
		EXECUTE();
		ros::Duration(MOTION_TIME_MOVEIT).sleep();
		gripper.gripper_open();
		ros::Duration(MOTION_TIME_MOVEIT).sleep();
	} else if (color.compare("green")==0) {
		next = addPosOffset(block_pose, collision_offset);
		EXECUTE();
		ros::Duration(MOTION_TIME_MOVEIT).sleep();
		next = addPosOffset(block_pose, gripper_offset);
		EXECUTE();
		ros::Duration(GRIPPER_TIME_MOVEIT).sleep();
		gripper.gripper_close();
		ros::Duration(GRIPPER_TIME_MOVEIT).sleep();
		next = addPosOffset(block_pose, collision_offset);
		EXECUTE();
		ros::Duration(MOTION_TIME_MOVEIT).sleep();
		takeALook();
		ros::Duration(MOTION_TIME_MOVEIT).sleep();
		next = addPosOffset(block_pose, collision_offset);
		EXECUTE();
		ros::Duration(MOTION_TIME_MOVEIT).sleep();
		next = addPosOffset(next, drop_offset_left);
		EXECUTE();
		ros::Duration(MOTION_TIME_MOVEIT).sleep();
		next = subPosOffset(next, collision_offset);
		EXECUTE();
		ros::Duration(MOTION_TIME_MOVEIT).sleep();
		gripper.gripper_open();
		ros::Duration(MOTION_TIME_MOVEIT).sleep();
	} else if (color.compare("wood")==0) {
		next = addPosOffset(block_pose, collision_offset);
		EXECUTE();
		ros::Duration(MOTION_TIME_MOVEIT).sleep();
		next = addPosOffset(block_pose, gripper_offset);
		EXECUTE();
		ros::Duration(GRIPPER_TIME_MOVEIT).sleep();
		gripper.gripper_close();
		ros::Duration(GRIPPER_TIME_MOVEIT).sleep();
		next = addPosOffset(block_pose, collision_offset);
		EXECUTE();
		ros::Duration(MOTION_TIME_MOVEIT).sleep();
		takeALook();
		ros::Duration(MOTION_TIME_MOVEIT).sleep();
		next = addPosOffset(next, drop_offset_right);
		EXECUTE();
		ros::Duration(MOTION_TIME_MOVEIT).sleep();
		next = subPosOffset(next, collision_offset);
		EXECUTE();
		ros::Duration(MOTION_TIME_MOVEIT).sleep();
		gripper.gripper_open();
		ros::Duration(MOTION_TIME_MOVEIT).sleep();
	}
	return true;
}

geometry_msgs::Pose MoveitPlanningInterface::transformEigenAffine3dToPose(Affine3d e) {
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

geometry_msgs::Pose MoveitPlanningInterface::convToPose(Vector3f plane_normal, Vector3f major_axis, Vector3f centroid) {
	geometry_msgs::Pose pose;
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
	pose = transformEigenAffine3dToPose(Affine_des_gripper);
	return pose;
}
geometry_msgs::Pose MoveitPlanningInterface::addPosOffset(geometry_msgs::Pose pose, Vector3d offset) {
	geometry_msgs::Pose result;
	result.position.x=pose.position.x+offset[0];
	result.position.y=pose.position.y+offset[1];
	result.position.z=pose.position.z+offset[2];
	result.orientation.x=pose.orientation.x;
	result.orientation.y=pose.orientation.y;
	result.orientation.z=pose.orientation.z;
	result.orientation.w=pose.orientation.w;
	return result;
}
geometry_msgs::Pose MoveitPlanningInterface::subPosOffset(geometry_msgs::Pose pose, Vector3d offset) {
	geometry_msgs::Pose result;
	result.position.x=pose.position.x-offset[0];
	result.position.y=pose.position.y-offset[1];
	result.position.z=pose.position.z-offset[2];
	result.orientation.x=pose.orientation.x;
	result.orientation.y=pose.orientation.y;
	result.orientation.z=pose.orientation.z;
	result.orientation.w=pose.orientation.w;
	return result;
}
geometry_msgs::Pose MoveitPlanningInterface::addPose(geometry_msgs::Pose pose_a, geometry_msgs::Pose pose_b) {
	geometry_msgs::Pose result;
	result.position.x=pose_a.position.x+pose_b.position.x;
	result.position.y=pose_a.position.y+pose_b.position.y;
	result.position.z=pose_a.position.z+pose_b.position.z;
	result.orientation.x=pose_a.orientation.x+pose_b.orientation.x;
	result.orientation.y=pose_a.orientation.y+pose_b.orientation.y;
	result.orientation.z=pose_a.orientation.z+pose_b.orientation.z;
	result.orientation.w=pose_a.orientation.w+pose_b.orientation.w;
	return result;
}