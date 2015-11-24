#include <arm_planning_lib/arm_planning_lib.h>

ArmPlanningInterface::ArmPlanningInterface(ros::NodeHandle* nodehandle): nh_(*nodehandle),
cart_move_action_client_("cartMoveActionServer", true) { // constructor
	
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
	
}
// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server
//int g_return_code=0;
void ArmPlanningInterface::doneCb_(const actionlib::SimpleClientGoalState& state,
								 const cwru_action::cwru_baxter_cart_moveResultConstPtr& result) {
	ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
	ROS_INFO("got return value= %d", result->return_code);
	cart_result_=*result;
}

bool ArmPlanningInterface::moveArmsBack(void) {
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
}

bool ArmPlanningInterface::planPath(geometry_msgs::PoseStamped pose) {
	
	//    ROS_INFO("requesting a cartesian-space motion plan");
	cart_goal_.command_code = cwru_action::cwru_baxter_cart_moveGoal::RT_ARM_PLAN_PATH_CURRENT_TO_GOAL_POSE;
	cart_goal_.des_pose_gripper_right = pose;
	cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmPlanningInterface::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
	finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
	//    ROS_INFO("return code: %d",cart_result_.return_code);
	if (!finished_before_timeout_ || cart_result_.return_code==cwru_action::cwru_baxter_cart_moveResult::RT_ARM_PATH_NOT_VALID || cart_result_.return_code!=cwru_action::cwru_baxter_cart_moveResult::SUCCESS) {
		return false;
	}
	computed_arrival_time_= cart_result_.computed_arrival_time; //action_client.get_computed_arrival_time();
	//    ROS_INFO("computed move time: %f",computed_arrival_time_);
	return true;
}

bool ArmPlanningInterface::planPath(Eigen::VectorXd joints) {
	ROS_INFO("requesting a joint-space motion plan");
	cart_goal_.command_code = cwru_action::cwru_baxter_cart_moveGoal::RT_ARM_PLAN_JSPACE_PATH_CURRENT_TO_QGOAL;
	cart_goal_.q_goal_right.resize(7);
	for (int i=0;i<7;i++)
	{
		cart_goal_.q_goal_right[i] = joints[i]; //specify the goal js pose
	}
	cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmPlanningInterface::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
	finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
	//	ROS_INFO("return code: %d",cart_result_.return_code);
	if (!finished_before_timeout_ || cart_result_.return_code==cwru_action::cwru_baxter_cart_moveResult::RT_ARM_PATH_NOT_VALID || cart_result_.return_code!=cwru_action::cwru_baxter_cart_moveResult::SUCCESS) {
		return false;
	}
	computed_arrival_time_= cart_result_.computed_arrival_time; //action_client.get_computed_arrival_time();
	//    ROS_INFO("computed move time: %f",computed_arrival_time_);
	return true;
}

bool ArmPlanningInterface::planPath(Eigen::Vector3d dp_displacement) {
	
	//	ROS_INFO("requesting a cartesian-space motion plan along vector");
	cart_goal_.command_code = cwru_action::cwru_baxter_cart_moveGoal::RT_ARM_PLAN_PATH_CURRENT_TO_GOAL_DP_XYZ;
	//must fill in desired vector displacement
	cart_goal_.arm_dp_right.resize(3);
	for (int i=0;i<3;i++)
	{
		cart_goal_.arm_dp_right[i] = dp_displacement[i];
	}
	cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmPlanningInterface::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
	finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
	//	ROS_INFO("return code: %d",cart_result_.return_code);
	if (!finished_before_timeout_ || cart_result_.return_code==cwru_action::cwru_baxter_cart_moveResult::RT_ARM_PATH_NOT_VALID || cart_result_.return_code!=cwru_action::cwru_baxter_cart_moveResult::SUCCESS) {
		return false;
	}
	computed_arrival_time_= cart_result_.computed_arrival_time; //action_client.get_computed_arrival_time();
	//    ROS_INFO("computed move time: %f",computed_arrival_time_);
	return true;
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
Eigen::VectorXd ArmPlanningInterface::getJointAngles(void) {
	Eigen::VectorXd joints;
	//   ROS_INFO("requesting right-arm joint angles");
	cart_goal_.command_code = cwru_action::cwru_baxter_cart_moveGoal::RT_ARM_GET_Q_DATA;
	cart_move_action_client_.sendGoal(cart_goal_, boost::bind(&ArmPlanningInterface::doneCb_, this, _1, _2)); // we could also name additional callback functions here, if desired
	finished_before_timeout_ = cart_move_action_client_.waitForResult(ros::Duration(2.0));
	joints.resize(7);
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
bool ColorMovement(string color) {
	if (color.compare("red")==0) {
		planPath();
		executePath();
	}
}
void ArmPlanningInterface::convToPose(std::vector<geometry_msgs::PoseStamped> &pose_seq, std::vector<Eigen::Vector3f> &position_seq, Eigen::Quaterniond &orientation) {
	int size = (int)pose_seq.size();
	for (int i = 0; i < size; i++) {
		(pose_seq[i]).pose.position.x = (position_seq[i])[0];
		(pose_seq[i]).pose.position.y = (position_seq[i])[1];
		(pose_seq[i]).pose.position.z = (position_seq[i])[2] + 0.1;
		(pose_seq[i]).pose.orientation.x = orientation.x();
		(pose_seq[i]).pose.orientation.y = orientation.y();
		(pose_seq[i]).pose.orientation.z = orientation.y();
		(pose_seq[i]).pose.orientation.w = orientation.w();
	}
	
}