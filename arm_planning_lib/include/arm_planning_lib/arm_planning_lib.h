#ifndef ARM_PLANNING_INTERFACE_H_
#define ARM_PLANNING_INTERFACE_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cwru_action/cwru_baxter_cart_moveAction.h>
#include <gripper_move/grippermove.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <string>
#include <math.h>
//#include <baxter_traj_streamer/baxter_traj_streamer.h>

#define REAL_WORLD

using namespace std;  //just to avoid requiring std::,  ...
using namespace Eigen;
typedef Matrix<double, 7, 1> Vector7d;

#define MOTION_TIME 2.0
#define GRIPPER_TIME 4.0

class ArmPlanningInterface {
private:
	ros::NodeHandle nh_;
	GripperMove gripper;
	//messages to send/receive cartesian goals / results:
	cwru_action::cwru_baxter_cart_moveGoal cart_goal_;
	cwru_action::cwru_baxter_cart_moveResult cart_result_;
	//an action client to send goals to cartesian-move action server
	actionlib::SimpleActionClient<cwru_action::cwru_baxter_cart_moveAction> cart_move_action_client_; //("cartMoveActionServer", true);
	double computed_arrival_time_;
	bool finished_before_timeout_;
	//callback fnc for cartesian action server to return result to this node:
	void doneCb_(const actionlib::SimpleClientGoalState& state,
				 const cwru_action::cwru_baxter_cart_moveResultConstPtr& result);
	geometry_msgs::Pose transformEigenAffine3dToPose(Affine3d e);
	geometry_msgs::PoseStamped addPosOffset(geometry_msgs::PoseStamped pose, Vector3d offset);
	geometry_msgs::PoseStamped subPosOffset(geometry_msgs::PoseStamped pose, Vector3d offset);
	geometry_msgs::PoseStamped addPose(geometry_msgs::PoseStamped pose_a, geometry_msgs::PoseStamped pose_b);
public:
	ArmPlanningInterface(ros::NodeHandle* nodehandle); //define the body of the constructor outside of class definition
	
	~ArmPlanningInterface(void) {
	}

	geometry_msgs::PoseStamped arm_back_pose;
	Vector7d arm_back_joints;

	geometry_msgs::PoseStamped take_look_pose;
	Vector7d take_look_joints;

	geometry_msgs::PoseStamped pre_grab_pose;
	geometry_msgs::PoseStamped grab_pose;

	Vector3d gripper_offset;
	geometry_msgs::PoseStamped gripper_pose;
	Vector3d collision_offset;
	Vector3d drop_offset_left;
	Vector3d drop_offset_right;

	geometry_msgs::PoseStamped global_pose_offset;
	Vector7d global_joints_offset;

	Quaterniond default_orientation;

	bool moveArmsBack(void);
	
	Vector7d getJointAngles(void);
	geometry_msgs::PoseStamped getGripperPose(void);
	
	bool planPath(geometry_msgs::PoseStamped pose);
	bool planPath(geometry_msgs::Pose pose);
	bool planPath(Vector7d joints);
	bool planPath(Vector3f plane_normal, Vector3f major_axis, Vector3f centroid) ;
	
	bool executePath(double timeout = 0.0);
	
	bool colorMovement(string color, geometry_msgs::PoseStamped block_pose);
	bool colorMovement(string color, geometry_msgs::Pose block_pose);
	
	bool takeALook(){if(planPath(take_look_pose)){ if(!executePath()) {	return false;} } else 	return false;}

	void convToStampPose(std::vector<geometry_msgs::PoseStamped> &pose_seq, std::vector<Vector3f> &position_seq, Quaterniond &orientation);
	geometry_msgs::PoseStamped convToStampPose(Vector3f plane_normal, Vector3f major_axis, Vector3f centroid);
	geometry_msgs::Pose convToPose(Vector3f plane_normal, Vector3f major_axis, Vector3f centroid);
	
};

#endif  // this closes the header-include trick...ALWAYS need one of these to match #ifndef
