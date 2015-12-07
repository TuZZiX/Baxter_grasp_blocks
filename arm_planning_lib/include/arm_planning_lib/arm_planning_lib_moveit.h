#ifndef ARM_PLANNING_INTERFACE_MOVEIT_H_
#define ARM_PLANNING_INTERFACE_MOVEIT_H_

#include <ros/ros.h>
#include <gripper_move/grippermove.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <string>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
//#include <baxter_traj_streamer/baxter_traj_streamer.h>

#define GAZEBO
#define MOVEIT

using namespace std;  //just to avoid requiring std::,  ...
using namespace Eigen;
typedef Matrix<double, 7, 1> Vector7d;

#ifdef MOVEIT
#define MOTION_TIME_MOVEIT 0.5
#define GRIPPER_TIME_MOVEIT 4.0
#endif

class MoveitPlanningInterface {
private:
	ros::NodeHandle nh_;
	GripperMove gripper;
	double computed_arrival_time_;
	bool finished_before_timeout_;

	ros::AsyncSpinner spinner;
	moveit::planning_interface::MoveGroup right_arm, left_arm;
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	moveit::planning_interface::MoveGroup::Plan my_plan;
	ros::Publisher display_publisher;
	moveit_msgs::DisplayTrajectory display_trajectory;

	geometry_msgs::Pose transformEigenAffine3dToPose(Affine3d e);
	geometry_msgs::Pose addPosOffset(geometry_msgs::Pose pose, Vector3d offset);
	geometry_msgs::Pose subPosOffset(geometry_msgs::Pose pose, Vector3d offset);
	geometry_msgs::Pose addPose(geometry_msgs::Pose pose_a, geometry_msgs::Pose pose_b);

	geometry_msgs::Pose arm_back_pose;
	Vector7d arm_back_joints;
	geometry_msgs::Pose left_arm_back_pose;

	geometry_msgs::Pose take_look_pose;
	Vector7d take_look_joints;

	Vector3d gripper_offset;
	geometry_msgs::Pose gripper_pose;
	Vector3d collision_offset;
	Vector3d drop_offset_left;
	Vector3d drop_offset_right;

	geometry_msgs::Pose global_pose_offset;
	Vector7d global_joints_offset;

public:
	MoveitPlanningInterface(ros::NodeHandle* nodehandle); //define the body of the constructor outside of class definition
	
	~MoveitPlanningInterface(void) {
	}

	bool moveArmsBack(void);
	
	Vector7d getJointAngles(void);
	geometry_msgs::Pose getGripperPose(void);
	
	void displayPath();
	bool planPath(geometry_msgs::Pose pose);
	bool planPath(Vector7d joints);
	bool planPath(Vector3f plane_normal, Vector3f major_axis, Vector3f centroid) ;
	
	bool executePath();
	bool executeAsync();
	void stop();
	
	bool colorMovement(string color, geometry_msgs::Pose block_pose);
	
	bool takeALook(){if(planPath(take_look_pose)){ if(!executePath()) {	return false;} } else 	return false;}

	geometry_msgs::Pose convToStampPose(Vector3f plane_normal, Vector3f major_axis, Vector3f centroid);
	geometry_msgs::Pose convToPose(Vector3f plane_normal, Vector3f major_axis, Vector3f centroid);
	
};

#endif  // this closes the header-include trick...ALWAYS need one of these to match #ifndef