#ifndef ARM_PLANNING_INTERFACE_H_
#define ARM_PLANNING_INTERFACE_H_


#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cwru_action/cwru_baxter_cart_moveAction.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <string>

using namespace std;  //just to avoid requiring std::, Eigen:: ...

//define a class to encapsulate some of the tedium of populating and sending goals,
// and interpreting responses
class ArmPlanningInterface {
private:
	ros::NodeHandle nh_;
	
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
public:
	ArmPlanningInterface(ros::NodeHandle* nodehandle); //define the body of the constructor outside of class definition
	
	~ArmPlanningInterface(void) {
	}
	bool moveArmsBack(void);
	
	Eigen::VectorXd getJointAngles(void);
	geometry_msgs::PoseStamped getGripperPose(void);
	
	bool planPath(geometry_msgs::PoseStamped pose);
	bool planPath(Eigen::VectorXd joints);
	bool planPath(Eigen::Vector3d dp_displacement);
	
	bool executePath(double timeout = 0.0);
	
	bool ColorMovement(string color);
	
	void convToPose(std::vector<geometry_msgs::PoseStamped> &pose_seq, std::vector<Eigen::Vector3f> &position_seq, Eigen::Quaterniond &orientation);
};

#endif  // this closes the header-include trick...ALWAYS need one of these to match #ifndef