#include <ros/ros.h>
//#include <geometry_msgs/Pose.h>
//#include <cwru_msgs/Path.h>
//#include <arm_planning_lib/arm_planning_lib.h>
//#include <pcl_chen/pcl_grabing.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Eigen> 
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <string>

//enum Color = {RED, WHITE, BLACK, GREEN, BLUE, WOODEN};
//using namespace Eigen;

bool publishToScreen(ros::NodeHandle &nh){
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("robot/xdisplay", 1);
	cv::Mat image = cv::imread("~/hand.jpg", CV_LOAD_IMAGE_COLOR);
	cv::waitKey(30);
	sensor_msgs::ImagePtr msg;
	msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
	pub.publish(msg);
	ros::spinOnce();
	return true;
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "final_project");
	ros::NodeHandle nh; 
	publishToScreen(nh);
	ros::spinOnce();
	return 0;
}
/*
int main(int argc, char argv**){
	ros::init(argc, argv, "final_project");
	ros::NodeHandle nh; 
	Pcl_grabing pcl(nh);
	ArmPlanningInterface planner(nh);
	//connection to the robot
	
	
	ROS_INFO("Moving arms to default position.\n");
	planner.moveArmsBack();
	bool table = false;
	
	ROS_INFO("Searching for table top.\n");
	while(!table){
		table = pcl.findTableTop();
		ros::spinOnce();
	}//nothing will happen until the table is found
	ROS_INFO("Table top found. Now waiting for hand signal to begin.\n");

	
	bool searching = true;
	bool handPresent = false;
	bool wasHand = false;
	publishToScreen(nh);
	while(searching){
		if(!handPresent){
			handPresent = pcl.checkForHand();
		}
		if(!handPresent){
			publishToScreen(nh);
		}
		if(handPresent && !wasHand){
			wasHand = pcl.checkForHand();
		}
		if(wasHand){
			bool block = pcl.isBlock();
			if(!block){
				handPresent = false;
				wasHand = false;
				ROS_INFO("Block not found after hand signal. Waiting for next hand signal.\n");
				publishToScreen(nh);
				ros::spinOnce();
				continue;
			}
			
			geometry_msgs::Pose blockPose = pcl.getBlockPose();
			//std_msgs::Float64 width = getWidth();
			//std_msgs::Float64 height = getHeight();
			geometry_msgs::Eigenvector color = pcl.getColor();
			Color c = determineColor(color);
			//cwru_msgs::Pose robotPose = getCurrentPose();
			
			ROS_INFO("Block found on table. Beginning planning.\n");
			
			bool success = planner.planPath(blockPose);
			while(!success){
				success = planner.planPath(blockPose);
				ros::spinOnce();
			}
			
			ROS_INFO("Path to block successfully planned.\n");
			
			success = planner.executePath(30.0);
			
			if(!success){
				ROS_INFO("Path failed to execute. \nRestarting process. \nWaiting for hand signal\n");
				handPresent = false;
				wasHand = false;
				planner.moveArmsBack();
				publishToScreen(nh);
				ros::spinOnce();
				continue;
			}
			
			ROS_INFO("Path to block successfully executed.\nGrabbing block now.\n");
			
			success = planner.grabBlock(width);
			
			while(!success){
				success = grabBlock(width);
			}
			
			String colorName = colorToString(c);
			
			ROS_INFO("Block successfully grabbed. Planning movement for %s block.\n", colorName);
			
			//robotPose = getCurrentPose();
			success = planner.ColorMovement(colorToString(c), blockPose);
			if(!success){
				ROS_INFO("Path failed. \nReleasing gripper and restarting process.\nWaiting for hand signal.\n");
				wasHand = false;
				handEntered = false;
				planner.releaseBlock();
				planner.moveArmsBack();
				publishToScreen(nh);
				ros::spinOnce();
				continue;
			}*/
			/*ROS_INFO("Path for %s block planned. Executing...\n", colorName);
			success = ArmPlanningInterface::executePath(30.0);
			if(!success){
				ROS_INFO("Path failed to execute.\nReleasing gripper and restarting process.\nWaiting for hand signal.\n");
				wasHand = false;
				handEntered = false;
				releaseBlock();
				planner.moveArmsBack();
				publishToScreen(nh);
				ros::spinOnce();
				continue;
			}*//*
			ROS_INFO("Path executed successfully.\nReleasing block and waiting for next hand signal.\n");
			planner.releaseBlock();
			wasHand = false;
			handEntered = false;
		}
		planner.moveArmsBack();
		ros::spinOnce()
	}
}

string colorToString(Color c){
	
	switch(c){
		case RED: return "red";
		case BLUE: return "blue";
		case BLACK: return "black";
		case WHITE: return "white";
		case WOODEN: return "wood";
		case GREEN: return "green";
		default: return "unknown";
	}
}

Color determineColor(Eigen::Vector3d color){
	int r = color[0];
	int g = color[1];
	int b = color[2];



	if(r<25 && g<25 && b<25){
		return BLACK;
	}

	if(r>230 && g>230 && b>230){
		return WHITE;
	}

	if(r>230){
		return RED;
	}

	if(b>230){
		return BLUE;
	}

	if(g>230){
		return GREEN;
	}

	return WOODEN;
}

/*bool planColorPath(Color c, cwru_msgs::Pose curPose){
	switch(c){
		case RED: return planRedMovement(curPose);
		case BLUE: return planBlueMovement(curPose);
		case BLACK: return planBlackMovement(curPose);
		case WHITE: return planWhiteMovement(curPose);
		case WOODEN: return planWoodMovement(curPose);
		case GREEN: return planGreenMovement(curPose);
		default: return false;
	}
}*/



