#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <cwru_msgs/Path.h>
#include <arm_planning_lib/arm_planning_lib.h>
#include <pcl_chen/pcl_grabing.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Eigen> 
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <string>

enum ColorEnum {RED, WHITE, BLACK, GREEN, BLUE, WOODEN};
typedef enum ColorEnum Color;
using namespace Eigen;

string determineColor(Eigen::Vector3d color){
	int r = color[0];
	int g = color[1];
	int b = color[2];



	if(r<25 && g<25 && b<25){
		return "black";
	}

	if(r>230 && g>230 && b>230){
		return "white";
	}

	if(r>230){
		return "red";
	}

	if(b>230){
		return "blue";
	}

	if(g>230){
		return "green";
	}

	return "wood";
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

void publishToScreen(ros::NodeHandle &nh){
	/*
	ros::Publisher pub = nh.advertise<sensor_msgs::Image>("robot/xdisplay", 10, true);
	cv::Mat image = cv::imread("test.jpg", CV_LOAD_IMAGE_COLOR);
	cv::waitKey(30);
	sensor_msgs::ImagePtr msg;
	msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
	pub.publish(*msg);*/
	ros::Duration(3.0).sleep();
	ros::spinOnce();
}

int main(int argc, char** argv){
	ros::init(argc, argv, "final_project");
	ros::NodeHandle nh; 
	Pcl_grabing pcl(&nh);
	ArmPlanningInterface planner(&nh);
	//connection to the robot
	geometry_msgs::Pose blockPose;
	Vector3f plane_normal, major_axis, centroid;
	
	ROS_INFO("Moving arms to default position.\n");
	planner.moveArmsBack();
	bool table = false;
	
	ROS_INFO("Searching for table top.\n");
	while(!table){
		table = pcl.findTableTop();
		ros::spinOnce();
		ROS_INFO("check point 1");
	}//nothing will happen until the table is found
	ROS_INFO("Table top found. Now waiting for hand signal to begin.\n");

	
	bool searching = true;
	bool handPresent = false;
	bool wasHand = false;
	publishToScreen(nh);
	ROS_INFO("check point 2");
	while(searching){
		/*
		if(!handPresent){
			handPresent = pcl.checkForHand();
			ROS_INFO("check point 3");
		}
		if(!handPresent){
			publishToScreen(nh);
			ROS_INFO("check point 4");
		}
		if(handPresent && !wasHand){
			wasHand = pcl.checkForHand();
			ROS_INFO("check point 5");
		}*/
//		if(wasHand){
		if(true){
			bool block = pcl.isBlock();
			if(!block){
				handPresent = false;
				wasHand = false;
				ROS_INFO("Block not found after hand signal. Waiting for next hand signal.\n");
				publishToScreen(nh);
				ros::spinOnce();
				continue;
				ROS_INFO("check point 6");
			}
			
			//blockPose = pcl.getBlockPose();
			pcl.getBlockVector(plane_normal, major_axis, centroid);
			blockPose = planner.convToPose(plane_normal, major_axis, centroid);

			ROS_INFO("check point 7");
			//std_msgs::Float64 width = getWidth();
			//std_msgs::Float64 height = getHeight();
			Eigen::Vector3d color = pcl.getColor();
			ROS_INFO("check point 8");
			string c = determineColor(color);
			ROS_INFO("check point 9");
			//cwru_msgs::Pose robotPose = getCurrentPose();
			
			ROS_INFO("%s Block found on table. Beginning planning.\n",c.c_str());
			bool success;
			/*
			bool success = planner.planPath(blockPose);
			ROS_INFO("check point 10");
			while(!success){
				success = planner.planPath(blockPose);
				ROS_INFO("check point 11");
				ros::spinOnce();
			}
			
			ROS_INFO("Path to block successfully planned.\n");
			
			success = planner.executePath();
			ROS_INFO("check point 12");
			if(!success){
				ROS_INFO("Path failed to execute. \nRestarting process. \nWaiting for hand signal\n");
				handPresent = false;
				wasHand = false;
				planner.moveArmsBack();
				publishToScreen(nh);
				ROS_INFO("check point 13");
				ros::spinOnce();
				continue;
			}
			
			ROS_INFO("Path to block successfully executed.\nGrabbing block now.\n");*/
			/*
			success = planner.grabBlock(width);
			
			while(!success){
				success = grabBlock(width);
			}
			*/

			ROS_INFO("POSE: X = %f, Y = %f, Z = %f, orientation: X = %f, Y = %f, Z = %f, W = %f", blockPose.position.x, blockPose.position.y, blockPose.position.z, blockPose.orientation.x, blockPose.orientation.y, blockPose.orientation.z, blockPose.orientation.w);
			ROS_INFO("check point 14");
			//ROS_INFO("Block successfully grabbed. Planning movement for block.\n");
			
			//robotPose = getCurrentPose();
			success = planner.colorMovement(c, blockPose);
			
			if(!success){
				ROS_INFO("Path failed. \nReleasing gripper and restarting process.\nWaiting for hand signal.\n");
				wasHand = false;
				handPresent = false;
				//planner.releaseBlock();
				planner.moveArmsBack();
				publishToScreen(nh);
				ROS_INFO("check point 15");
				ros::spinOnce();
				continue;
			} else {
				ROS_INFO("Path executed successfully.\nReleasing block and waiting for next hand signal.\n");
			}
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
			}*/
			//planner.releaseBlock();
			wasHand = false;
			handPresent = false;
			ros::spinOnce();
		}
		planner.moveArmsBack();
		ros::spinOnce();
	}
}




