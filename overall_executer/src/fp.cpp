#define MOVEIT

#include <ros/ros.h>
#include <ros/package.h> 
#include <geometry_msgs/Pose.h>
#include <cwru_msgs/Path.h>
#ifdef MOVEIT
#include <arm_planning_lib/arm_planning_lib_moveit.h>
#else
#include <arm_planning_lib/arm_planning_lib.h>
#endif
#include <pcl_chen/pcl_grabing.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Eigen> 
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <string>


using namespace Eigen;

string determineColor(Eigen::Vector3d color){
    int r = color[0];
    int g = color[1];
    int b = color[2];



    if(r<150 && g<150 && b<150){
        return "black";
    }

    if(r>190 && g>190 && b>190){
        if (g-b > 10)
        {
            return "wood";
        } else {
            return "white";
        }
    }

    if(r>160 && g<150 && b<170){
        return "red";
    }

    if(b>160 && r>80&& r<180 && g>80 && g<190){
        return "blue";
    }

    if(g>160 && r>80 && r<200 && b<160){
        return "green";
    }

    return "wood";
}

void publishToScreen(ros::NodeHandle &nh, string path){
//	image_transport::ImageTransport it(nh);
	string pkg_path = ros::package::getPath("overall_executer");
	string append = "/image/";
	string full_path = pkg_path+append+path;
	ROS_INFO("%s",full_path.c_str());
	ros::Publisher pub = nh.advertise<sensor_msgs::Image>("robot/xdisplay", 10, true);
	cv::Mat image = cv::imread(full_path, CV_LOAD_IMAGE_COLOR);
	cv::waitKey(30);
	sensor_msgs::ImagePtr msg;
	msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
	pub.publish(*msg);
	ros::Duration(1.0).sleep();
	ros::spinOnce();
}

void printColorToScreen(ros::Nodehandle &nh, string c){
	switch(c){
		case "red": publishToScreen(nh, "red_found.jpg"); break;
		case "white": publishToScreen(nh, "white_found.jpg"); break;
		case "blue": publishToScreen(nh, "blue_found.jpg"); break;
		case "green": publishToScreen(nh, "green_found.jpg"); break;
		case "wood": publishToScreen(nh, "wooden_found.jpg"); break;
		case "black": publishToScreen(nh, "black_found.jpg"); break;
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "overall_executer");
	ros::NodeHandle nh; 
	publishToScreen(nh, "init.jpg");
	Pcl_grabing pcl(&nh);
	#ifdef MOVEIT
	MoveitPlanningInterface planner(&nh);
	#else
	ArmPlanningInterface planner(&nh);
#	endif
	
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
	}//nothing will happen until the table is found
	ROS_INFO("Table top found. Now waiting for hand signal to begin.\n");

	
	bool searching = true;
	bool handPresent = false;
	bool wasHand = false;
	publishToScreen(nh, "wait_hand_sig.jpg");
	while(searching){
		
		if(!handPresent){
			handPresent = pcl.checkForHand();
			ROS_INFO("check point 3");
		}
		if(!handPresent){
			publishToScreen(nh, "test.jpg");
			ROS_INFO("check point 4");
		}
		if(handPresent && !wasHand){
			wasHand = !(pcl.checkForHand());
			ROS_INFO("check point 5");
		}
		if(wasHand){
		//if(true){
			bool block = pcl.isBlock();
			if(!block){
				handPresent = false;
				wasHand = false;
				ROS_INFO("Block not found after hand signal. Waiting for next hand signal.\n");
				publishToScreen(nh, "working.jpg");
				ros::spinOnce();
				continue;
			}
			
			//blockPose = pcl.getBlockPose();
			pcl.getBlockVector(plane_normal, major_axis, centroid);
			blockPose = planner.convToPose(plane_normal, major_axis, centroid);

			//std_msgs::Float64 width = getWidth();
			//std_msgs::Float64 height = getHeight();
			Eigen::Vector3d color = pcl.getColor();
			string c = determineColor(color);
			//cwru_msgs::Pose robotPose = getCurrentPose();
			
			ROS_INFO("%s Block found on table. Beginning planning.\n",c.c_str());
			printColorToScreen(nh, c);
			bool success = false;
			/*
			bool success = planner.planPath(blockPose);
			while(!success){
				success = planner.planPath(blockPose);
				ros::spinOnce();
			}
			
			ROS_INFO("Path to block successfully planned.\n");
			
			success = planner.executePath();
			if(!success){
				ROS_INFO("Path failed to execute. \nRestarting process. \nWaiting for hand signal\n");
				handPresent = false;
				wasHand = false;
				planner.moveArmsBack();
				publishToScreen(nh, "test.jpg");
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
			//ROS_INFO("Block successfully grabbed. Planning movement for block.\n");
			
			//robotPose = getCurrentPose();
			success = planner.colorMovement(c, blockPose);
			
			if(!success){
				ROS_INFO("Path failed. \nReleasing gripper and restarting process.\nWaiting for hand signal.\n");
				wasHand = false;
				handPresent = false;
				//planner.releaseBlock();
				planner.moveArmsBack();
				publishToScreen(nh, "working.jpg");
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
				publishToScreen(nh, "test.jpg");
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




