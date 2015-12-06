#include <ros/ros.h>
#include <ros/package.h> 
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
#include <math.h>
#include <pcl_chen/pcl_grabing.h> 

#include <tf/transform_listener.h>  //  transform listener headers
#include <tf/transform_broadcaster.h>


void publishToScreen(ros::NodeHandle &nh, string path){
//  image_transport::ImageTransport it(nh);
    string pkg_path = ros::package::getPath("overall_executer");
    string append = "/image/";
    string full_path = pkg_path+append+path;
    ROS_INFO("%s",full_path.c_str());
    ros::Publisher pub = nh.advertise<sensor_msgs::Image>("robot/xdisplay", 10, true);
    cv::Mat image = cv::imread(full_path, CV_LOAD_IMAGE_COLOR);
    cv::waitKey(100);
    sensor_msgs::ImagePtr msg;
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    pub.publish(*msg);
    ros::Duration(5.0).sleep();
    ros::spinOnce();
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "pcl_grabing_test_main");
    ros::NodeHandle nh;
    publishToScreen(nh, "init.jpg");
    Pcl_grabing pcl(&nh);
    ROS_INFO("Instantiation done.");
	bool ret;
    bool table = false;
    bool handPresent = false;
    bool wasHand = false;
    int pic = 0;


    ROS_INFO("Searching for table top.\n");
    while(!table){
        table = pcl.findTableTop();
        ros::spinOnce();
    }//nothing will happen until the table is found
    ROS_INFO("Table top found. Now waiting for hand signal to begin.\n");
    publishToScreen(nh, "wait_hand_sig.jpg");
    while(ros::ok())
    {
        if(!handPresent){
            publishToScreen(nh, "wait_hand_sig.jpg");
            
            handPresent = pcl.checkForHand();
        }
        if(handPresent && !wasHand){
            publishToScreen(nh, "found_hand_sig.jpg");
            wasHand = pcl.checkForHand();

        }
        if(wasHand){
            publishToScreen(nh, "working.jpg");
            ros::Duration(10.0).sleep();  // sleep for half a second
            ros::spinOnce();
            /*
            bool block = pcl.isBlock();
            if(!block){
                handPresent = false;
                wasHand = false;
                ROS_INFO("Block not found after hand signal. Waiting for next hand signal.\n");
                ros::spinOnce();
                continue;
            }*/
        }
        wasHand = false;
        handPresent = false;
        //ros::Duration(0.5).sleep();
        ros::spinOnce();
        // = pcl.findTableTop();
        //ROS_WARN("findTableTop return %s",ret?"successed":"failed");
        //ret = pcl.isBlock();
        //ROS_WARN("isBlock return %s",ret?"successed":"failed");
        //ret = pcl.checkForHand();
        //ROS_WARN("checkForHand return %s",ret?"successed":"failed");

        //ros::Duration(0.5).sleep();  // sleep for half a second
        //ros::spinOnce();
    }
	
}