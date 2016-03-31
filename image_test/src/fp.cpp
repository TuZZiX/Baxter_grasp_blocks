#include <ros/ros.h>
#include <ros/package.h> 
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Eigen> 
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <string>

using namespace std;
//enum Color = {RED, WHITE, BLACK, GREEN, BLUE, WOODEN};
//using namespace Eigen;

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
/*
void publishToScreen2(ros::NodeHandle &nh){
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("robot/xdisplay", 1);
	ROS_INFO("22222");
	cv::Mat image = cv::imread("baxterworking.png", CV_LOAD_IMAGE_COLOR);
	cv::waitKey(30);
	ROS_INFO("33333");
	sensor_msgs::ImagePtr msg;
	msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
	ROS_INFO("44444");
	pub.publish(*msg);
	ROS_INFO("55555");
	ros::Duration(3.0).sleep();
	ros::spinOnce();
}

void publishToScreen3(ros::NodeHandle &nh){
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("robot/xdisplay", 10);
	ROS_INFO("22222");
	cv::Mat image = cv::imread("/home/user/researchsdk.png", CV_LOAD_IMAGE_COLOR);
	cv::waitKey(30);
	ROS_INFO("33333");
	sensor_msgs::ImagePtr msg;
	msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
	ROS_INFO("44444");
	pub.publish(*msg);
	ROS_INFO("55555");
	ros::Duration(3.0).sleep();
	ros::spinOnce();
}*/


int main(int argc, char** argv) {
	ros::init(argc, argv, "final_project");
	ros::NodeHandle nh; 
	ros::Rate rate(2);
	while( ros::ok() )
    {
    	publishToScreen(nh,"test.jpg");

//    	ROS_INFO("-------------------------------------------------------------");

//    	ROS_INFO("11111");
//    	publishToScreen2(nh);

//    	ROS_INFO("-------------------------------------------------------------");

//    	ROS_INFO("11111");
//    	publishToScreen3(nh);
        rate.sleep();
        ros::spinOnce();
    }
	return 0;
}