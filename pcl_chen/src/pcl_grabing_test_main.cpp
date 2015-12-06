#include<ros/ros.h>  //  for ros
#include<math.h>

#include <pcl_chen/pcl_grabing.h> 

#include <Eigen/Eigen>  //  for the Eigen library
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <tf/transform_listener.h>  //  transform listener headers
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "pcl_grabing_test_main");
    ros::NodeHandle nh;
    Pcl_grabing cwru_pcl_utils(&nh);
    ROS_INFO("Instantiation done.");
	

    while(ros::ok())
    {

        cwru_pcl_utils.findTableTop();
        //cwru_pcl_utils.isBlock();
        //cwru_pcl_utils.checkForHand();


        ros::Duration(0.5).sleep();  // sleep for half a second
        ros::spinOnce();
    }
	
}