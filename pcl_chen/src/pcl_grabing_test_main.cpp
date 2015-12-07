#include<ros/ros.h>  //  for ros
#include<math.h>

#include <pcl_chen/pcl_grabing.h> 

#include <Eigen/Eigen>  //  for the Eigen library
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <tf/transform_listener.h>  //  transform listener headers
#include <tf/transform_broadcaster.h>

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

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "pcl_grabing_test_main");
    ros::NodeHandle nh;
    Pcl_grabing cwru_pcl_utils(&nh);
    ROS_INFO("Instantiation done.");
	bool ret;
    while(!cwru_pcl_utils.findTableTop());
    while(ros::ok())
    {

        
        //ROS_WARN("findTableTop return %s",ret?"successed":"failed");
        ret = cwru_pcl_utils.isBlock();

        ROS_WARN("isBlock return %s",ret?"successed":"failed");
        ROS_WARN("Block Color is  %s",determineColor(cwru_pcl_utils.getColor()).c_str());
        ROS_WARN("get_color2 return is  %s",cwru_pcl_utils.getColor2().c_str());
        //ret = cwru_pcl_utils.checkForHand();
        //SROS_WARN("checkForHand return %s",ret?"successed":"failed");

        ros::Duration(0.5).sleep();  // sleep for half a second
        ros::spinOnce();
    }
	
}
