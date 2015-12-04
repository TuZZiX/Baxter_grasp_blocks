#include <arm_planning_lib/arm_planning_lib.h>


int main(int argc, char** argv) {
    ros::init(argc, argv, "arm_planning_lib_test"); //node name
    ros::NodeHandle nh;
    ArmPlanningInterface interface(&nh);
    ROS_INFO("setting poses");
    int count = 0;
    Vector7d my_angle;
    geometry_msgs::PoseStamped my_pose;
    my_angle << -0.18503156094537968, 0.44527797626583865, 0.36021593694382065, 1.827325037963473, 1.0199002550686511, 1.0137709796128629, 1.2701787563643496;
    string colors[] = {"red", "blue", "white", "black", "green", "wood"};

    my_pose.pose.position.x = 0.446450;
    my_pose.pose.position.y = -0.149133;
    my_pose.pose.position.z = -0.012427;
    my_pose.pose.orientation.x = -0.335958;
    my_pose.pose.orientation.y = 0.941877;
    my_pose.pose.orientation.z = -0.000015;
    my_pose.pose.orientation.w = -0.000002;
    /*
    while (ros::ok()) {
    	ROS_INFO("executing %d", ++count);
    	interface.moveArmsBack();
    	ros::Duration(5.0).sleep();

//   		interface.planPath(my_angle);
//   		interface.executePath();
//   		ros::Duration(5.0).sleep();
        ROS_INFO("executing %d", ++count);
    	interface.planPath(interface.pre_grab_pose);
    	interface.executePath();
    	ros::Duration(5.0).sleep();
        ROS_INFO("executing %d", ++count);
    	interface.planPath(interface.grab_pose);
    	interface.executePath();
    	ros::Duration(5.0).sleep();
        ROS_INFO("executing %d", ++count);
    	interface.takeALook();
    	ros::Duration(5.0).sleep();

		ros::spinOnce();
    }*/
    //ROS_INFO("total %d colors", colors.size());

/*
    bool ret;
    while (ros::ok()) {
        interface.moveArmsBack();
        ros::Duration(5.0).sleep();
        ros::spinOnce();
        for (int i = 0; i < 6; ++i)
        {
            ROS_INFO("executing %s", colors[i].c_str());
            ret = interface.colorMovement((colors[i]),my_pose);
            if(ret == false)
                ROS_INFO("Color not finished");
            ros::Duration(5.0).sleep();
            ros::spinOnce();
        }
    }*/

    interface.planPath(my_pose);
    interface.executePath();
    ros::Duration(5.0).sleep();
    ros::spinOnce();
    return 0;
}