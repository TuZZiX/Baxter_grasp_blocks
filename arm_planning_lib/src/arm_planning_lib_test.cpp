#include <arm_planning_lib/arm_planning_lib.h>


int main(int argc, char** argv) {
    ros::init(argc, argv, "arm_planning_lib_test"); //node name
    ros::NodeHandle nh;
    ArmPlanningInterface interface(&nh);
    ROS_INFO("setting poses");
    int count = 0;
    Vector7d my_angle;
    my_angle << -0.18503156094537968, 0.44527797626583865, 0.36021593694382065, 1.827325037963473, 1.0199002550686511, 1.0137709796128629, 1.2701787563643496;
    string colors[] = {"red", "blue", "white", "black", "green", "wood"};
/*
pre_grab
pose: 
  position: 
    x: 0.62630899645
    y: -0.00805334156627
    z: 0.206422664913
  orientation: 
    x: 0.629827329351
    y: 0.776484996568
    z: 0.0187988533019
    w: -0.00593198287789

grab
pose: 
  position: 
    x: 0.627811922278
    y: -0.00340972086974
    z: 0.0543679733262
  orientation: 
    x: 0.629826654045
    y: 0.776525376337
    z: 0.0171547560392
    w: -0.0056956215282

drop_left
  position: 
    x: 0.618217876627
    y: -0.373787660953
    z: 0.0347969815439
  orientation: 
    x: 0.629429968147
    y: 0.776821296576
    z: 0.0183006089831
    w: -0.00562815199012

drop_right
  position: 
    x: 0.607757389961
    y: 0.29148022485
    z: 0.0556670907964
  orientation: 
    x: 0.629136911805
    y: 0.777105813518
    z: 0.0147144639623
    w: -0.00876272515292

gripper at table top
  position: 
    x: 0.575521702564
    y: -0.22915244597
    z: 0.0475048224968
  orientation: 
    x: 0.62945411565
    y: 0.776795019467
    z: 0.0183889186523
    w: -0.00623391483899

hand at table top
pose: 
  position: 
    x: 0.578822113329
    y: -0.222901535766
    z: -0.129146973155
  orientation: 
    x: 0.629548500816
    y: 0.776861010357
    z: 0.00458504421153
    w: 0.0116031494482
take a look
pose: 
  position: 
    x: 0.52542198926
    y: -0.19065849761
    z: 0.154626356725
  orientation: 
    x: -0.249821685981
    y: -0.335221027398
    z: -0.504147162678
    w: 0.755679579166

bakc to start
pose: 
  position: 
    x: 0.628566212076
    y: -0.347307547123
    z: 0.350523968221
  orientation: 
    x: 0.630141382934
    y: 0.775598336984
    z: 0.000519772148981
    w: 0.0369971217577
*/
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
    bool ret;
    while (ros::ok()) {
        interface.moveArmsBack();
        ros::Duration(5.0).sleep();
        ros::spinOnce();
        for (int i = 0; i < 6; ++i)
        {
            ROS_INFO("executing %s", colors[i].c_str());
            ret = interface.colorMovement((colors[i]),interface.grab_pose);
            if(ret == false)
                ROS_INFO("Color not finished");
            ros::Duration(5.0).sleep();
            ros::spinOnce();
        }
    }
}