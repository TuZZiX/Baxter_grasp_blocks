// example_action_client: 
// wsn, October, 2014

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include<gripper/dgripperAction.h>
#include <iostream>
#include <string>
#include <std_msgs/Int6.h>
#include <std_msgs/Bool.h>
// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server
void doneCb(const actionlib::SimpleClientGoalState& state,
        const gripper_test::grippertestResultConstPtr& result) {
    ROS_INFO(" succeed");
   
}

int main(int argc, char** argv) {
        using namespace std;
        ros::init(argc, argv, "gripper_test_client_node"); // name this node 
        bool cmd;
        gripper_test::grippertestGoal goal; 
        cout <<"please input cmd";
        cin >>cmd;
        actionlib::SimpleActionClient<gripper_test::grippertestAction> action_client("gripper_test_client", true);
        ROS_INFO("waiting for server: ");
        bool server_exists = action_client.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
        // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running
        //bool server_exists = action_client.waitForServer(); //wait forever

        if (!server_exists) {
            ROS_WARN("could not connect to server; halting");
            return 0; // bail out; optionally, could print a warning message and retry
        }
        
       
        ROS_INFO("connected to action server");  // if here, then we connected to the server;

        while(true) {
        // stuff a goal message:
       
       std_msgs::Int16 cmd;
        cmd.data=cmd;
        goal.cmd = cmd.data;

        action_client.sendGoal(goal,&doneCb); 
      
        
        bool finished_before_timeout = action_client.waitForResult(ros::Duration(5.0));
      
        if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result for goal number %d",g_count);
            return 0;
        }
        else {
         
        }
        
        }

    return 0;
}

