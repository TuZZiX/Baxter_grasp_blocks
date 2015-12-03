

#ifndef GRIPPER_MOVE_H_
#define GRIPPER_MOVE_H_

#include<ros/ros.h>



class GripperMove
{
public:
    GripperMove(ros::NodeHandle* nodehandle);
    void gripper_close();
    void gripper_open();

private:
       ros::NodeHandle nh_;
       ros::Publisher position_publisher;
    
};

#endif  
