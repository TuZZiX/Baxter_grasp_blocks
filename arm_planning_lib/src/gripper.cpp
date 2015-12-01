ros::NodeHandle n;
ros::Publisher red_publisher = n.advertise<std_msgs::Int16>("dynamixel_motor1_cmd", 1);
ros::Publisher blue_publisher = n.advertise<std_msgs::Int16>("/dynamixel_motor1_cmd", 1);
ros::Publisher white_publisher = n.advertise<std_msgs::Int16>("dynamixel_motor1_cmd", 1);
ros::Publisher black_publisher = n.advertise<std_msgs::Int16>("dynamixel_motor1_cmd", 1);
ros::Publisher green_publisher = n.advertise<std_msgs::Int16>("dynamixel_motor1_cmd", 1);
ros::Publisher wood_publisher = n.advertise<std_msgs::Int16>("dynamixel_motor1_cmd", 1);
   std_msgs::Int16 _red;
   std_msgs::Int16 _blue;
   std_msgs::Int16 _white;
   std_msgs::Int16 _black;
   std_msgs::Int16 _green;
   std_msgs::Int16 _wood;

/////////////////////////////////////////////////////////////////////
_red.data=a;
_blue.data=b;
_white.data=c;
_black.data=d;
_green.data=e;
_wood.data=f;


red_publisher.publish(_a);
blue_publisher.publish(_b);
white_publisher.publish(_c);
black_publisher.publish(_d);
green_publisher.publish(_e);
wood_publisher.publish(_f);

