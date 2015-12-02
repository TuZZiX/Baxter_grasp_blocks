// pcl_grabing.cpp 
// Zhiang Chen, Nov 2015
// for class Pcl_grabing in pcl_grabing.h

#include <pcl_chen/pcl_grabing.h>
#include <ros/ros.h>
#include <math.h>
#include <Eigen/Eigen>  //  for the Eigen library
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <tf/transform_listener.h>  //  transform listener headers
#include <tf/transform_broadcaster.h>

Pcl_grabing::Pcl_grabing(ros::NodeHandle* nodehandle) : pcl_wsn(nodehandle), display_ptr_(new PointCloud<pcl::PointXYZ>),
pclKinect_clr_ptr_(new PointCloud<pcl::PointXYZRGB>), transformed_pclKinect_clr_ptr_(new PointCloud<pcl::PointXYZRGB>) {
    TableHeight = roughHeight;
    TableColor(0) = roughColor_R;
    TableColor(1) = roughColor_G;
    TableColor(2) = roughColor_B;
    points_publisher = nodehandle->advertise<sensor_msgs::PointCloud2>("display_points", 1, true);
    pointcloud_subscriber_ = nodehandle->subscribe("/kinect/depth/points", 1, &Pcl_grabing::KinectCameraCB, this);
    got_kinect_cloud_ = false;
    
}

void Pcl_grabing::update_kinect_points() 
{
	reset_got_kinect_cloud(); // turn on the camera
    ROS_INFO("begin to update kinect points");
    while (!got_kinect_cloud()) {
        ROS_INFO("did not receive pointcloud");
        ros::spinOnce();
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("got a pointcloud.");


    tf::StampedTransform tf_sensor_frame_to_torso_frame; //use this to transform sensor frame to torso frame
    tf::TransformListener tf_listener; //start a transform listener

    // let's warm up the tf_listener, to make sure it get's all the transforms it needs to avoid crashing:
    bool tferr = true;
    ROS_INFO("waiting for tf between kinect_pc_frame and torso...");
    while (tferr) {
        tferr = false;
        try {
            // The direction of the transform returned will be from the target_frame to the source_frame. 
            // Which if applied to data, will transform data in the source_frame into the target_frame. See tf/CoordinateFrameConventions#Transform_Direction
            tf_listener.lookupTransform("torso", "kinect_pc_frame", ros::Time(0), tf_sensor_frame_to_torso_frame);
        } catch (tf::TransformException &exception) {
            ROS_ERROR("%s", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }
    ROS_INFO("tf is good"); //  tf-listener found a complete chain from sensor to world; ready to roll

    Eigen::Affine3f A_sensor_wrt_torso;
    A_sensor_wrt_torso = pcl_wsn.transformTFToEigen(tf_sensor_frame_to_torso_frame);

    //ROS_INFO_STREAM("Affine"<<endl<<A_sensor_wrt_torso.matrix()); // check Affine


    transform_clr_kinect_cloud(A_sensor_wrt_torso);
    ROS_INFO("transformed color kinect points");
    set_got_kinect_cloud(); // turn off the camera
}

void Pcl_grabing::transform_clr_kinect_cloud(Eigen::Affine3f A) {
    transformed_pclKinect_clr_ptr_->header = pclKinect_clr_ptr_->header;
    transformed_pclKinect_clr_ptr_->is_dense = pclKinect_clr_ptr_->is_dense;
    transformed_pclKinect_clr_ptr_->width = pclKinect_clr_ptr_->width;
    transformed_pclKinect_clr_ptr_->height = pclKinect_clr_ptr_->height;
    int npts = pclKinect_clr_ptr_->points.size();
    cout << "transforming npts = " << npts << endl;
    transformed_pclKinect_clr_ptr_->points.resize(npts);

    //somewhat odd notation: getVector3fMap() reading OR WRITING points from/to a pointcloud, with conversions to/from Eigen
    for (int i = 0; i < npts; ++i) {
        transformed_pclKinect_clr_ptr_->points[i].getVector3fMap() = A * pclKinect_clr_ptr_->points[i].getVector3fMap();
        transformed_pclKinect_clr_ptr_->points[i].r = pclKinect_clr_ptr_->points[i].r;
        transformed_pclKinect_clr_ptr_->points[i].g = pclKinect_clr_ptr_->points[i].g;
        transformed_pclKinect_clr_ptr_->points[i].b = pclKinect_clr_ptr_->points[i].b;
    }
}

bool Pcl_grabing::findTableTop() {
    update_kinect_points();
    int npts = transformed_pclKinect_clr_ptr_->points.size();
    vector<int> index;
    Eigen::Vector3f pt;
    vector<double> color_err_RGB;
    double color_err;
    color_err = 255;
    color_err_RGB.resize(3);
    index.clear();
    ROS_INFO("begin to find the table top");
    for (int i = 0; i < npts; i++) 
    {
        pt = transformed_pclKinect_clr_ptr_->points[i].getVector3fMap();
        color_err_RGB[0] = abs(roughColor_R - transformed_pclKinect_clr_ptr_->points[i].r);
        color_err_RGB[1] = abs(roughColor_G - transformed_pclKinect_clr_ptr_->points[i].g);
        color_err_RGB[2] = abs(roughColor_B - transformed_pclKinect_clr_ptr_->points[i].b);
        color_err = color_err_RGB[0] + color_err_RGB[1] + color_err_RGB[2];
        if (abs(pt[2] - roughHeight) < HeightRange) 
        {
            if (color_err < ColorRange) 
            {
                if(pt[0]>Table_X_Min && pt[0]<Table_X_Max)
                {
                    if(pt[0]>Table_Y_Min && pt[0]<Table_Y_Max)
                    {
                        index.push_back(i);    
                    }
                }
            }
        }
    }
    if (index.size() < 20) 
    {
        ROS_INFO("failed to find table top.");
        return 0;
    }
    int n_display = index.size();
    ROS_INFO("found out %d points on table.", n_display);


    display_ptr_->header = transformed_pclKinect_clr_ptr_->header;
    display_ptr_->is_dense = transformed_pclKinect_clr_ptr_->is_dense;
    display_ptr_->width = n_display; 
    display_ptr_->height = transformed_pclKinect_clr_ptr_->height;
    display_ptr_->points.resize(n_display);
    for (int i = 0; i < n_display; i++) {
        display_ptr_->points[i].getVector3fMap() = transformed_pclKinect_clr_ptr_->points[index[i]].getVector3fMap();
    }
    ROS_INFO("display_point conversed.");

    display_points(*display_ptr_); 
    
    TableCentroid =pcl_wsn.compute_centroid(display_ptr_);
    TableHeight = TableCentroid(2);

    ROS_INFO_STREAM("Table Centroid:"<<TableCentroid.transpose());
    ROS_INFO_STREAM("Table Height"<<TableHeight);
    
    return true;
}

bool Pcl_grabing::isBlock()
{
    update_kinect_points();
    int npts = transformed_pclKinect_clr_ptr_->points.size();
    Eigen::Vector3f pt;
    Eigen::Vector3f dist;
    vector<int> index;
    index.clear();
    double distance = 1;
    BlockColor<<0,0,0;
    for (int i = 0; i < npts; i++) 
    {
        pt = transformed_pclKinect_clr_ptr_->points[i].getVector3fMap();
        dist = pt - TableCentroid;
        dist[2]=0;
        distance = dist.norm();
        if(distance < TableRadius)
            if(pt[2]>(TableHeight+0.003) && pt[2]<BlockMaxHeight)
            {
                index.push_back(i);
                BlockColor(0)+=transformed_pclKinect_clr_ptr_->points[i].r;
                BlockColor(1)+=transformed_pclKinect_clr_ptr_->points[i].g;
                BlockColor(2)+=transformed_pclKinect_clr_ptr_->points[i].b;
            }
    }
    int n_block_points = index.size();
    if(n_block_points<10)
    {
        ROS_INFO("There is no block on the stool.");
        return 0;
    }
    ROS_INFO("There is a block with %d points", n_block_points);
    BlockColor/=n_block_points;
    ROS_INFO_STREAM("The block color:"<<BlockColor.transpose());
    
    display_ptr_->header = transformed_pclKinect_clr_ptr_->header;
    display_ptr_->is_dense = transformed_pclKinect_clr_ptr_->is_dense;
    display_ptr_->width = n_block_points; //transformed_pclKinect_clr_ptr_->width;
    display_ptr_->height = transformed_pclKinect_clr_ptr_->height;
    display_ptr_->points.resize(n_block_points);
    for (int i = 0; i < n_block_points; i++) 
    {
        display_ptr_->points[i].getVector3fMap() = transformed_pclKinect_clr_ptr_->points[index[i]].getVector3fMap();
    }
    display_points(*display_ptr_);
    
    Eigen::Vector3f BlockCentroid;
    BlockCentroid =pcl_wsn.compute_centroid(display_ptr_);
    ROS_INFO_STREAM("The centroid of the block:"<<BlockCentroid.transpose());


    vector<int> block_index;
    block_index.clear();
    for (int i = 0; i < n_block_points; i++) 
    {
        pt=display_ptr_->points[i].getVector3fMap();
        dist = pt - BlockCentroid;
        dist[2]=0;
        distance = dist.norm();
        if(distance < BlockTopRadius)
        {
            block_index.push_back(i);
        }
    }
    int n_block_top = block_index.size();
    ROS_INFO("There are %d points around the block's top center",n_block_top);
    pcl::PointCloud<pcl::PointXYZ>::Ptr block_ptr_(new PointCloud<pcl::PointXYZ>);
    block_ptr_->header=display_ptr_->header;
    block_ptr_->is_dense=display_ptr_->is_dense;
    block_ptr_->width=n_block_top;
    block_ptr_->height=display_ptr_->height;
    block_ptr_->points.resize(n_block_top);   
    for (int i = 0; i < n_block_top; i++) 
    {
        block_ptr_->points[i].getVector3fMap()=display_ptr_->points[block_index[i]].getVector3fMap();
    }

    BlockTopCentroid = pcl_wsn.compute_centroid(block_ptr_);
    ROS_INFO_STREAM("The centroid of the block's top:"<<BlockTopCentroid.transpose());
    //display_points(*block_ptr_);


    block_index.clear();
    for(int i = 0; i < n_block_points; i++)
    {
        pt=display_ptr_->points[i].getVector3fMap();
        if(abs(pt[2]-BlockTopCentroid[2])<0.002)
        {
            block_index.push_back(i);
        }
    }
    n_block_top = block_index.size();
    block_ptr_->header=display_ptr_->header;
    block_ptr_->is_dense=display_ptr_->is_dense;
    block_ptr_->width=n_block_top;
    block_ptr_->height=display_ptr_->height;
    block_ptr_->points.resize(n_block_top);   
    for (int i = 0; i < n_block_top; i++) 
    {
        block_ptr_->points[i].getVector3fMap()=display_ptr_->points[block_index[i]].getVector3fMap();
    }
    
    double block_dist;
    pcl_wsn.fit_points_to_plane(block_ptr_,Block_Normal,block_dist);
    Block_Major = pcl_wsn.get_major_axis();
    ROS_INFO_STREAM("The major vector of the block's top:"<<Block_Major.transpose());
    //display_points(*block_ptr_);

    return true;
}


Eigen::Vector3d Pcl_grabing::getColor()
{
    return BlockColor;
}

int Pcl_grabing::getColor2()
{
    int n=0;
    Eigen::Vector3d red,blue,black,green,b,d;
    b=BlockColor/BlockColor.norm();

    red<<188,54,100; // red
    red=red/red.norm();
    d=red-b;
    if(d.norm()<Eps)
        return n=1; // red

    green<<149,212,10;
    green/=green.norm();
    d=green-b;
    if(d.norm()<Eps)
        return n=2;// green

    blue<<95,212,10;
    blue/=blue.norm();
    d=blue-b;
    if(d.norm()<Eps)
        return n=3; // blue

    black<<75,80,88;
    black/=black.norm();
    d=black-b;
    if (d.norm()<Eps)
        return n=4; // black

    return n=0; // error
}

geometry_msgs::Pose Pcl_grabing::getBlockPose()
{
    geometry_msgs::Pose pose;
    pose.position.x = BlockTopCentroid[0];
    pose.position.y = BlockTopCentroid[1];
    pose.position.z = BlockTopCentroid[2];
    
    Eigen::Vector3f x_axis(1,0,0);
    double sn_theta = Block_Major.dot(x_axis);
    double theta = acos(sn_theta);
    
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = sin(theta/2);
    pose.orientation.w = cos(theta/2);
    
    return pose;
}

bool Pcl_grabing::checkForHand()
{
    update_kinect_points();
    int npts = transformed_pclKinect_clr_ptr_->points.size();
    Eigen::Vector3f pt;
    Eigen::Vector3f dist;
    double distance = 1;
    vector<int> index;
    index.clear();
    for (int i = 0; i < npts; i++) 
    {
        pt = transformed_pclKinect_clr_ptr_->points[i].getVector3fMap();
        dist = pt - TableCentroid;
        dist[2]=0;
        distance = dist.norm();
        if(distance < TableRadius)
        {
            if(pt[2]>(TableHeight+HandMinHeight))
            {
                index.push_back(i);
            }
        }
    }
    int n_hand_points = index.size();
    if(n_hand_points<20)
    {
        ROS_INFO("there is no hand.");
        return 0;
    }
    ROS_INFO("got hand.");
    return true;
}

void Pcl_grabing::display_points(PointCloud<pcl::PointXYZ> points)
{
	sensor_msgs::PointCloud2 pcl2_display_cloud;
    pcl::toROSMsg(points, pcl2_display_cloud);
    pcl2_display_cloud.header.stamp = ros::Time::now();
    points_publisher.publish(pcl2_display_cloud);
}

void Pcl_grabing::KinectCameraCB(const sensor_msgs::PointCloud2ConstPtr& cloud) 
{
	if (!got_kinect_cloud_) 
	{
	    pcl::fromROSMsg(*cloud,*pclKinect_clr_ptr_);
		got_kinect_cloud_ = true;
	}

}