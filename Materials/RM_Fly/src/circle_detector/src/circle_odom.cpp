#include <ros/ros.h>
#include <iostream>
#include<cmath>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h> 
#include<pcl/filters/extract_indices.h>

#include <eigen3/Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include<geometry_msgs/PoseStamped.h>
 
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include"circle_detector/config.hpp"
#include "circle_detector/circle_detector.hpp"


int main(int argc, char** argv)
{
    ros::init(argc,argv,"circle_odom");
    ros::NodeHandle nh("~");

    Circle_Detector circle_detector(nh);

    while(ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}
