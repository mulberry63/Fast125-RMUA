#ifndef _VISUAL_
#define _VISUAL_
#include<ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <iostream>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


class Visualizer
{
public:

    void vis_orientation (const Eigen::Vector3d &vertex, const Eigen::Vector3d &direction,
                         const ros::Publisher &circle_orientation_vis, int id = 0)
    {
        visualization_msgs::Marker ProjectedPointsMaker;
        ProjectedPointsMaker.id = id;
        ProjectedPointsMaker.type = visualization_msgs::Marker::SPHERE_LIST;
        ProjectedPointsMaker.header.stamp = ros::Time::now();
        ProjectedPointsMaker.header.frame_id = "world";
        ProjectedPointsMaker.action = visualization_msgs::Marker::ADD;
        ProjectedPointsMaker.ns = "projectedpoints";
        ProjectedPointsMaker.pose.orientation.w = 1.0;
        ProjectedPointsMaker.color.r = 1.00;
        ProjectedPointsMaker.color.g = 0.00;
        ProjectedPointsMaker.color.b = 0.00;
        ProjectedPointsMaker.color.a = 1.00;
        ProjectedPointsMaker.scale.x = 0.20 ;
        ProjectedPointsMaker.scale.y = 0.20 ;
        ProjectedPointsMaker.scale.z = 0.20 ;

        for (size_t i = 0 ; i < 100;i++)
        {
            geometry_msgs::Point point;
            point.x = (vertex + direction * 0.02 * i)(0);
            point.y = (vertex + direction * 0.02 * i)(1);
            point.z = (vertex + direction * 0.02 * i)(2);
            ProjectedPointsMaker.points.push_back(point);
        }
        circle_orientation_vis.publish(ProjectedPointsMaker);
    }

    void vis_pointcloud (const std::vector<Eigen::Vector3d> &points,const ros::Publisher &vis_pts_pub)
    {
        pcl::PointCloud<pcl::PointXYZ> pt_for_vis;
        Eigen::Vector3d circle_pos(0,0,0);
        for (size_t i = 0 ; i < points.size() ;i++)
        {
            pcl::PointXYZ ptt;
            ptt.x = points[i](0);
            ptt.y = points[i](1);
            ptt.z = points[i](2);
            pt_for_vis.points.push_back(ptt);
        }

        sensor_msgs::PointCloud2 circle_pt_vis_msg_;
        pcl::toROSMsg(pt_for_vis,circle_pt_vis_msg_);
        circle_pt_vis_msg_.header.frame_id = "world";
        vis_pts_pub.publish(circle_pt_vis_msg_);
    }


    // wxx
    void vis_pointcloud_intensity (const std::vector<Eigen::Vector4d> &points,const ros::Publisher &vis_pts_pub)
    {
        pcl::PointCloud<pcl::PointXYZI> pt_for_vis;
        Eigen::Vector3d circle_pos(0,0,0);
        for (size_t i = 0 ; i < points.size() ;i++)
        {
            pcl::PointXYZI ptt;
            ptt.x = points[i](0);
            ptt.y = points[i](1);
            ptt.z = points[i](2);
            ptt.intensity = points[i](3);
            pt_for_vis.points.push_back(ptt);
        }

        sensor_msgs::PointCloud2 circle_pt_vis_msg_;
        pcl::toROSMsg(pt_for_vis,circle_pt_vis_msg_);
        circle_pt_vis_msg_.header.frame_id = "world";
        vis_pts_pub.publish(circle_pt_vis_msg_);
    }

    void vis_pointcloud (const pcl::PointCloud<pcl::PointXYZ> &points,const ros::Publisher &vis_pts_pub)
    {
        sensor_msgs::PointCloud2 circle_pt_vis_msg_;
        pcl::toROSMsg(points,circle_pt_vis_msg_);
        circle_pt_vis_msg_.header.frame_id = "world";
        vis_pts_pub.publish(circle_pt_vis_msg_);
    }

};


#endif

