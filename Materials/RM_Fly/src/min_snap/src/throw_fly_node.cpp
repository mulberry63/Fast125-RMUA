#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/AttitudeTarget.h>
#include "quadrotor_msgs/PositionCommand.h"

ros::Subscriber attitude_sub, debug_odom_sub, triger_sub, cmd_sub;
ros::Publisher attitude_pub, debug_odom_pub, attitude_pub_debug;
bool pub_debug_odom = false;
bool plan_success = false;

void attitude_cb(const mavros_msgs::AttitudeTarget::ConstPtr &msg)
{
    if (plan_success)
        return;

    attitude_pub.publish(msg);
    attitude_pub_debug.publish(msg);
}

void odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    if (pub_debug_odom)
    {
        debug_odom_pub.publish(msg);
    }
}

void triger_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    pub_debug_odom = true;
}

void cmd_cb(const quadrotor_msgs::PositionCommand::ConstPtr &msg)
{
    plan_success = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "throw_fly_node");
    ros::NodeHandle nh("~");
    ros::Rate rate(1000);

    attitude_sub = nh.subscribe("/sub_attitude", 100, attitude_cb, ros::TransportHints().tcpNoDelay());
    attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/pub_attitude", 10);
    attitude_pub_debug = nh.advertise<mavros_msgs::AttitudeTarget>("/pub_attitude_debug", 10);
    cmd_sub = nh.subscribe("/cmd", 100, cmd_cb, ros::TransportHints().tcpNoDelay());

    // for debug
    debug_odom_sub = nh.subscribe("/debug_sub_odom", 100, odom_cb, ros::TransportHints().tcpNoDelay());
    debug_odom_pub = nh.advertise<nav_msgs::Odometry>("/debug_pub_odom", 10);
    triger_sub = nh.subscribe("/triger", 100, triger_cb, ros::TransportHints().tcpNoDelay());

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}