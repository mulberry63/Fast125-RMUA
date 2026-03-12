#include <cmath>
#include <iostream>
#include <Eigen/Eigen>
#include <memory>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include "ros/ros.h"
#include <airsim_ros_pkgs/PoseCmd.h>
#include <airsim_ros_pkgs/AngleRateThrottle.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <uav_utils/utils.h>
ros::Subscriber cmd_sub,airsim_odom_sub;
ros::Publisher odom_pub,airsim_posecmd_pub,odom_for_vio_pub,airsim_bodyratecmd_pub;
void airsim_odom_callback(const nav_msgs::Odometry::ConstPtr& input_odom);
void ctrl_md_callback(const mavros_msgs::AttitudeTarget& input_cmd);
int main(int argc, char** argv)
{
    ros::init(argc, argv, "state_transform_node");
    ros::NodeHandle nh("~");
    airsim_odom_sub= nh.subscribe( "input_odom", 100,airsim_odom_callback,ros::TransportHints().tcpNoDelay());
    cmd_sub = nh.subscribe("input_cmd", 100, ctrl_md_callback,ros::TransportHints().tcpNoDelay());
    odom_pub = nh.advertise<nav_msgs::Odometry>("output_odom",10);
    airsim_posecmd_pub   = nh.advertise<airsim_ros_pkgs::PoseCmd>("output_posecmd",10);
    airsim_bodyratecmd_pub   = nh.advertise<airsim_ros_pkgs::AngleRateThrottle>("output_bodyratecmd",10);
    odom_for_vio_pub = nh.advertise<nav_msgs::Odometry>("odom_for_vio",10);
    ros::spin();
    return 0;

}
void airsim_odom_callback(const nav_msgs::Odometry::ConstPtr& input_odom){
    nav_msgs::Odometry odom_msg;
    odom_msg = *input_odom;
    odom_msg.header.frame_id = "world";
    std::swap(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y);
    odom_msg.pose.pose.position.z = -odom_msg.pose.pose.position.z;
    Eigen::Quaterniond attitude_quater;
    attitude_quater.w() = odom_msg.pose.pose.orientation.w;
    attitude_quater.x() = odom_msg.pose.pose.orientation.x;
    attitude_quater.y() = odom_msg.pose.pose.orientation.y;
    attitude_quater.z() = odom_msg.pose.pose.orientation.z;//NED
    Eigen::Matrix3d rota = attitude_quater.toRotationMatrix();
    Eigen::Matrix3d nedtoenu;
    nedtoenu << 0,1,0,
                1,0,0,
                0,0,-1;//enu
    Eigen::Matrix3d body1tobody2;
    body1tobody2<<1,0,0,
                    0,-1,0,
                    0,0,-1;
    rota = nedtoenu*rota;
    rota = rota*body1tobody2;
    Eigen::Quaterniond enuq(rota);
    odom_msg.pose.pose.orientation.w = enuq.w();
    odom_msg.pose.pose.orientation.x = enuq.x();
    odom_msg.pose.pose.orientation.y = enuq.y();
    odom_msg.pose.pose.orientation.z = enuq.z();
    std::swap(odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y);
    odom_msg.twist.twist.linear.z = -odom_msg.twist.twist.linear.z;
    odom_msg.twist.twist.angular.y = -odom_msg.twist.twist.angular.y;
    odom_msg.twist.twist.angular.z = -odom_msg.twist.twist.angular.z;
    odom_pub.publish(odom_msg);

    std::swap(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y);
    odom_msg.pose.pose.position.y = -odom_msg.pose.pose.position.y;
    std::swap(odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y);
    odom_msg.twist.twist.linear.y = -odom_msg.twist.twist.linear.y;

    attitude_quater.w() = odom_msg.pose.pose.orientation.w;
    attitude_quater.x() = odom_msg.pose.pose.orientation.x;
    attitude_quater.y() = odom_msg.pose.pose.orientation.y;
    attitude_quater.z() = odom_msg.pose.pose.orientation.z;//NED
    rota = attitude_quater.toRotationMatrix();
    nedtoenu << 0,1,0,
                -1,0,0,
                0,0,1;//enu
    body1tobody2 << 1,0,0,
                     0,1,0,
                     0,0,1;//enu
    rota = body1tobody2 * nedtoenu * rota;
    Eigen::Quaterniond enuq_1(rota);
    odom_msg.pose.pose.orientation.w = enuq_1.w();
    odom_msg.pose.pose.orientation.x = enuq_1.x();
    odom_msg.pose.pose.orientation.y = enuq_1.y();
    odom_msg.pose.pose.orientation.z = enuq_1.z();

    Eigen::Vector3d body_rate;
    body_rate << odom_msg.twist.twist.angular.x, odom_msg.twist.twist.angular.y, odom_msg.twist.twist.angular.z;
    body_rate = body1tobody2 * nedtoenu * body_rate;
    odom_msg.twist.twist.angular.x = body_rate(0);
    odom_msg.twist.twist.angular.y = body_rate(1);
    odom_msg.twist.twist.angular.z = body_rate(2);

    odom_for_vio_pub.publish(odom_msg);
}
void ctrl_md_callback(const mavros_msgs::AttitudeTarget& input_cmd){
    airsim_ros_pkgs::PoseCmd cmd_msg;
    cmd_msg.throttle = input_cmd.thrust;
    Eigen::Matrix3d enutoned;
	enutoned << 0,1,0,
            	1,0,0,
                0,0,-1;
	Eigen::Matrix3d body1tobody2;
	body1tobody2<<1,0,0,
					0,-1,0,
					0,0,-1;
    Eigen::Quaterniond raw_q;
    raw_q.x() = input_cmd.orientation.x;
    raw_q.y() = input_cmd.orientation.y;
    raw_q.z() = input_cmd.orientation.z;
    raw_q.w() = input_cmd.orientation.w;
	Eigen::Quaterniond fianq(enutoned*raw_q.toRotationMatrix()*body1tobody2);
	Eigen::Vector3d eulerAngles;
	eulerAngles = uav_utils::quaternion_to_ypr(fianq);
	// pitch, roll, yaw
	double roll,pitch,yaw;
	cmd_msg.yaw = eulerAngles[0];
	cmd_msg.pitch = eulerAngles[1];
	cmd_msg.roll = eulerAngles[2];
    // airsim_posecmd_pub.publish(cmd_msg);

    airsim_ros_pkgs::AngleRateThrottle bodyrate_msg;
    bodyrate_msg.throttle = input_cmd.thrust;
    bodyrate_msg.rollRate = input_cmd.body_rate.x;
    bodyrate_msg.pitchRate = -input_cmd.body_rate.y;
    bodyrate_msg.yawRate = -input_cmd.body_rate.z;
    airsim_bodyratecmd_pub.publish(bodyrate_msg);

}
