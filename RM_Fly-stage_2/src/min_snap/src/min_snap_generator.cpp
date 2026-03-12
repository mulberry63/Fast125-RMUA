#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <quadrotor_msgs/PositionCommand.h>
#include "min_snap/min_snap_closeform.h"

ros::Publisher goal_list_pub;
ros::Publisher poly_coef_pub;
ros::Subscriber goal_list_sub;
ros::Subscriber rviz_goal_sub;
ros::Subscriber cmd_sub;
ros::Subscriber odom_sub;
ros::Subscriber imu_sub;

int id = 0;
double meanvel = 1.0;
nav_msgs::Odometry odom;
sensor_msgs::Imu imu_msg;
geometry_msgs::Pose goal_pt;
geometry_msgs::PoseArray goal_list;
// my_planner::minsnapOptimization minsnap_solver;
my_planner::minsnapCloseform minsnap_solver;
std::vector<Eigen::Vector3d> waypoints;
Eigen::MatrixXd start_vaj = Eigen::MatrixXd::Zero(3, 3);
Eigen::MatrixXd end_vaj = Eigen::MatrixXd::Zero(3, 3);
quadrotor_msgs::PolynomialTrajectory poly_pub_topic;

// target list
int target_list_size_ = 0;
std::vector<Eigen::Vector3d> target_list_pos_;
Eigen::Vector3d end_vel_{0, 0, 0};
bool fake_planning_flag = false;
bool plan_ok = false;


void pub_poly_coefs(bool pub_flag)
{
    Eigen::MatrixXd poly_coef = minsnap_solver.getPolyCoef();
    Eigen::MatrixXd dec_vel = minsnap_solver.getDecVel();
    // Eigen::MatrixXd lambda = minsnap_solver.getLambda();
    Eigen::VectorXd time = minsnap_solver.getTime();

    poly_pub_topic.num_segment += time.size();
    poly_pub_topic.trajectory_id = id;

    ROS_WARN("decision variable:");
    for (int i = 0; i < time.size()+1; i++)
    {
        cout << "Point number = " << i + 1 << endl
             << dec_vel.middleRows(i * 4, 4) << endl;
    }

    // ROS_WARN("lambda variable:");
    // cout << lambda << endl;

    for (int i = 0; i < time.size(); i++)
    {
        for (int j = (i + 1) * 8 - 1; j >= i * 8; j--)
        {
            poly_pub_topic.coef_x.push_back(poly_coef(j, 0));
            poly_pub_topic.coef_y.push_back(poly_coef(j, 1));
            poly_pub_topic.coef_z.push_back(poly_coef(j, 2));
        }
        poly_pub_topic.time.push_back(time(i));
    }

    poly_pub_topic.header.frame_id = "world";
    poly_pub_topic.header.stamp = ros::Time::now();

    if(pub_flag)
    {
        poly_coef_pub.publish(poly_pub_topic);

        poly_pub_topic.coef_x.clear();
        poly_pub_topic.coef_y.clear();
        poly_pub_topic.coef_z.clear();
        poly_pub_topic.time.clear();
        poly_pub_topic.num_segment = 0;
    }
}

void fake_planning()
{
    goal_list.poses.clear();
    geometry_msgs::Pose odom_pt;
    odom_pt = odom.pose.pose;
    goal_list.poses.push_back(odom_pt);

    for( int i = 0; i < target_list_pos_.size(); i++ )
    {
        goal_pt.position.x = odom_pt.position.x + target_list_pos_[i][0];
        goal_pt.position.y = odom_pt.position.y + target_list_pos_[i][1];
        goal_pt.position.z = odom_pt.position.z + target_list_pos_[i][2];
        goal_list.poses.push_back(goal_pt);
    }
    goal_list.header.stamp = ros::Time::now();
    goal_list.header.frame_id = "world";
    goal_list.header.seq = id++;
    goal_list_pub.publish(goal_list);

    Eigen::Vector3d wp;
    waypoints.clear();
    for (int i = 0; i < int(goal_list.poses.size()); i++)
    {
        wp << goal_list.poses[i].position.x, goal_list.poses[i].position.y, goal_list.poses[i].position.z;
        waypoints.push_back(wp);
    }
    minsnap_solver.Init(waypoints, meanvel);

    start_vaj(0, 0) = odom.twist.twist.linear.x;
    start_vaj(0, 1) = odom.twist.twist.linear.y;
    start_vaj(0, 2) = odom.twist.twist.linear.z;

    // wz
    Eigen::Vector3d cur_acceleration; 
    cur_acceleration(0) =  imu_msg.linear_acceleration.x;
    cur_acceleration(1) = -imu_msg.linear_acceleration.y;
    cur_acceleration(2) = -imu_msg.linear_acceleration.z;

    Eigen::Quaterniond q;
    q.w()=odom.pose.pose.orientation.w;
    q.x()=odom.pose.pose.orientation.x;
    q.y()=odom.pose.pose.orientation.y;
    q.z()=odom.pose.pose.orientation.z;
    
    double G=9.80665;
    Eigen::Vector3d world_acc = q*cur_acceleration- Eigen::Vector3d(0, 0, G);

    start_vaj(1, 0) = world_acc[0];
    start_vaj(1, 1) = world_acc[1];
    start_vaj(1, 2) = world_acc[2];

    std::cout << "P: " << odom.pose.pose.position.x << " " << odom.pose.pose.position.y << " " << odom.pose.pose.position.z << std::endl;
    std::cout << "V: " << odom.twist.twist.linear.x << " " << odom.twist.twist.linear.y << " " << odom.twist.twist.linear.z << std::endl;
    std::cout << "A: " << world_acc.transpose() << std::endl;

    ROS_INFO("Init success");
    minsnap_solver.set_sta_state(start_vaj);
    minsnap_solver.set_end_state(end_vaj);
    minsnap_solver.calMinsnap_polycoef();
    pub_poly_coefs(1);  
}

void odom_goal_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    odom = *msg;

    // wxx shit trick
    if( abs( msg->header.stamp.toSec() - imu_msg.header.stamp.toSec() ) > 0.01 )
        return;

    if( fake_planning_flag )
    {
        if( plan_ok )
            return;
        
        if( abs(odom.pose.pose.position.x) < 10 && abs(odom.pose.pose.position.y) < 10 && abs(odom.pose.pose.position.z) < 10 )
        {
            fake_planning();  
            plan_ok = true;
        }

    }
}


void imu_cb(const sensor_msgs::Imu ::ConstPtr &msg)
{
    imu_msg = *msg;
}



void cmd_cb(const quadrotor_msgs::PositionCommand::ConstPtr &msg)
{
    start_vaj(0, 0) = msg->velocity.x;
    start_vaj(0, 1) = msg->velocity.y;
    start_vaj(0, 2) = msg->velocity.z;

    start_vaj(1, 0) = msg->acceleration.x;
    start_vaj(1, 1) = msg->acceleration.y;
    start_vaj(1, 2) = msg->acceleration.z;

    start_vaj(2, 0) = msg->jerk.x;
    start_vaj(2, 1) = msg->jerk.y;
    start_vaj(2, 2) = msg->jerk.z;
}


// debug
void solve_min_snap()
{
    Eigen::MatrixXd mid_vaj = Eigen::MatrixXd::Zero(3, 3);
    mid_vaj << 0.0, 0, 2, 
               0, 0, 2, 
               0, 0, 0;
    Eigen::Vector3d wp;
    waypoints.clear();

    wp << odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z;
    waypoints.push_back(wp);
    wp << goal_list.poses[0].position.x, goal_list.poses[0].position.y, goal_list.poses[0].position.z;
    waypoints.push_back(wp);

    minsnap_solver.Init(waypoints, -1.0);
    minsnap_solver.set_sta_state(start_vaj);
    minsnap_solver.set_end_state(mid_vaj);
    minsnap_solver.calMinsnap_polycoef();
    pub_poly_coefs(0);  

    waypoints.clear();
    for (int i = 0; i < int(goal_list.poses.size()); i++)
    {
        wp << goal_list.poses[i].position.x, goal_list.poses[i].position.y, goal_list.poses[i].position.z;
        waypoints.push_back(wp);
    }
    if (meanvel > 0)
    {
        minsnap_solver.Init(waypoints, meanvel);
    }
    else
    {
        minsnap_solver.Init(waypoints);
    }
    ROS_INFO("Init success");
    minsnap_solver.set_sta_state(mid_vaj);
    minsnap_solver.set_end_state(end_vaj);
    minsnap_solver.calMinsnap_polycoef();
    pub_poly_coefs(1);    
}

// void solve_min_snap()
// {
//     Eigen::Vector3d wp;
//     waypoints.clear();

//     for (int i = 0; i < int(goal_list.poses.size()); i++)
//     {
//         wp << goal_list.poses[i].position.x, goal_list.poses[i].position.y, goal_list.poses[i].position.z;
//         waypoints.push_back(wp);
//     }
//     if (meanvel > 0)
//     {
//         minsnap_solver.Init(waypoints, meanvel);
//     }
//     else
//     {
//         minsnap_solver.Init(waypoints);
//     }
//     ROS_INFO("Init success");
//     minsnap_solver.set_sta_state(start_vaj);
//     minsnap_solver.set_end_state(end_vaj);
//     minsnap_solver.calMinsnap_polycoef();
//     pub_poly_coefs();    
// }

void triger_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    goal_list.poses.clear();

    // goal_pt = odom.pose.pose;
    // goal_list.poses.push_back(goal_pt);

    for( int i = 0; i < target_list_pos_.size(); i++ )
    {
        goal_pt.position.x = target_list_pos_[i][0];
        goal_pt.position.y = target_list_pos_[i][1];
        goal_pt.position.z = target_list_pos_[i][2];
        goal_list.poses.push_back(goal_pt);
    }
    goal_list.header.stamp = ros::Time::now();
    goal_list.header.frame_id = "world";
    goal_list.header.seq = id++;
    goal_list_pub.publish(goal_list);

    solve_min_snap();
    ROS_INFO("solver finished");
    
    plan_ok = false;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "min_snap_generator");
    ros::NodeHandle nh("~");
    ros::Rate rate(10);

    // read target list and end point vel from xml
    std::vector<double> end_vel_vec;
    std::vector<double> target_list_pos_vec;
    nh.param<std::vector<double>>("target_list/end_vel", end_vel_vec, std::vector<double>());
    nh.param("target_list/list_size", target_list_size_, 0);
    nh.param<std::vector<double>>("target_list/target_pos", target_list_pos_vec, std::vector<double>());

    end_vel_ = Eigen::Vector3d{end_vel_vec[0], end_vel_vec[1], end_vel_vec[2]};
    end_vaj << end_vel_.transpose(), Eigen::Vector3d::Zero().transpose(), Eigen::Vector3d::Zero().transpose();
    std::cout << end_vaj << std::endl;
    if( target_list_pos_vec.size() < (target_list_size_*3) )
        ROS_ERROR("ERROR in obs_pos size!");
    target_list_pos_.clear();  
    for( int i = 0; i < target_list_size_; i++ )
    {
        target_list_pos_.push_back(Eigen::Vector3d{target_list_pos_vec[3*i], target_list_pos_vec[3*i+1], target_list_pos_vec[3*i+2]});
    }

    ros::param::get("/min_snap_generator/mean_vel", meanvel);
    ros::param::get("/min_snap_generator/fake_planning_flag", fake_planning_flag);


    odom_sub = nh.subscribe("/odom_topic", 10, odom_goal_cb);
    imu_sub = nh.subscribe("/imu_topic", 10, imu_cb);
    cmd_sub = nh.subscribe("/position_cmd", 10, cmd_cb);
    rviz_goal_sub = nh.subscribe("/rviz_goal", 10, triger_cb);
    goal_list_pub = nh.advertise<geometry_msgs::PoseArray>("/goal_list", 10);
    poly_coef_pub = nh.advertise<quadrotor_msgs::PolynomialTrajectory>("/poly_coefs", 10);


    poly_pub_topic.num_order = 7;
    poly_pub_topic.start_yaw = 0;
    poly_pub_topic.final_yaw = 0;
    poly_pub_topic.mag_coeff = 0;
    poly_pub_topic.order.push_back(0);

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}