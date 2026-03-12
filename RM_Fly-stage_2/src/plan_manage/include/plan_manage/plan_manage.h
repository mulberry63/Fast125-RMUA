#ifndef _PLAN_MANAGE_H_
#define _PLAN_MANAGE_H_

#include "ros/ros.h"
#include <math.h>
#include <iostream>
#include <Eigen/Eigen>

#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>

#include "traj_opt/global_traj_opt.h"
#include "traj_opt/global_traj_opt_replan.h"
#include "traj_opt/rm_traj_opt_uniform.h"

// #include <ros/package.h>
// #include <cmath>
// #include <memory>
// #include <geometry_msgs/Point.h>
// #include <ros/console.h>
// #include <sensor_msgs/Joy.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <visualization_msgs/Marker.h>
// #include <visualization_msgs/MarkerArray.h>
// #include "nav_msgs/Path.h"
// #include "sensor_msgs/point_cloud_conversion.h"
// #include <std_msgs/Float64.h>

namespace planner
{

class PlanManager
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber waypoints_sub_, odom_sub_, circle_sub_, depth_img_sub_;
    ros::Publisher traj_pub_, desired_yaw_pub_;
    ros::Timer process_timer_, replan_timer_;

    Eigen::Vector3d odom_p_, odom_v_;
    Eigen::Vector3d start_p_, start_v_, start_a_;
    Eigen::Vector3d target_p_, target_v_, target_a_;

    Eigen::MatrixXd initState_, finState_;

    int local_replan_type_;
    double min_replan_dist_, min_replan_duration_, min_circle_update_dist_;

    double piece_length_, replan_duration_, alpha_gate_update_;
    int gate_index_ = -1;
    bool get_glb_traj_ = false, get_lc_traj_ = false;
    Trajectory glb_traj_, lc_traj_;
    ros::Time glb_start_timestamp_, lc_start_timestamp_;

    std::string host_ip_;
    std::vector<std::string> gate_names_;
    std::vector<Eigen::Vector3d> gate_pos_list_;

    traj_opt::Config config_;
    std::shared_ptr<traj_opt::GlobTrajOpt> globTrajOptPtr_;
    std::shared_ptr<traj_opt::GlbTrajReplanOpt> lcTrajOptPtr_;
    std::shared_ptr<traj_opt::RmTrajOptUniform> lcTrajOptPtr_2_;
    std::shared_ptr<visualization::Visualization> visPtr_;

    ros::Publisher circle_vis_pub_, obs_vis_pub_;
    std::shared_ptr<Circle12> circle_12_;
    std::shared_ptr<Circle13> circle_13_;
    std::shared_ptr<MovingObs0> moving_obs_0_;
    std::shared_ptr<MovingObs0> moving_obs_1_;
    std::shared_ptr<MovingObs0> moving_obs_2_;
    std::shared_ptr<MovingObs0> moving_obs_3_;
    bool get_se3_traj_ = false;

    // for dyn_obs API
    std::shared_ptr<DynObs> moving_obs_;

    ros::Time odom_timestamp_;

    // for odom drift fix
    Eigen::Vector3d pg_to_vio_ = Eigen::Vector3d::Zero(), last_pg_to_vio_ = Eigen::Vector3d::Zero();
    ros::Subscriber pg_to_vio_sub_;

public:
    PlanManager(ros::NodeHandle& nh);
    ~PlanManager() {}

    void TimerCallback(const ros::TimerEvent& event);

    void ReplanCallback(const ros::TimerEvent& event);

    void rcvOdometryCallback(const nav_msgs::Odometry & odom);

    void rcvGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void rcvCircleCallback(const geometry_msgs::PoseStamped& msg);

    void rcvPgToVioCallback(const geometry_msgs::Pose& msg);

    void global_plan();

    void local_replan(const int& gate_index);

    void global_replan(const int& gate_index);

    void global_se3_replan(const int& gate_index);

    // void local_replan_2(const int& gate_index);

    // void replan_from_lc_traj(const int& gate_index);

    // void replan_from_glb_traj(const int& gate_index);

    void get_replan_state(ros::Time& replan_timestamp, Eigen::MatrixXd& initState, Eigen::MatrixXd& finState, Trajectory& tmp_traj, const int& gate_index, double& replan_t);

    void visualize_traj();

    void visualize_rviz_obs();

    void visualize_rviz_moving_circle();

    void publish_traj(const Trajectory& traj, const ros::Time& traj_timestamp);

    quadrotor_msgs::PolynomialTrajectory traj2msg(const Trajectory& traj, const ros::Time& traj_timestamp);

    void publish_desired_yaw();

    double calculate_desired_yaw();

};// class plan_manager

}// namespace planner

#endif
