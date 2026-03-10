#ifndef __CtrlFSM_H
#define __CtrlFSM_H

#include <ros/ros.h>
#include <ros/assert.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/SetMode.h>

#include "input.h"
#include "hovthrkf.h"
#include "controller.h"

#include "rpg_mpc/mpc_params.h"
#include "rpg_mpc/mpc_wrapper.h"
#include "rpg_mpc/mpc_input.h"
#include "rpg_mpc/mpc_controller.h"
#include <mavros_msgs/SetMode.h>
#include <state_predictor/state_predictor.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/AttitudeTarget.h>

class CtrlFSM
{
public:
	Parameter_t& param;

	RC_Data_t rc_data;
	State_Data_t state_data;
	Odom_Data_t odom_data;
	Imu_Data_t imu_data;
	Command_Data_t cmd_data;

	Controller& controller;
	HovThrKF& hov_thr_kf;

	
	rpg_mpc::MpcController<float> &mpc_controller;
	Eigen::Matrix<float, rpg_mpc::kStateSize, rpg_mpc::kSamples + 1> mpc_predicted_states_;
	Eigen::Matrix<float, rpg_mpc::kInputSize, rpg_mpc::kSamples> mpc_predicted_inputs_;
	Eigen::Matrix<float, rpg_mpc::kStateSize, 1> est_state_;
	Trajectory_Data_t trajectory_data;
	

	// ros::Publisher des_pose_pub;
	ros::Publisher traj_start_trigger_pub;

	Eigen::Vector4d hover_pose;

	enum State_t
	{
		MANUAL_CTRL,             // Ctrl is deactived. FCU is controled by the remote controller only
		AUTO_HOVER,	 			// Ctrl is actived, it will keep the drone hover from odom measurments while waiting for commands from PositionCommand topic.
		CMD_CTRL				 // Ctrl is actived, and controling the drone.
	};

	CtrlFSM(Parameter_t &, Controller &, HovThrKF &, rpg_mpc::MpcController<float> &);
	void process();
	bool rc_is_received(const ros::Time& now_time);
	bool cmd_is_received(const ros::Time& now_time);
	bool odom_is_received(const ros::Time& now_time);
	bool imu_is_received(const ros::Time& now_time);
	bool px4_init();


private:
	State_t state;
	// ---- control related ----
	void process_hover_control(Controller_Output_t& u, SO3_Controller_Output_t& u_so3);
	void process_cmd_control(Controller_Output_t& u, SO3_Controller_Output_t& u_so3);
	// void publish_desire(const Desired_State_t& des);

	// ---- tools ----
	double get_yaw_from_odom();
	void align_with_imu(Controller_Output_t& u);
	void set_hov_with_odom();

	void toggle_offboard_mode(bool on_off);  // It will only try to toggle once, so not blocked.

	void publish_trigger(const nav_msgs::Odometry& odom_msg);
};

#endif