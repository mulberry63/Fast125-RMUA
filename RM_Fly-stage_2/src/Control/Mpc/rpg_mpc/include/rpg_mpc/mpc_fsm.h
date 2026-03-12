#ifndef __MPCFSM_H
#define __MPCFSM_H

#include <ros/ros.h>
#include <ros/assert.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include "rpg_mpc/mpc_params.h"
#include "rpg_mpc/mpc_wrapper.h"
#include "rpg_mpc/mpc_input.h"
#include "rpg_mpc/mpc_controller.h"
// #include "ThrustCurve.h"
#include <mavros_msgs/SetMode.h>
#include <state_predictor/state_predictor.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/AttitudeTarget.h>

namespace rpg_mpc {


struct AutoTakeoffLand_t
{
	bool landed{true};
	ros::Time toggle_takeoff_land_time;
	//ros::Time toggle_auto_land_time;
	//ros::Time last_set_cmd_time;
	Eigen::Vector4d start_pose;
	
	static constexpr double MOTORS_SPEEDUP_TIME = 3.0; // motors idle running for 3 seconds before takeoff
};

template<typename T>
class MPCFSM
{
public:
	//***************PX4CTRL Real *****************

	RC_Data_t rc_data;
	State_Data_t state_data;
	ExtendedState_Data_t extended_state_data;
	Odom_Data_t odom_data;
	Imu_Data_t imu_data;
	Command_Data_t cmd_data;
	Battery_Data_t bat_data;
	Takeoff_Land_Data_t takeoff_land_data;
	Rpm_Data_t rpm_data;

	Start_Trigger_Data_t start_trigger_data;
	Cmd_Trigger_Data_t cmd_trigger_data;
	Trajectory_Data_t trajectory_data;
	ros::Publisher traj_start_trigger_pub;
	ros::Publisher ctrl_FCU_pub;
	ros::Publisher debug_pub; //debug
	ros::ServiceClient set_FCU_mode_srv;
	ros::ServiceClient arming_client_srv;
	ros::ServiceClient reboot_FCU_srv;

	ros::Publisher des_yaw_pub;
	ros::Publisher planning_stop_pub_;
    ros::Publisher planning_restart_pub_;

	Eigen::Vector4d hover_pose;
	ros::Time last_set_hover_pose_time;

	ros::Time Time_Offboard = ros::Time::now();
	int flag = 1;
	float dshot_value = 0.80;
	Eigen::Vector3d odom_eulerAngle;

	enum State_t
	{
		MANUAL_CTRL = 1, // px4ctrl is deactived. FCU is controled by the remote controller only
		AUTO_HOVER, // px4ctrl is actived, it will keep the drone hover from odom measurments while waiting for commands from PositionCommand topic.
		CMD_CTRL,	// px4ctrl is actived, and controling the drone.
		AUTO_TAKEOFF,
		AUTO_LAND,
	};

	MPCFSM(const ros::NodeHandle& nh, const ros::NodeHandle& pnh, MpcParams<T> &, MpcController<T> &);


	void process();
	bool rc_is_received(const ros::Time &now_time);
	bool cmd_is_received(const ros::Time &now_time);
	bool odom_is_received(const ros::Time &now_time);
	bool imu_is_received(const ros::Time &now_time);
	bool bat_is_received(const ros::Time &now_time);
	bool recv_new_odom();
	bool get_landed() { return takeoff_land.landed; }

private:
	// Subscribers and publisher.
	ros::Publisher pub_control_command, pub_predicted_trajectory_, first_predicted_pos_visualization;
	// ros::Timer exec_timer_, pub_cmd_timer_;

	AutoTakeoffLand_t takeoff_land;
	State_t fsm_state; // Should only be changed in PX4CtrlFSM::process() function!

	// Handles
	ros::NodeHandle nh_;
	ros::NodeHandle pnh_;

	MpcParams<T> &params_;
	MpcController<T> &controller_;

  	state_predictor::StatePredictor state_predictor_;
	quadrotor_common::QuadStateEstimate predicted_state;

	Eigen::Matrix<T, kStateSize, 1> est_state_;
	Eigen::Matrix<T, kStateSize, kSamples + 1> mpc_predicted_states_;
	Eigen::Matrix<T, kInputSize, kSamples> mpc_predicted_inputs_;
	
	void setEstimateState(const quadrotor_common::QuadStateEstimate &odom_est_state);
	void setTakeoffTraj(quadrotor_msgs::Trajectory &TakeoffTraj);
  	void setHoverTraj(quadrotor_msgs::Trajectory &HoverTraj);

	void publishPrediction(const Eigen::Ref<const Eigen::Matrix<T, kStateSize, kSamples + 1>> predicted_traj,
						   ros::Time& time);
	void PublishSpeedControlCommand(const Eigen::Ref<const Eigen::Matrix<T, kInputSize, 1>> predicted_input,
									ros::Time& time);
	void publish_thrust_ctrl(const Eigen::Ref<const Eigen::Matrix<T, kInputSize, 1>> predicted_input, const ros::Time &stamp);
	void publish_bodyrate_ctrl(const Eigen::Ref<const Eigen::Matrix<T, kInputSize, 1>> predicted_input, const ros::Time &stamp,
								double collective_thrust_normalized);
	void goToMPCPos(Eigen::Vector3d desired_position, Eigen::Vector3d desired_velocity, Eigen::Vector3d desired_acceleration,
					quadrotor_msgs::Trajectory &MPCTraj);
	
	bool first_time_start, first_time_hover;
	Eigen::Vector3d takeoff_pos, takeoff_vel, takeoff_acc;
	Eigen::Vector3d hover_pos, hover_vel, hover_acc;
	// ---- control related ----
	// quadrotor_common::QuadStateEstimate get_hover_des();
	// quadrotor_common::QuadStateEstimate get_cmd_des();

	// ---- auto takeoff/land ----
	// void motors_idling(const Imu_Data_t &imu, Controller_Output_t &u);
	// void land_detector(const State_t state, const Desired_State_t &des, const Odom_Data_t &odom); // Detect landing 
	// void set_start_pose_for_takeoff_land(const Odom_Data_t &odom);
	// Desired_State_t get_rotor_speed_up_des(const ros::Time now);
	// Desired_State_t get_takeoff_land_des(const double speed);

	// ---- tools ----
	// void set_hov_with_odom();
	// void set_hov_with_rc();
	void publish_trigger(const nav_msgs::Odometry &odom_msg);
	bool toggle_offboard_mode(bool on_off); // It will only try to toggle once, so not blocked.
	bool toggle_arm_disarm(bool arm); // It will only try to toggle once, so not blocked.
	void reboot_FCU();

	///newly added
	Eigen::Matrix4d control_allocation, inv_control_allocation;

	inline void calculateTorque(const Eigen::Ref<Eigen::Vector3d> ref_angular_acceleration,
								const Eigen::Ref<Eigen::Vector3d> ref_angular_velocity,
								const Eigen::Ref<Eigen::Matrix3d> inertia,
								Eigen::Ref<Eigen::Vector3d> torque){
		torque =  inertia *ref_angular_velocity + ref_angular_velocity.cross(inertia*ref_angular_velocity);
	}

	inline void calculateRotorSpeed(const double & thrust,
									const Eigen::Ref<Eigen::Vector3d> torque,
									const Eigen::Ref<Eigen::Matrix4d> control_allocation,
									Eigen::Ref<Eigen::Vector4d> rotor_speeds){
		Eigen::Vector4d thrust_and_torque;
		thrust_and_torque << thrust , torque;
		rotor_speeds = control_allocation * thrust_and_torque;
}

};

} //namespace rpg_mpc
#endif