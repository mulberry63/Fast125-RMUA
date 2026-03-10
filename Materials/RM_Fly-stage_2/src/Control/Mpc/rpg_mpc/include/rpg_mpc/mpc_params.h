/*    rpg_quadrotor_mpc
 *    A model predictive control implementation for quadrotors.
 *    Copyright (C) 2017-2018 Philipp Foehn, 
 *    Robotics and Perception Group, University of Zurich
 * 
 *    Intended to be used with rpg_quadrotor_control and rpg_quadrotor_common.
 *    https://github.com/uzh-rpg/rpg_quadrotor_control
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef __MPCPARAM_H
#define __MPCPARAM_H

#pragma once

#include <ros/ros.h>
#include <rpg_mpc/mpc_wrapper.h>
 
namespace rpg_mpc
{

template <typename T>
class MpcParams
{
	public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	struct Q_Gain
	{
		T Q_pos_xy;
		T Q_pos_z;
		T Q_attitude_rp;
		T Q_attitude_yaw;
		T Q_velocity;
		// T Q_bodyrate;
	};

	struct R_Gain
	{
		T R_thrust;
		T R_pitchroll;
		T R_yaw;
	};

	//For real-world experiments
	struct MsgTimeout
	{
		double odom;
		double rc;
		double cmd;
		double imu;
		double bat;
	};

	struct ThrustMapping
	{
		bool print_val;
		double K1;
		double K2;
		double K3;
		bool accurate_thrust_model;
		double hover_percentage;
	};

	struct AutoTakeoffLand
	{
		bool enable;
		bool enable_auto_arm;
		bool no_RC;
		double height;
		double speed;
	};

	// struct Mpc_Constraint
	// {
	// 	double min_thrust;
	// 	double max_thrust;
	// };

	// struct Dynamic_Parameter
	// {
	// 	double arm_length;
	// 	double rotor_drag_coeff;
	// 	double rotor_thrust_coeff;
	// 	double mass;
	// 	double inertia_x;
	// 	double inertia_y;
    // 	double inertia_z;
	// };


	Q_Gain q_gain;
	R_Gain r_gain;
	MsgTimeout msg_timeout;
	ThrustMapping thr_map;
	AutoTakeoffLand takeoff_land;

	// Mpc_Constraint mpc_constraint;
	// Dynamic_Parameter dynamic_parameter;



	// mpc constraint
	T min_thrust;
	T max_thrust;
	T max_bodyrate_xy;
	T max_bodyrate_z;

	T ctrl_freq_max;
	// Dynamic parameters
	T arm_length;
	T rotor_drag_coeff;
	T rotor_thrust_coeff;
	T mass;
	T inertia_x;
	T inertia_y;
	T inertia_z;

	T state_cost_exponential_;
	T input_cost_exponential_;

	T control_command_delay_;
	Eigen::Matrix<T, kCostSize, kCostSize> Q_;
	Eigen::Matrix<T, kInputSize, kInputSize> R_;  

	T takeoff_height_;
	T start_land_velocity_;
	T start_land_acceleration_;
	T traj_rcv_timeout_;

	bool changed_;
	bool print_info_;

	double max_angle;
	double max_manual_vel;
	double low_voltage;

	bool use_bodyrate_ctrl;
	bool use_thrust_ctrl;

  MpcParams() :
    changed_(false),
    print_info_(false),
    state_cost_exponential_(0.0),
    input_cost_exponential_(0.0),
    min_thrust(0.0),
    max_thrust(0.0),
	max_bodyrate_z(0.0),
	max_bodyrate_xy(0.0),
    Q_(Eigen::Matrix<T, kCostSize, kCostSize>::Zero()),
    R_(Eigen::Matrix<T, kInputSize, kInputSize>::Zero())
  {
  }

	~MpcParams()
	{
	}

	void config_from_ros_handle(const ros::NodeHandle &nh)
	{
		read_essential_param(nh, "Q_pos_xy",  q_gain.Q_pos_xy);
		read_essential_param(nh, "Q_pos_z",  q_gain.Q_pos_z);
		read_essential_param(nh, "Q_attitude_rp", q_gain.Q_attitude_rp);
		read_essential_param(nh, "Q_attitude_yaw", q_gain.Q_attitude_yaw);
		read_essential_param(nh, "Q_velocity", q_gain.Q_velocity);
		// read_essential_param(nh, "Q_bodyrate", q_gain.Q_bodyrate);

		read_essential_param(nh, "R_thrust", r_gain.R_thrust);
		read_essential_param(nh, "R_pitchroll", r_gain.R_pitchroll);
		read_essential_param(nh, "R_yaw", r_gain.R_yaw);

		read_essential_param(nh, "min_thrust", min_thrust);
		read_essential_param(nh, "max_thrust", max_thrust);
		read_essential_param(nh, "max_bodyrate_xy", max_bodyrate_xy);
		read_essential_param(nh, "max_bodyrate_z", max_bodyrate_z);

		read_essential_param(nh, "state_cost_exponential", state_cost_exponential_);
		read_essential_param(nh, "input_cost_exponential", input_cost_exponential_);

		read_essential_param(nh, "arm_length", arm_length);
		read_essential_param(nh, "rotor_drag_coeff", rotor_drag_coeff);
		read_essential_param(nh, "rotor_thrust_coeff", rotor_thrust_coeff);
		read_essential_param(nh, "mass", mass);
		read_essential_param(nh, "inertia_x", inertia_x);
		read_essential_param(nh, "inertia_y", inertia_y);
		read_essential_param(nh, "inertia_z", inertia_z);
		read_essential_param(nh, "control_command_delay", control_command_delay_);

		read_essential_param(nh, "takeoff_height", takeoff_height_);
		read_essential_param(nh, "start_land_velocity", start_land_velocity_);
		read_essential_param(nh, "start_land_acceleration", start_land_acceleration_);
		read_essential_param(nh, "traj_rcv_timeout", traj_rcv_timeout_);
		read_essential_param(nh, "ctrl_freq_max", ctrl_freq_max);


		read_essential_param(nh, "auto_takeoff_land/enable", takeoff_land.enable);
		read_essential_param(nh, "auto_takeoff_land/enable_auto_arm", takeoff_land.enable_auto_arm);
		read_essential_param(nh, "auto_takeoff_land/no_RC", takeoff_land.no_RC);
		read_essential_param(nh, "auto_takeoff_land/takeoff_height", takeoff_land.height);
		read_essential_param(nh, "auto_takeoff_land/takeoff_land_speed", takeoff_land.speed);

		read_essential_param(nh, "thrust_model/print_value", thr_map.print_val);
		read_essential_param(nh, "thrust_model/K1", thr_map.K1);
		read_essential_param(nh, "thrust_model/K2", thr_map.K2);
		read_essential_param(nh, "thrust_model/K3", thr_map.K3);
		read_essential_param(nh, "thrust_model/accurate_thrust_model", thr_map.accurate_thrust_model);
		read_essential_param(nh, "thrust_model/hover_percentage", thr_map.hover_percentage);

		// revised by wyz
		read_essential_param(nh, "max_manual_vel", max_manual_vel);
		read_essential_param(nh, "max_angle", max_angle);
		read_essential_param(nh, "low_voltage", low_voltage);
		read_essential_param(nh, "use_bodyrate_ctrl", use_bodyrate_ctrl);
		read_essential_param(nh, "use_thrust_ctrl", use_thrust_ctrl);

		read_essential_param(nh, "msg_timeout/odom", msg_timeout.odom);
		read_essential_param(nh, "msg_timeout/rc", msg_timeout.rc);
		read_essential_param(nh, "msg_timeout/cmd", msg_timeout.cmd);
		read_essential_param(nh, "msg_timeout/imu", msg_timeout.imu);
		read_essential_param(nh, "msg_timeout/bat", msg_timeout.bat);

		Q_ = (Eigen::Matrix<T, kCostSize, 1>() <<
			q_gain.Q_pos_xy, q_gain.Q_pos_xy, q_gain.Q_pos_z,
			q_gain.Q_attitude_rp, q_gain.Q_attitude_rp, q_gain.Q_attitude_rp, q_gain.Q_attitude_yaw,
			q_gain.Q_velocity, q_gain.Q_velocity, q_gain.Q_velocity).finished().asDiagonal();
		R_ = (Eigen::Matrix<T, kInputSize, 1>() <<
			r_gain.R_thrust, r_gain.R_pitchroll, r_gain.R_pitchroll, r_gain.R_yaw).finished().asDiagonal();
			
		max_angle /= (180.0 / M_PI);

		if ( takeoff_land.enable_auto_arm && !takeoff_land.enable )
		{
			takeoff_land.enable_auto_arm = false;
			ROS_ERROR("\"enable_auto_arm\" is only allowd with \"auto_takeoff_land\" enabled.");
		}
		if ( takeoff_land.no_RC && (!takeoff_land.enable_auto_arm || !takeoff_land.enable) )
		{
			takeoff_land.no_RC = false;
			ROS_ERROR("\"no_RC\" is only allowd with both \"auto_takeoff_land\" and \"enable_auto_arm\" enabled.");
		}

		if ( thr_map.print_val )
		{
			ROS_WARN("You should disable \"print_value\" if you are in regular usage.");
		}

		std::cout << "param ended!" << std::endl;
	};
	// void config_full_thrust(double hov);

private:
	template <typename TName, typename TVal>
	void read_essential_param(const ros::NodeHandle &nh, const TName &name, TVal &val)
	{
		if (nh.getParam(name, val))
		{
			// pass
		}
		else
		{
			ROS_ERROR_STREAM("Read param: " << name << " failed.");
			ROS_BREAK();
		}
	};
};
} // namespace rpg_mpc

#endif