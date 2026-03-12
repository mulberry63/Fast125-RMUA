#include "rpg_mpc/mpc_fsm.h"
#include <uav_utils/converters.h>

using namespace std;
using namespace uav_utils;

namespace rpg_mpc {

template<typename T>
MPCFSM<T>::MPCFSM(const ros::NodeHandle& nh, const ros::NodeHandle& pnh, MpcParams<T> &params, MpcController<T> &controller) :
	nh_(nh),
	pnh_(pnh),
	state_predictor_(nh_, pnh_),
	predicted_state(),
	params_(params),
	controller_(controller),
	est_state_((Eigen::Matrix<T, kStateSize, 1>() << 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0).finished()),
	mpc_predicted_states_(Eigen::Matrix<T, kStateSize, kSamples + 1>::Zero()),
    mpc_predicted_inputs_(Eigen::Matrix<T, kInputSize, kSamples>::Zero())
{
	fsm_state = MANUAL_CTRL;
	hover_pose.setZero();

	pub_predicted_trajectory_ =
    nh_.advertise<nav_msgs::Path>("/hummingbird/mpc/trajectory_predicted", 1);

  	pub_control_command = 
    nh_.advertise<mav_msgs::Actuators>("/hummingbird/command/motor_speed", 1);

}

/* 
        Finite State Machine

           system start
               /
              /
             v
----- > MANUAL_CTRL <-----------------
|         ^   |    \                 |
|         |   |     \                |
|         |   |      > AUTO_TAKEOFF  |
|         |   |        /             |
|         |   |       /              |
|         |   |      /               |
|         |   v     /                |
|       AUTO_HOVER <                 |
|         ^   |  \  \                |
|         |   |   \  \               |
|         |	  |    > AUTO_LAND -------
|         |   |
|         |   v
-------- CMD_CTRL

*/

template<typename T>
void MPCFSM<T>::process()
{
	ros::Time now_time = ros::Time::now();
	bool mpc_trig = false;

	predicted_state = odom_data.received_state_est_;

	//use or not use state_predictor 
	// state_predictor_.updateWithStateEstimate(predicted_state);
	// ros::Time wall_time_now = ros::Time::now();
	// ros::Time command_execution_time =
	// wall_time_now + ros::Duration(params_.control_command_delay_);
	// predicted_state = state_predictor_.predictState(command_execution_time);

	setEstimateState(predicted_state);
	double collective_thrust_normalized;
	// cout << "11111" << endl;
	switch (fsm_state)
	{
		case MANUAL_CTRL:
		{
			// cout << "params_.takeoff_land.enable: " << params_.takeoff_land.enable << endl;
			// cout << "takeoff_land_data.triggered: " << takeoff_land_data.triggered << endl;
			// cout << "takeoff_land_data.takeoff_land_cmd: " << takeoff_land_data.takeoff_land_cmd << endl;
			mpc_trig = false;
			if (rc_data.enter_hover_mode) // Try to jump to AUTO_HOVER
			{
				if (!odom_is_received(now_time))
				{
					ROS_ERROR("[MPCctrl] Reject AUTO_HOVER(L2). No odom!");
					break;
				}
				if (cmd_is_received(now_time))
				{
					ROS_ERROR("[MPCctrl] Reject AUTO_HOVER(L2). You are sending commands before toggling into AUTO_HOVER, which is not allowed. Stop sending commands now!");
					break;
				}
				if (odom_data.v.norm() > 3.0)
				{
					ROS_ERROR("[MPCctrl] Reject AUTO_HOVER(L2). Odom_Vel=%fm/s, which seems that the locolization module goes wrong!", odom_data.v.norm());
					break;
				}
				mpc_trig = true;
				quadrotor_msgs::Trajectory HoverTraj;
				setHoverTraj(HoverTraj);
				controller_.execMPC(HoverTraj, est_state_, mpc_predicted_states_, mpc_predicted_inputs_, collective_thrust_normalized);				
				first_time_hover = true;
				fsm_state = AUTO_HOVER;
				controller_.resetThrustMapping();
				// set_hov_with_odom();
				toggle_offboard_mode(true);

				ROS_INFO("\033[32m[MPCctrl] MANUAL_CTRL(L1) --> AUTO_HOVER(L2)\033[32m");
			}
			else if ( params_.takeoff_land.enable && takeoff_land_data.triggered && takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::TakeoffLand::TAKEOFF) // Try to jump to AUTO_TAKEOFF
			{
				// cout << "takeoffmode!" << endl;
				if (!odom_is_received(now_time))
				{
					ROS_ERROR("[MPCctrl] Reject AUTO_TAKEOFF. No odom!");
					break;
				}
				if (!cmd_is_received(now_time))
				{
					ROS_ERROR("[MPCctrl] You haven't received MPC takeoff traj yet!");
					break;
				}
				if (odom_data.v.norm() > 0.1)
				{
					ROS_ERROR("[MPCctrl] Reject AUTO_TAKEOFF. Odom_Vel=%fm/s, non-static takeoff is not allowed!", odom_data.v.norm());
					break;
				}
				// if (!get_landed())
				// {
				// 	ROS_ERROR("[MPCctrl] Reject AUTO_TAKEOFF. land detector says that the drone is not landed now!");
				// 	break;
				// }
				// if (rc_is_received(now_time)) // Check this only if RC is connected.
				// {
				// 	if (!rc_data.is_hover_mode || !rc_data.is_command_mode || !rc_data.check_centered() )
				// 	{
				// 		ROS_ERROR("[MPCctrl] Reject AUTO_TAKEOFF. If you have your RC connected, keep its switches at \"auto hover\" and \"command control\" states, and all sticks at the center, then takeoff again.");
				// 		while (ros::ok())
				// 		{
				// 			ros::Duration(0.01).sleep();
				// 			ros::spinOnce();
				// 			if (rc_data.is_hover_mode && rc_data.is_command_mode && rc_data.check_centered())
				// 			{
				// 				ROS_INFO("\033[32m[MPCctrl] OK, you can takeoff again.\033[32m");
				// 				break;
				// 			}
				// 		}
				// 		break;
				// 	}
				// }
				fsm_state = AUTO_TAKEOFF;
				controller_.resetThrustMapping();
				// set_start_pose_for_takeoff_land(odom_data);
				mpc_trig = true;
				// quadrotor_msgs::Trajectory TakeoffTraj;
				// setTakeoffTraj(TakeoffTraj);

				toggle_offboard_mode(true); // toggle on offboard before arm
				for (int i = 0; i < 10 && ros::ok(); ++i) // wait for 0.1 seconds to allow mode change by FMU // mark
				{
					ros::Duration(0.01).sleep();
					ros::spinOnce();
				}

				controller_.execMPC(trajectory_data.mpc_traj, est_state_, mpc_predicted_states_, mpc_predicted_inputs_, collective_thrust_normalized);
				// if (params_.takeoff_land.enable_auto_arm)
				// {
				// 	toggle_arm_disarm(true);
				// }
				takeoff_land.toggle_takeoff_land_time = now_time;
				// first_time_start = true;
				ROS_INFO("\033[32m[MPCctrl] MANUAL_CTRL(L1) --> AUTO_TAKEOFF\033[32m");
				takeoff_land_data.triggered = false;	
			}

			if (rc_data.toggle_reboot) // Try to reboot. EKF2 based PX4 FCU requires reboot when its state estimator goes wrong.
			{
				if (state_data.current_state.armed)
				{
					ROS_ERROR("[MPCctrl] Reject reboot! Disarm the drone first!");
					break;
				}
				reboot_FCU();
			}

			break;
		}

		case AUTO_TAKEOFF:
		{
			mpc_trig = true;

			if(odom_data.received_state_est_.position[2] > params_.takeoff_height_){
				//reached takeoff_height! switch to hover state
				quadrotor_msgs::Trajectory HoverTraj;
				setHoverTraj(HoverTraj);
				controller_.execMPC(HoverTraj, est_state_, mpc_predicted_states_, mpc_predicted_inputs_, collective_thrust_normalized);				
				first_time_hover = true;
				fsm_state = AUTO_HOVER;
				ROS_INFO("\033[32m[MPCctrl] TAKEOFF(L1) --> AUTO_HOVER(L2)\033[32m");
			}
			else{
				toggle_offboard_mode(true);
				controller_.execMPC(trajectory_data.mpc_traj, est_state_, mpc_predicted_states_, mpc_predicted_inputs_, collective_thrust_normalized);
			}
			break;
		}

		case AUTO_HOVER:
		{
			mpc_trig = false;
			if (!rc_data.is_hover_mode || !odom_is_received(now_time))
			{
				fsm_state = MANUAL_CTRL;
				toggle_offboard_mode(false);

				ROS_WARN("[MPCctrl] AUTO_HOVER(L2) --> MANUAL_CTRL(L1)");
			}
			else if (rc_data.is_command_mode && cmd_is_received(now_time))
			{
				if (state_data.current_state.mode == "GUIDED_NOGPS")
				{
					mpc_trig = true;
					controller_.execMPC(trajectory_data.mpc_traj, est_state_, mpc_predicted_states_, mpc_predicted_inputs_, collective_thrust_normalized);
					fsm_state = CMD_CTRL;
					ROS_INFO("\033[32m[MPCctrl] AUTO_HOVER(L1) --> CMD_CTRL(L2)\033[32m");
				}
			}
			else if (takeoff_land_data.triggered && takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::TakeoffLand::LAND)
			{
				//To do ZRB!!
				fsm_state = AUTO_LAND;
				// set_start_pose_for_takeoff_land(odom_data);

				ROS_INFO("\033[32m[MPCctrl] AUTO_HOVER(L2) --> AUTO_LAND\033[32m");
				takeoff_land_data.triggered = false;	
			}
			else
			{
				mpc_trig = true;
				quadrotor_msgs::Trajectory HoverTraj;
				setHoverTraj(HoverTraj);
				// ROS_INFO_STREAM("state: " << est_state_);
				controller_.execMPC(HoverTraj, est_state_, mpc_predicted_states_, mpc_predicted_inputs_, collective_thrust_normalized);
				if (rc_data.enter_command_mode)
				{
					publish_trigger(odom_data.msg);
					ROS_INFO("\033[32m[MPCctrl] TRIGGER sent, allow user command.\033[32m");
				}

				// cout << "des.p=" << des.p.transpose() << endl;
			}

			break;
		}

		case CMD_CTRL:
		{
			mpc_trig = false;
			if (!rc_data.is_hover_mode || !odom_is_received(now_time))
			{
				fsm_state = MANUAL_CTRL;
				toggle_offboard_mode(false);

				ROS_WARN("[MPCctrl] From CMD_CTRL(L3) to MANUAL_CTRL(L1)!");
			}
			else if (!rc_data.is_command_mode || !cmd_is_received(now_time))
			{
				fsm_state = AUTO_HOVER;
				mpc_trig = true;
				quadrotor_msgs::Trajectory HoverTraj;
				setHoverTraj(HoverTraj);
				controller_.execMPC(HoverTraj, est_state_, mpc_predicted_states_, mpc_predicted_inputs_, collective_thrust_normalized);
				ROS_INFO("[MPCctrl] From CMD_CTRL(L3) to AUTO_HOVER(L2)!");
			}
			else
			{
				mpc_trig = true;
				controller_.execMPC(trajectory_data.mpc_traj, est_state_, mpc_predicted_states_, mpc_predicted_inputs_, collective_thrust_normalized);
			}

			if (takeoff_land_data.triggered && takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::TakeoffLand::LAND)
			{
				ROS_ERROR("[MPCctrl] Reject AUTO_LAND, which must be triggered in AUTO_HOVER. \
						Stop sending control commands for longer than %fs to let px4ctrl return to AUTO_HOVER first.",
						params_.msg_timeout.cmd);
				takeoff_land_data.triggered = false;	
			}

			//ZRB::: 6.11 simulation_test
			// if(trajectory_data.rcv_traj_interval < params_.traj_rcv_timeout_){
			// 	mpc_trig = true;
			// 	controller_.execMPC(trajectory_data.mpc_traj, est_state_, mpc_predicted_states_, mpc_predicted_inputs_);
			// }

			break;
		}
		default:
			break;
	}
	
	if (fsm_state == AUTO_HOVER || fsm_state == CMD_CTRL)
	{
		controller_.estimateThrustModel(imu_data.a, bat_data.volt, odom_data.v, params_);
	}

	if(mpc_trig){
		ros::Time pub_time = ros::Time::now();
		//pub predicted trajectory
		publishPrediction(mpc_predicted_states_, pub_time);
		// //pub control command and update state predictor
		//ZRB warning 2022.6.10
		// if(fsm_state != CMD_CTRL)
		PublishSpeedControlCommand(mpc_predicted_inputs_.col(0).template cast<T>(), pub_time);

		// toggle_offboard_mode(true);
		if (params_.use_thrust_ctrl)
		{
			publish_thrust_ctrl(mpc_predicted_inputs_.col(0).template cast<T>(), now_time);
		}
		else if(params_.use_bodyrate_ctrl){
			publish_bodyrate_ctrl(mpc_predicted_inputs_.col(0).template cast<T>(), now_time, collective_thrust_normalized);
		}
	}

	// STEP6: Clear flags beyound their lifetime
	rc_data.enter_hover_mode = false;
	rc_data.enter_command_mode = false;
	rc_data.toggle_reboot = false;
}


template<typename T>
void MPCFSM<T>::setEstimateState(const quadrotor_common::QuadStateEstimate &odom_est_state){
  est_state_(kPosX) = odom_est_state.position[0];
  est_state_(kPosY) = odom_est_state.position[1];
  est_state_(kPosZ) = odom_est_state.position[2];
  est_state_(kOriW) = odom_est_state.orientation.w();
  est_state_(kOriX) = odom_est_state.orientation.x();
  est_state_(kOriY) = odom_est_state.orientation.y();
  est_state_(kOriZ) = odom_est_state.orientation.z();
  est_state_(kVelX) = odom_est_state.velocity[0];
  est_state_(kVelY) = odom_est_state.velocity[1];
  est_state_(kVelZ) = odom_est_state.velocity[2];
//   est_state_(kAngX) = odom_est_state.bodyrates[0];
//   est_state_(kAngY) = odom_est_state.bodyrates[1];
//   est_state_(kAngZ) = odom_est_state.bodyrates[2];
  

  // yaw pitch roll
  odom_eulerAngle = odom_est_state.orientation.matrix().eulerAngles(2,1,0);
//   cout << "odom_eulerAngle: " << odom_eulerAngle << endl;
}

template<typename T>
void MPCFSM<T>::setTakeoffTraj(quadrotor_msgs::Trajectory &TakeoffTraj){
	static ros::Time start_time = ros::Time::now();
	
	takeoff_vel << 0.0, 0.0, 0.0;
	takeoff_acc << 0.0, 0.0, 0.0;
	if(first_time_start){
		start_time = ros::Time::now();
		takeoff_pos[0] = odom_data.received_state_est_.position[0];
		takeoff_pos[1] = odom_data.received_state_est_.position[1];    
		first_time_start = false;
  	}

	takeoff_pos[2] = params_.start_land_velocity_ * (ros::Time::now() - start_time).toSec();
  	takeoff_vel[2] = params_.start_land_velocity_;
	if ((ros::Time::now() - start_time).toSec()  < params_.start_land_velocity_ / params_.start_land_acceleration_) {
		takeoff_acc[2] = params_.start_land_acceleration_;
		takeoff_vel[2] = params_.start_land_acceleration_ * (ros::Time::now() - start_time).toSec();
	} else {
		takeoff_acc[2] = 0;
	}
	goToMPCPos(takeoff_pos, takeoff_vel, takeoff_acc, TakeoffTraj);
}

template<typename T>
void MPCFSM<T>::setHoverTraj(quadrotor_msgs::Trajectory &HoverTraj){
	static ros::Time start_time = ros::Time::now();
	
	hover_vel << 0.0, 0.0, 0.0;
	hover_acc << 0.0, 0.0, 0.0;
	if(first_time_hover){
		start_time = ros::Time::now();
		hover_pos[0] = odom_data.received_state_est_.position[0];
		hover_pos[1] = odom_data.received_state_est_.position[1];    
		hover_pos[2] = params_.takeoff_height_;
		first_time_hover = false;
  	}

	goToMPCPos(hover_pos, hover_vel, hover_acc, HoverTraj);
}

template<typename T>
void MPCFSM<T>::goToMPCPos(Eigen::Vector3d desired_position, Eigen::Vector3d desired_velocity, Eigen::Vector3d desired_acceleration,
					quadrotor_msgs::Trajectory &MPCTraj){

  quadrotor_msgs::TrajectoryPoint mpc_ref_point;
//   double desired_yaw = 0.0;
  MPCTraj.type = 4; //snap
  MPCTraj.header.stamp = ros::Time::now();
  MPCTraj.header.frame_id = "world";
 
  for(int i = 0; i < 21; i++){
	mpc_ref_point.time_from_start = ros::Duration(0 + i*0.05);
	mpc_ref_point.pose.position.x =  (desired_position[0] - odom_data.received_state_est_.position[0]) / 20.0 * double(i) + odom_data.received_state_est_.position[0];
	mpc_ref_point.pose.position.y =  (desired_position[1] - odom_data.received_state_est_.position[1]) / 20.0 * double(i) + odom_data.received_state_est_.position[1];
	mpc_ref_point.pose.position.z =  (desired_position[2] - odom_data.received_state_est_.position[2]) / 20.0 * double(i) + odom_data.received_state_est_.position[2];
	mpc_ref_point.velocity.linear.x =  desired_velocity[0];
	mpc_ref_point.velocity.linear.y =  desired_velocity[1];
	mpc_ref_point.velocity.linear.z =  desired_velocity[2];
	mpc_ref_point.acceleration.linear.x =  desired_acceleration[0];
	mpc_ref_point.acceleration.linear.y =  desired_acceleration[1];
	mpc_ref_point.acceleration.linear.z =  desired_acceleration[2];
	mpc_ref_point.jerk.linear.x =  0;
	mpc_ref_point.jerk.linear.y =  0;
	mpc_ref_point.jerk.linear.z =  0;
	mpc_ref_point.snap.linear.x =  0;
	mpc_ref_point.snap.linear.y =  0;
	mpc_ref_point.snap.linear.z =  0;
	mpc_ref_point.heading = 0;
  	mpc_ref_point.heading_rate = 0;
  	mpc_ref_point.heading_acceleration = 0;

  	MPCTraj.points.push_back(mpc_ref_point);
  }
}

template<typename T>
void MPCFSM<T>::publishPrediction(
    const Eigen::Ref<const Eigen::Matrix<T, kStateSize, kSamples + 1>> predicted_traj,
    ros::Time& time) {
  nav_msgs::Path path_msg;
  path_msg.header.stamp = time;
  path_msg.header.frame_id = "world";
  geometry_msgs::PoseStamped pose;
  const T dt = controller_.mpc_time_step_;
  // std::cout << "pub_prediction" << std:: endl;
  for (int i = 0; i < kSamples+1; i++) {
    pose.header.stamp = time + ros::Duration(i * dt);
    pose.header.seq = i;
    pose.pose.position.x = predicted_traj(kPosX, i);
    pose.pose.position.y = predicted_traj(kPosY, i);
    pose.pose.position.z = predicted_traj(kPosZ, i);
    // std::cout << "iter:" << i << " " << inputs(kThrust1, i) << std::endl;
	// if(i==0){
	// 	ROS_INFO_STREAM("predicted: " << pose.pose.position);
	// }
	Eigen::Vector3d pre_vel(predicted_traj(kVelX, i), predicted_traj(kVelY, i), predicted_traj(kVelZ, i));
	Eigen::Quaterniond q(predicted_traj(kOriW, i), predicted_traj(kOriX, i), predicted_traj(kOriY, i), predicted_traj(kOriZ, i));
	//pub predicted bodyrates
	pose.pose.orientation.w = predicted_traj(kOriW, i);
    pose.pose.orientation.x = predicted_traj(kOriX, i);
    pose.pose.orientation.y = predicted_traj(kOriY, i);
    pose.pose.orientation.z = predicted_traj(kOriZ, i);
	// cout << "siyuanshu: " << pose.pose.orientation.w*pose.pose.orientation.w + pose.pose.orientation.x*pose.pose.orientation.x +
	// 					pose.pose.orientation.y*pose.pose.orientation.y + pose.pose.orientation.z*pose.pose.orientation.z<< endl;
	//pub predicted attitude 
	//
    double pitch_angle_degree = asin(2 * (q.w() * q.y() - q.z() * q.x())) * 180 / 3.1415926;
    double yaw_angle_degree = atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()))
								* 180 / 3.1415926 ;
	double roll_angle_degree = atan2(2 * (q.w() * q.x() + q.y() * q.z()), 1 - 2 * (q.x() * q.x() + q.y() * q.y()));
    // pose.pose.orientation.w = predicted_traj(kOriW, i);
    // pose.pose.orientation.x = roll_angle_degree;
    // pose.pose.orientation.y = pitch_angle_degree;
    // pose.pose.orientation.z = yaw_angle_degree;
 
	//pub predicted vel (convert to body frame, same as odometry)
	pre_vel = q.conjugate() * pre_vel;
    pose.pose.orientation.w = predicted_traj(kOriW, i);
    pose.pose.orientation.x = pre_vel[0];
    pose.pose.orientation.y = pre_vel[1];
    pose.pose.orientation.z = yaw_angle_degree;

	//pub predicted orientation
    // pose.pose.orientation.w = predicted_traj(kOriW, i);
    // pose.pose.orientation.x = predicted_traj(kOriX, i);
    // pose.pose.orientation.y = predicted_traj(kOriY, i);
    // pose.pose.orientation.z = predicted_traj(kOriZ, i);
    path_msg.poses.push_back(pose);
  }

  pub_predicted_trajectory_.publish(path_msg);

}

template<typename T>
void MPCFSM<T>::PublishSpeedControlCommand(
	const Eigen::Ref<const Eigen::Matrix<T, kInputSize, 1>> predicted_input,
	ros::Time& time){
	//pub commmand    
	Eigen::Matrix<T, kInputSize, 1> input_bounded = predicted_input.template cast<T>();
	mav_msgs::Actuators command;

	Eigen::Vector4d rotor_thrusts;
	
	rotor_thrusts << input_bounded(INPUT_THRUST::kThrust1), input_bounded(INPUT_THRUST::kThrust2), 
							input_bounded(INPUT_THRUST::kThrust3), input_bounded(INPUT_THRUST::kThrust4);
	// std::cout << "rotor_thrusts: " << rotor_thrusts << std::endl;
	Eigen::Vector4d rotor_speeds;

	for (int i = 0; i < 4; i++) {  //get rotors speed directly
		double motor_speed_squared =
			rotor_thrusts[i] / params_.rotor_thrust_coeff;
		// quadrotor_common::limit(&motor_speed_squared, 0.0,
		//                         pow(max_rotor_speed_, 2.0));
		// rotor_speeds[i] = sqrt(motor_speed_squared) * 0.8;
		rotor_speeds[i] = sqrt(motor_speed_squared);
	}
	
	
	command.header.stamp = time;
	command.angular_velocities.clear();
	for(int i = 0; i < rotor_speeds.size(); i++){
		command.angular_velocities.push_back(rotor_speeds[i]);
	}
	// std::cout << "rotor_speeds: " << rotor_speeds << std::endl;
	pub_control_command.publish(command);

	//update state predictor
	quadrotor_common::ControlCommand control_cmd;
	control_cmd.armed = true;
	control_cmd.control_mode = quadrotor_common::ControlMode::ROTOR_THRUSTS;
	control_cmd.collective_thrust = (rotor_thrusts[0] + rotor_thrusts[1] + rotor_thrusts[2] + rotor_thrusts[3]) / params_.mass;
	control_cmd.timestamp = ros::Time::now();
	control_cmd.expected_execution_time =
		control_cmd.timestamp + ros::Duration(params_.control_command_delay_);  

	quadrotor_msgs::ControlCommand control_cmd_msg;
	control_cmd_msg = control_cmd.toRosMessage();
	state_predictor_.pushCommandToQueue(control_cmd);
}


template<typename T>
bool MPCFSM<T>::rc_is_received(const ros::Time &now_time)
{
	return (now_time - rc_data.rcv_stamp).toSec() < params_.msg_timeout.rc;
}
template<typename T>
bool MPCFSM<T>::cmd_is_received(const ros::Time &now_time)
{
	//recv mpc traj interval
	return (now_time - trajectory_data.rcv_stamp).toSec() < params_.msg_timeout.cmd;
}
template<typename T>
bool MPCFSM<T>::odom_is_received(const ros::Time &now_time)
{
	return (now_time - odom_data.rcv_stamp).toSec() < params_.msg_timeout.odom;
}
template<typename T>
bool MPCFSM<T>::imu_is_received(const ros::Time &now_time)
{
	return (now_time - imu_data.rcv_stamp).toSec() < params_.msg_timeout.imu;
}
template<typename T>
bool MPCFSM<T>::bat_is_received(const ros::Time &now_time)
{
	return (now_time - bat_data.rcv_stamp).toSec() < params_.msg_timeout.bat;
}
template<typename T>
bool MPCFSM<T>::recv_new_odom()
{
	if (odom_data.rcv_new_msg)
	{
		odom_data.rcv_new_msg = false;
		return true;
	}
	return false;
}


template<typename T>
void MPCFSM<T>::publish_bodyrate_ctrl(const Eigen::Ref<const Eigen::Matrix<T, kInputSize, 1>> predicted_input, 
									const ros::Time &stamp, double collective_thrust_normalized)
{
	mavros_msgs::AttitudeTarget msg;

	msg.header.stamp = stamp;
	msg.header.frame_id = std::string("FCU");

	msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;


	Eigen::Matrix<T, kInputSize, 1> input_bounded = predicted_input.template cast<T>();
	// double collective_thrust;
	Eigen::Vector3d bodyrates;
	
	// collective_thrust = input_bounded(INPUT_BODYRATE::kThrust);
	bodyrates << input_bounded(INPUT_BODYRATE::kRateX), input_bounded(INPUT_BODYRATE::kRateY), 
							input_bounded(INPUT_BODYRATE::kRateZ);

	msg.body_rate.x = bodyrates[0];
	msg.body_rate.y = bodyrates[1];
	msg.body_rate.z = bodyrates[2];

	if(collective_thrust_normalized > 0.7) collective_thrust_normalized = 0.7;
	msg.thrust = collective_thrust_normalized;


	ctrl_FCU_pub.publish(msg);
}


template<typename T>
void MPCFSM<T>::publish_thrust_ctrl(const Eigen::Ref<const Eigen::Matrix<T, kInputSize, 1>> predicted_input, 
									const ros::Time &stamp)
{
	mavros_msgs::AttitudeTarget msg;

	msg.header.stamp = stamp;
	msg.header.frame_id = std::string("FCU");

	msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
					mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
					mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;

	msg.orientation.x = 0.00;
	msg.orientation.y = 0.00;
	msg.orientation.z = 0.00; 
	msg.orientation.w = 1.00;

	Eigen::Matrix<T, kInputSize, 1> input_bounded = predicted_input.template cast<T>();
	Eigen::Vector4d rotor_thrusts;
	
	rotor_thrusts << input_bounded(INPUT_THRUST::kThrust1), input_bounded(INPUT_THRUST::kThrust2), 
							input_bounded(INPUT_THRUST::kThrust3), input_bounded(INPUT_THRUST::kThrust4);

	// rotor_thrusts << 3.98, 3.98, 3.98, 3.98;
	// std::cout << "rotor_thrusts: " << rotor_thrusts << std::endl;
	Eigen::Vector4d rotor_speeds;

	for (int i = 0; i < 4; i++) {  //get rotors speed directly

		// if(rotor_thrusts[i] > 5.6) rotor_thrusts[i] = 5.6;

		double motor_speed_squared =
			rotor_thrusts[i] / params_.rotor_thrust_coeff;
		// quadrotor_common::limit(&motor_speed_squared, 0.0,
		//                         pow(max_rotor_speed_, 2.0));
		rotor_speeds[i] = sqrt(motor_speed_squared);

		// revised by wyz
		double error = rotor_speeds[i] - rpm_data.rpm[i];
		static double last_error = 0; //used to compute D error
		static double inte_error_sum = 0;
		// const int inte_window_size = 100;
		static vector<double> inte_error_list;
		const double Kp = 0.30;
		const double Ki = 0.042;
		const double Kd = 0;
		const double inte_max = 10000;

		double deri_error = error - last_error;

		// inte_error_list.push_back(error);
		inte_error_sum += error;

		if(inte_error_sum > inte_max) inte_error_sum = inte_max;
		if(inte_error_sum < -inte_max) inte_error_sum = -inte_max;
		// if(inte_error_list.size() > inte_window_size){
		// 	vector<double>::iterator k = inte_error_list.begin();
		// 	inte_error_list.erase(k);
		// 	double top_error = inte_error_list.front();
		// 	inte_error_sum -= top_error;
		// }

		rotor_speeds[i] += Kp * error + Kd * deri_error + Ki * inte_error_sum;
		if(rotor_speeds[i] > 15000.0)
		{
			rotor_speeds[i] = 15000.0;
		}

		last_error = error;
	}

	// revised by wyz
	// RPM -> dshot
	// rpm = dshot*29.5 + 1900.00;
	// std::cout << "rotor_speed:  " << rotor_speeds << std::endl;

	Eigen::Vector4d dshot;

	//rpm = p1 * dshot^2 + p2 * dshot + p3 
	// p1 = -0.003329, p2 = 16.25, p3 = -10.06
	double dshot0, dshot1, dshot2, dshot3; 
	const double p1 = -0.003329, p2 = 16.25, p3 = -10.06;
	// std::cout << "rotor_speeds: " << rotor_speeds << std::endl;
	dshot0 = (1 / (2 * p1)) * (-p2 + sqrt(p2 * p2 - 4 * p1 * (p3 - rotor_speeds[0])));
	dshot1 = (1 / (2 * p1)) * (-p2 + sqrt(p2 * p2 - 4 * p1 * (p3 - rotor_speeds[1])));
	dshot2 = (1 / (2 * p1)) * (-p2 + sqrt(p2 * p2 - 4 * p1 * (p3 - rotor_speeds[2])));
	dshot3 = (1 / (2 * p1)) * (-p2 + sqrt(p2 * p2 - 4 * p1 * (p3 - rotor_speeds[3])));
	dshot << dshot0 / 2, dshot1 / 2, dshot2 / 2, dshot3 / 2;   //dshot is 0-2000 originally, but is 0-1000 in the revised firmware
	// std::cout << "dshot: " << dshot*2.0  << std::endl;


	msg.body_rate.x = int(dshot[0]) + dshot[1] / 1000.0;
	msg.body_rate.y = int(dshot[2]) + dshot[3] / 1000.0;
	// for morphing quadrotor;
	// msg.body_rate.z = delta_x_data.current_delta_x.data;

	// if thrust != 0.00, No use thrust ctrl mode;
	// if thrust == 0.00, use thrust ctrl mode;
	msg.thrust = 0.00;

	ctrl_FCU_pub.publish(msg);
}
template<typename T>
void MPCFSM<T>::publish_trigger(const nav_msgs::Odometry &odom_msg)
{
	geometry_msgs::PoseStamped msg;
	msg.header.frame_id = "world";
	msg.pose = odom_msg.pose.pose;

	traj_start_trigger_pub.publish(msg);
}
template<typename T>
bool MPCFSM<T>::toggle_offboard_mode(bool on_off)
{
	mavros_msgs::SetMode offb_set_mode;

	if (on_off)
	{
		state_data.state_before_offboard = state_data.current_state;
		if (state_data.state_before_offboard.mode == "GUIDED_NOGPS") // Not allowed
			state_data.state_before_offboard.mode = "STABILIZE";

		offb_set_mode.request.custom_mode = "GUIDED_NOGPS";
		if (!(set_FCU_mode_srv.call(offb_set_mode) && offb_set_mode.response.mode_sent))
		{
			ROS_ERROR("Enter GUIDED_NOGPS rejected by PX4!");
			return false;
		}
	}
	else
	{
		offb_set_mode.request.custom_mode = state_data.state_before_offboard.mode;
		if (!(set_FCU_mode_srv.call(offb_set_mode) && offb_set_mode.response.mode_sent))
		{
			ROS_ERROR("Exit GUIDED_NOGPS rejected by PX4!");
			return false;
		}
	}

	return true;

	// if (params_.print_dbg)
	// 	printf("offb_set_mode mode_sent=%d(uint8_t)\n", offb_set_mode.response.mode_sent);
}
template<typename T>
bool MPCFSM<T>::toggle_arm_disarm(bool arm)
{
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = arm;
	if (!(arming_client_srv.call(arm_cmd) && arm_cmd.response.success))
	{
		if (arm)
			ROS_ERROR("ARM rejected by PX4!");
		else
			ROS_ERROR("DISARM rejected by PX4!");

		return false;
	}

	return true;
}
template<typename T>
void MPCFSM<T>::reboot_FCU()
{
	// https://mavlink.io/en/messages/common.html, MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN(#246)
	mavros_msgs::CommandLong reboot_srv;
	reboot_srv.request.broadcast = false;
	reboot_srv.request.command = 246; // MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
	reboot_srv.request.param1 = 1;	  // Reboot autopilot
	reboot_srv.request.param2 = 0;	  // Do nothing for onboard computer
	reboot_srv.request.confirmation = true;

	reboot_FCU_srv.call(reboot_srv);

	ROS_INFO("Reboot FCU");

	// if (params_.print_dbg)
	// 	printf("reboot result=%d(uint8_t), success=%d(uint8_t)\n", reboot_srv.response.result, reboot_srv.response.success);
}

template
class MPCFSM<float>;

template
class MPCFSM<double>;

} //namespace rpg_mpc
