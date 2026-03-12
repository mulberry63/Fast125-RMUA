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


#pragma once

#include <thread>

#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <mav_msgs/Actuators.h>
#include "quadrotor_msgs/Trajectory.h"
#include "quadrotor_msgs/TrajectoryPoint.h"
#include "quadrotor_msgs/ControlCommand.h"
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <nav_msgs/Odometry.h>
// #include <state_predictor/state_predictor.h>
#include <quadrotor_common/control_command.h>
#include <quadrotor_common/quad_state_estimate.h>

#include <queue>
#include "rpg_mpc/mpc_wrapper.h"
#include "rpg_mpc/mpc_params.h"

namespace rpg_mpc {

enum STATE {
  kPosX = 0,
  kPosY = 1,
  kPosZ = 2,
  kOriW = 3,
  kOriX = 4,
  kOriY = 5,
  kOriZ = 6,
  kVelX = 7,
  kVelY = 8,
  kVelZ = 9,
};

enum INPUT_THRUST {
  kThrust1 = 0,
  kThrust2 = 1,
  kThrust3 = 2,
  kThrust4 = 3
};

enum INPUT_BODYRATE {
  kThrust = 0,
  kRateX = 1,
  kRateY = 2,
  kRateZ = 3
};

struct TorquesAndThrust {
  Eigen::Vector3d body_torques;
  double collective_thrust;
};

template<typename T>
class MpcController {
public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static_assert(kStateSize == 10,
                "MpcController: Wrong model size. Number of states does not match.");
  static_assert(kInputSize == 4,
                "MpcController: Wrong model size. Number of inputs does not match.");

  MpcController(MpcParams<T> &params);

  // MpcController() : MpcController(ros::NodeHandle()) {}

  // MpcController() : MpcController(ros::NodeHandle()) {};

  void execMPC(const quadrotor_msgs::Trajectory &reference_trajectory, 
               const Eigen::Matrix<T, kStateSize, 1> &estimated_state,
               Eigen::Matrix<T, kStateSize, kSamples + 1> &predicted_states,
               Eigen::Matrix<T, kInputSize, kSamples> &predicted_inputs,
               double &collective_thrust);

  T mpc_time_step_;
  std::queue<std::pair<ros::Time, double>> timed_thrust;
  // Thrust-accel mapping params
	double thr_scale_compensate;
	const double rho2 = 0.998; // do not change
	double thr2acc;
	double P;

  void resetThrustMapping(void);

  bool estimateThrustModel(
  const Eigen::Vector3d &est_a,
  const double voltage,
  const Eigen::Vector3d &est_v,
  const MpcParams<T> &param);

private:
  // Internal helper functions.

  // void offCallback(const std_msgs::Empty::ConstPtr& msg);

  // bool setStateEstimate(
  //     const quadrotor_common::QuadStateEstimate& state_estimate);

  Eigen::Vector4d thrust_mixer(const TorquesAndThrust& torques_and_thrust);

  Eigen::Quaterniond computeDesiredAttitude(
    const Eigen::Vector3d& desired_acceleration, const double reference_heading);

  quadrotor_msgs::ControlCommand computeReferenceInputs(
    const quadrotor_msgs::TrajectoryPoint& reference_state,
    const Eigen::Matrix<T, kStateSize, 1>& estimated_state);

  Eigen::Vector3d computeRobustBodyXAxis(
    const Eigen::Vector3d &x_B_prototype, const Eigen::Vector3d &x_C,
    const Eigen::Vector3d &y_C,
    const Eigen::Quaterniond &est_q) const; //jxlin: new add
  bool setReference(const quadrotor_msgs::Trajectory& reference_trajectory,
                    const Eigen::Matrix<T, kStateSize, 1> &estimated_state);

  bool almostZero(const double value) const;
  bool almostZeroThrust(const double thrust_value) const;




  /****** move callback to main function ******/
  // void PublishThrustControlCommand(
  //     const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state,
  //     const Eigen::Ref<const Eigen::Matrix<T, kInputSize, 1>> input,
  //     ros::Time& time);

  // void PublishSpeedControlCommand(
  //     const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state,
  //     const Eigen::Ref<const Eigen::Matrix<T, kInputSize, 1>> input,
  //     ros::Time& time);

  // bool publishPrediction(
  //     const Eigen::Ref<const Eigen::Matrix<T, kStateSize, kSamples + 1>> states,
  //     const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kSamples>> inputs,
  //     ros::Time& time);

  // void odometryCallback(const nav_msgs::OdometryConstPtr& msg);

  // void trajCallback(const quadrotor_msgs::TrajectoryConstPtr& msg);

  // void pubCmdCallback(const ros::TimerEvent& e);

  void preparationThread();

  bool setNewParams(MpcParams<T>& params);

  // Handles (move to main function)
  // ros::NodeHandle nh_;
  // ros::NodeHandle pnh_;

  // Subscribers and publisher. (move to main function)
  // ros::Subscriber odom_sub, traj_sub;
  // ros::Publisher pub_control_command, pub_predicted_trajectory_;
  // ros::Timer exec_timer_, pub_cmd_timer_;

  // Parameters
  MpcParams<T> params_;
  
  Eigen::Matrix3d inertia_;

  // MPC
  MpcWrapper<T> mpc_wrapper_;

  // Preparation Thread
  std::thread preparation_thread_;
  double odom_yaw_;
  bool quaternion_norm_ok_;
  bool odom_ok_;
  // quadrotor_msgs::Trajectory reference_trajectory_;
  bool first_traj_received_;
  // Constants
  static constexpr double kMinNormalizedCollectiveThrust_ = 1.0;
  static constexpr double kAlmostZeroValueThreshold_ = 0.001;
  static constexpr double kAlmostZeroThrustThreshold_ = 0.01;
  // state_predictor::StatePredictor state_predictor_;

  Eigen::Vector4d rotor_thrust_cmd_;
  // quadrotor_common::QuadStateEstimate received_state_est_;

  // Variables
  T timing_feedback_, timing_preparation_;
  bool solve_from_scratch_;
  // Eigen::Matrix<T, kStateSize, 1> est_state_;
  Eigen::Matrix<T, kStateSize, kSamples + 1> reference_states_;
  Eigen::Matrix<T, kInputSize, kSamples + 1> reference_inputs_;
};


} // namespace MPC
