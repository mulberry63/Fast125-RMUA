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


#include "rpg_mpc/mpc_controller.h"
#include <nav_msgs/Odometry.h>
#include <ctime>

namespace rpg_mpc {

template<typename T>
MpcController<T>::MpcController(MpcParams<T> &params) :
    params_(params),
    mpc_wrapper_(MpcWrapper<T>()),
    reference_states_(Eigen::Matrix<T, kStateSize, kSamples + 1>::Zero()),
    reference_inputs_(Eigen::Matrix<T, kInputSize, kSamples + 1>::Zero()),
    timing_feedback_(T(1e-3)),
    timing_preparation_(T(1e-3)){
  // std::cout << "fkhfk" << std::endl;
  // if (!params_.loadParameters(pnh_)) {
  //   ROS_ERROR("[%s] Could not load parameters.", pnh_.getNamespace().c_str());
  //   ros::shutdown();
  //   return;
  // }

  inertia_ = Eigen::Matrix3d::Zero();

  inertia_(0, 0) = params_.inertia_x; 
  inertia_(1, 1) = params_.inertia_y; 
  inertia_(2, 2) = params_.inertia_z; 
  // std::cout << "aaaaaa" << std::endl;
  setNewParams(params_);
  // mpc_wrapper_setDynamicParams(params_.mass);
  // std::cout << "bbbbb" << std::endl;

  rotor_thrust_cmd_.setZero();

  first_traj_received_ = false;
  solve_from_scratch_ = true;
  preparation_thread_ = std::thread(&MpcWrapper<T>::prepare, mpc_wrapper_);
}
  
template<typename T>
void MpcController<T>::execMPC(const quadrotor_msgs::Trajectory &reference_trajectory, 
               const Eigen::Matrix<T, kStateSize, 1> &estimated_state,
               Eigen::Matrix<T, kStateSize, kSamples + 1> &predicted_states,
               Eigen::Matrix<T, kInputSize, kSamples> &predicted_inputs,
               double &collective_thrust) {
  static ros::Time mpc_start_time = ros::Time::now();
  ros::Time call_time = ros::Time::now();
  const clock_t start = clock();
  clock_t alg_start = clock();


  // if(! first_traj_received_) return;
  if (reference_trajectory.points.size() == 0){
    std::cout << "Empty Traj!" << std::endl;
    return; 
  }

  setNewParams(params_);
  // if (params.changed_) {
  //   params_ = params;
  //   setNewParams(params_);
  // }

  // ROS_ERROR("run_mpc");
  //if there is no ref trajectory, then hover at yuandi
  // bool no_traj = false;



  preparation_thread_.join();

  setReference(reference_trajectory, estimated_state);

  static const bool do_preparation_step(false);

  // Get the feedback from MPC.
  
  // ROS_INFO_STREAM("ref_state: " << std::endl << reference_states_);
  // ROS_INFO_STREAM("ref_input: " << std::endl << reference_inputs_);
  // ROS_INFO_STREAM("est_state_: " << std::endl << estimated_state);

  mpc_wrapper_.setTrajectory(reference_states_, reference_inputs_);

  static bool change_solve_scratch = true;
  // if(solve_state && change_solve_scratch) {
  //   solve_from_scratch_ = true;
  //   change_solve_scratch = false;
  // }
  
  if (0) {
    // ROS_INFO("Solving MPC with hover as initial guess.");
    // ROS_INFO_STREAM("state!: " << est_state_);
    
    // if((call_time-mpc_start_time).toSec() > 7){
    //   // std::cout << "asdfasdf " << std::endl;
    //   mpc_wrapper_.solve(reference_states_.col(0));
    // }
    // else 

    //ZRB warning 2022.6.10
    // if(!solve_state)
      // mpc_wrapper_.solve(estimated_state);
    // else
      mpc_wrapper_.solve(estimated_state);
    solve_from_scratch_ = false;
  } else {
        
    // if((call_time-mpc_start_time).toSec() > 7){
    //   // ROS_INFO_STREAM("asdfasdf" << reference_states_.col(0));
    //   mpc_wrapper_.update(reference_states_.col(0), do_preparation_step);
    // }
    // else estimate_state

    //ZRB warning 2022.6.10
    // if(!solve_state)
      // mpc_wrapper_.update(estimated_state, do_preparation_step);
    // else {
    //       // ROS_INFO_STREAM("asdfasdf" << reference_states_.col(0));
          mpc_wrapper_.update(estimated_state, do_preparation_step);
    // }
      
  }
  mpc_wrapper_.getStates(predicted_states);
  mpc_wrapper_.getInputs(predicted_inputs);
  // std::cout<<"predicted_states\n"<<predicted_states<<std::endl;
  // std::cout<<"predicted_inputs\n"<<predicted_inputs<<std::endl;

  // Publish the predicted trajectory.
  // publishPrediction(predicted_states_, predicted_inputs_, call_time);

  // Start a thread to prepare for the next execution.
  preparation_thread_ = std::thread(&MpcController<T>::preparationThread, this);

  // Timing
  const clock_t end = clock();
  timing_feedback_ = 0.9 * timing_feedback_ +
                     0.1 * double(end - start) / CLOCKS_PER_SEC;
  // if (params_.print_info_)
  ROS_INFO_THROTTLE(1.0, "MPC Timing: Latency: %1.1f ms  |  Total: %1.1f ms",
                      timing_feedback_ * 1000, (timing_feedback_ + timing_preparation_) * 1000);

  //thrust calibration
  Eigen::Matrix<T, kInputSize, 1> mpc_input = predicted_inputs.col(0);
  // std::cout << "thr2acc: " << thr2acc << std::endl;
  // std::cout << "thrust: " << mpc_input[0] << std::endl;
  collective_thrust = mpc_input[0] / thr2acc;

  timed_thrust.push(std::pair<ros::Time, double>(ros::Time::now(), collective_thrust));
  while (timed_thrust.size() > 100)
    timed_thrust.pop();
}


template<typename T>
Eigen::Quaterniond MpcController<T>::computeDesiredAttitude(
    const Eigen::Vector3d& desired_acceleration, const double reference_heading){
  const Eigen::Quaterniond q_heading = Eigen::Quaterniond(
      Eigen::AngleAxisd(reference_heading, Eigen::Vector3d::UnitZ()));

  // Compute desired orientation
  const Eigen::Vector3d x_C = q_heading * Eigen::Vector3d::UnitX();
  const Eigen::Vector3d y_C = q_heading * Eigen::Vector3d::UnitY();

  Eigen::Vector3d z_B;
  z_B = desired_acceleration.normalized();

  const Eigen::Vector3d x_B_prototype = y_C.cross(z_B);
  const Eigen::Vector3d x_B = x_B_prototype.normalized();

  const Eigen::Vector3d y_B = (z_B.cross(x_B)).normalized();

  // From the computed desired body axes we can now compose a desired attitude
  const Eigen::Matrix3d R_W_B((Eigen::Matrix3d() << x_B, y_B, z_B).finished());

  const Eigen::Quaterniond desired_attitude(R_W_B);

  return desired_attitude;
}

template<typename T>
bool MpcController<T>::almostZero(const double value) const {
  return fabs(value) < kAlmostZeroValueThreshold_;
}

template<typename T>
bool MpcController<T>::almostZeroThrust(const double thrust_value) const {
  return fabs(thrust_value) < kAlmostZeroThrustThreshold_;
}
template<typename T>
Eigen::Vector3d MpcController<T>::computeRobustBodyXAxis(
    const Eigen::Vector3d &x_B_prototype, const Eigen::Vector3d &x_C,
    const Eigen::Vector3d &y_C,
    const Eigen::Quaterniond &est_q) const
{
  Eigen::Vector3d x_B = x_B_prototype;

  // cout << "x_B.norm()=" << x_B.norm() << endl;

  if (almostZero(x_B.norm()))
  {
    // if cross(y_C, z_B) == 0, they are collinear =>
    // every x_B lies automatically in the x_C - z_C plane

    // Project estimated body x-axis into the x_C - z_C plane
    const Eigen::Vector3d x_B_estimated =
        est_q * Eigen::Vector3d::UnitX();
    const Eigen::Vector3d x_B_projected =
        x_B_estimated - (x_B_estimated.dot(y_C)) * y_C;
    if (almostZero(x_B_projected.norm()))
    {
      // Not too much intelligent stuff we can do in this case but it should
      // basically never occur
      x_B = x_C;
    }
    else
    {
      x_B = x_B_projected.normalized();
    }
  }
  else
  {
    x_B.normalize();
  }

  // if the quad is upside down, x_B will point in the "opposite" direction
  // of x_C => flip x_B (unfortunately also not the solution for our problems)
  if (x_B.dot(x_C) < 0.0)
  {
    x_B = -x_B;
    // std::cout << "CCCCCCCCCCCCCC" << std::endl;
  }

  return x_B;
}
template<typename T>
quadrotor_msgs::ControlCommand MpcController<T>::computeReferenceInputs(
    const quadrotor_msgs::TrajectoryPoint& reference_state,const Eigen::Matrix<T, kStateSize, 1>& estimated_state){

  quadrotor_msgs::ControlCommand reference_command;

  const double dx = 0.26;
  const double dy = 0.28;
  const double dz = 0.42;
  
  const Eigen::Quaterniond q_heading = Eigen::Quaterniond(
      Eigen::AngleAxisd(reference_state.heading, Eigen::Vector3d::UnitZ())); //jxlin: UnitZ: 0 0 1  Eigen::AngleAxisd(参数：角度，轴)

  const Eigen::Vector3d x_C = q_heading * Eigen::Vector3d::UnitX(); //jxlin: UnitX: 1 0 0
  const Eigen::Vector3d y_C = q_heading * Eigen::Vector3d::UnitY(); //jxlin: UnitY: 0 1 0

  const Eigen::Vector3d gravity(0.0, 0.0, -9.81);

  Eigen::Vector3d ref_vel(reference_state.velocity.linear.x, reference_state.velocity.linear.y, 
                          reference_state.velocity.linear.z);
  Eigen::Vector3d ref_acc(reference_state.acceleration.linear.x, reference_state.acceleration.linear.y, 
                          reference_state.acceleration.linear.z);
  Eigen::Vector3d ref_jer(reference_state.jerk.linear.x, reference_state.jerk.linear.y, 
                          reference_state.jerk.linear.z);
  Eigen::Vector3d ref_sna(reference_state.snap.linear.x, reference_state.snap.linear.y, 
                          reference_state.snap.linear.z);
  // Eigen::Quaterniond ref_ori = Eigen::Quaterniond(reference_state.pose.orientation.w, reference_state.pose.orientation.x, 
  //                                                 reference_state.pose.orientation.y, reference_state.pose.orientation.z);
  Eigen::Quaterniond odom_q = Eigen::Quaterniond(estimated_state(kOriW), estimated_state(kOriX), estimated_state(kOriY), estimated_state(kOriZ));
  const double ref_yaw_rate = reference_state.heading_rate;
  const double ref_yaw_acceleration = reference_state.heading_acceleration;
            

  const Eigen::Vector3d alpha = ref_acc - gravity + dx * ref_vel;
  const Eigen::Vector3d beta = ref_acc - gravity + dy * ref_vel;
  const Eigen::Vector3d gamma = ref_acc - gravity + dz * ref_vel;

  // Reference attitude
  const Eigen::Vector3d x_B_prototype = y_C.cross(alpha);
  const Eigen::Vector3d x_B = computeRobustBodyXAxis(x_B_prototype, x_C, y_C, odom_q);

  Eigen::Vector3d y_B = beta.cross(x_B);
  if (almostZero(y_B.norm()))
  {
    const Eigen::Vector3d z_B_estimated =
        odom_q * Eigen::Vector3d::UnitZ();
    y_B = z_B_estimated.cross(x_B);
    if (almostZero(y_B.norm()))
    {
      y_B = y_C;
    }
    else
    {
      y_B.normalize();
    }
  }
  else
  {
    y_B.normalize();
  }

  const Eigen::Vector3d z_B = x_B.cross(y_B);

  const Eigen::Matrix3d R_W_B_ref((Eigen::Matrix3d() << x_B, y_B, z_B).finished());

  const Eigen::Quaterniond q_W_B = Eigen::Quaterniond(R_W_B_ref);

  reference_command.orientation.w = q_W_B.w();
  reference_command.orientation.x = q_W_B.x();
  reference_command.orientation.y = q_W_B.y();
  reference_command.orientation.z = q_W_B.z();

  // Reference thrust
  reference_command.collective_thrust = z_B.dot(gamma);

  // Rotor drag matrix
  const Eigen::Matrix3d D = Eigen::Vector3d(dx, dy, dz).asDiagonal();

  // Reference body rates
  const double B1 = reference_command.collective_thrust -
                    (dz - dx) * z_B.dot(ref_vel);
  const double C1 = -(dx - dy) * y_B.dot(ref_vel);
  const double D1 = x_B.dot(ref_jer) + dx * x_B.dot(ref_acc);
  const double A2 = reference_command.collective_thrust +
                    (dy - dz) * z_B.dot(ref_vel);
  const double C2 = (dx - dy) * x_B.dot(ref_vel);
  const double D2 = -y_B.dot(ref_jer) -
                    dy * y_B.dot(ref_acc);
  const double B3 = -y_C.dot(z_B);
  const double C3 = (y_C.cross(z_B)).norm();
  const double D3 = ref_yaw_rate * x_C.dot(x_B);

  const double denominator = B1 * C3 - B3 * C1;

  if (almostZero(denominator))
  {
    reference_command.bodyrates.x = 0.0;
    reference_command.bodyrates.y = 0.0;
    reference_command.bodyrates.z = 0.0;
  }
  else
  {
    // Compute body rates
    if (almostZero(A2))
    {
      reference_command.bodyrates.x = 0.0;
    }
    else
    {
      reference_command.bodyrates.x =
          (-B1 * C2 * D3 + B1 * C3 * D2 - B3 * C1 * D2 + B3 * C2 * D1) /
          (A2 * denominator);
    }
    reference_command.bodyrates.y = (-C1 * D3 + C3 * D1) / denominator;
    reference_command.bodyrates.z = (B1 * D3 - B3 * D1) / denominator;
  }
  // Reference angular accelerations
  Eigen::Matrix3d bodyrates_skew_symmetric;
  bodyrates_skew_symmetric << 0, -reference_command.bodyrates.z, reference_command.bodyrates.y,
                              reference_command.bodyrates.z, 0, -reference_command.bodyrates.x,
                              -reference_command.bodyrates.y, reference_command.bodyrates.x, 0;

  const double thrust_deri = z_B.dot(ref_jer) + reference_command.bodyrates.x * (dy - dz) * y_B.dot(ref_vel) + 
                                      reference_command.bodyrates.y * (dz - dx) * x_B.dot(ref_vel) + dz * z_B.dot(ref_acc);
  const Eigen::Vector3d epsilon = R_W_B_ref * (bodyrates_skew_symmetric * bodyrates_skew_symmetric * D + D * bodyrates_skew_symmetric * bodyrates_skew_symmetric + 
                         2 * bodyrates_skew_symmetric * D * bodyrates_skew_symmetric.transpose()) * R_W_B_ref.transpose() * ref_vel + 
                         2 * R_W_B_ref * (bodyrates_skew_symmetric * D + D * bodyrates_skew_symmetric.transpose()) * R_W_B_ref.transpose() * ref_acc +
                         R_W_B_ref * D * R_W_B_ref.transpose() * ref_jer;
  const double eps1 = x_B.dot(ref_sna) - 2.0 * thrust_deri * reference_command.bodyrates.y - 
                      reference_command.collective_thrust * reference_command.bodyrates.x * reference_command.bodyrates.z + 
                      x_B.dot(epsilon);

  const double eps2 = -y_B.dot(ref_sna) - 2.0 * thrust_deri * reference_command.bodyrates.x + 
                      reference_command.collective_thrust * reference_command.bodyrates.y * reference_command.bodyrates.z - 
                      y_B.dot(epsilon);
  const double eps3 = ref_yaw_acceleration * x_C.dot(x_B) + 2.0 * ref_yaw_rate * reference_command.bodyrates.z * x_C.dot(y_B) - 
                      2.0 * ref_yaw_rate * reference_command.bodyrates.y * x_C.dot(z_B) - 
                      reference_command.bodyrates.x * reference_command.bodyrates.y * y_C.dot(y_B) - 
                      reference_command.bodyrates.x * reference_command.bodyrates.z * y_C.dot(z_B);
  if (almostZero(denominator))
  {
    reference_command.angular_accelerations.x = 0.0;
    reference_command.angular_accelerations.y = 0.0;
    reference_command.angular_accelerations.z = 0.0;
  }
  else
  {
    // Compute body rates
    if (almostZero(A2))
    {
      reference_command.angular_accelerations.x = 0.0;
    }
    else
    {
      reference_command.angular_accelerations.x =
          (-B1 * C2 * eps3 + B1 * C3 * eps2 - B3 * C1 * eps2 + B3 * C2 * eps1) /
          (A2 * denominator);
    }
    reference_command.angular_accelerations.y = (-C1 * eps3 + C3 * eps1) / denominator;
    reference_command.angular_accelerations.z = (B1 * eps3 - B3 * eps1) / denominator;
  }
  return reference_command;
}

template<typename T>
Eigen::Vector4d MpcController<T>::thrust_mixer(const TorquesAndThrust& torques_and_thrust){

  Eigen::Vector4d des_rotor_speed_;
  Eigen::Vector4d des_rotor_thrust;
  for (int i = 0; i < 4; i++) {
    des_rotor_speed_[i] = 0.0;
    des_rotor_thrust[i] = 0.0;
  }


  Eigen::Matrix4d control_allocation;

  control_allocation = (Eigen::Matrix4d() <<
      params_.mass/4.0, -sqrt(2) / (4.0 * params_.arm_length),  -sqrt(2) / (4.0 * params_.arm_length),    -1/(4.0*params_.rotor_drag_coeff),
      params_.mass/4.0,  sqrt(2) / (4.0 * params_.arm_length),   sqrt(2) / (4.0 * params_.arm_length),    -1/(4.0*params_.rotor_drag_coeff),
      params_.mass/4.0,  sqrt(2) / (4.0 * params_.arm_length),  -sqrt(2) / (4.0 * params_.arm_length),     1/(4.0*params_.rotor_drag_coeff),
      params_.mass/4.0, -sqrt(2) / (4.0 * params_.arm_length),   sqrt(2) / (4.0 * params_.arm_length),     1/(4.0*params_.rotor_drag_coeff)).finished();
  Eigen::Vector4d thrust_and_torque;
  thrust_and_torque << torques_and_thrust.collective_thrust, torques_and_thrust.body_torques.x(),
                       torques_and_thrust.body_torques.y(), torques_and_thrust.body_torques.z();
  // std::cout << "thrust_and_torque: " << thrust_and_torque << std::endl;
  des_rotor_thrust = control_allocation * thrust_and_torque;

  // std::cout << "params_.arm_length" << params_.arm_length << std::endl;
  // std::cout << "params_.rotor_drag_coeff" << params_.rotor_drag_coeff << std::endl;
  // std::cout << "params_.mass" << params_.mass << std::endl;
  // std::cout << "params_.rotor_thrust_coeff" << params_.rotor_thrust_coeff << std::endl;
  //Compute desired rotor_thrust (with mass)
  // for (int i = 0; i < 4; i++) {
  //   des_rotor_thrust[i] = params_.rotor_thrust_coeff * des_rotor_speed_[i];
  // }

  // TODO: Implement RPG saturation from PX4FMU
  //ZRB:  saturation is not implemented

  // // Apply limits and take square root
  // for (int i = 0; i < 4; i++) {
  //   quadrotor_common::limit(&rotor_speed_cmds.angular_velocities[i], 0.0,
  //                           pow(max_rotor_speed_, 2.0));
  //   rotor_speed_cmds.angular_velocities[i] =
  //       sqrt(rotor_speed_cmds.angular_velocities[i]);
  // }

  return des_rotor_thrust;
}

template<typename T>
bool MpcController<T>::setReference(
    const quadrotor_msgs::Trajectory& reference_trajectory,
    const Eigen::Matrix<T, kStateSize, 1> &estimated_state) {
  reference_states_.setZero();
  reference_inputs_.setZero();

  const T dt = mpc_wrapper_.getTimestep();
  mpc_time_step_ = dt;
  
  Eigen::Matrix<T, 3, 1> acceleration;
  const Eigen::Matrix<T, 3, 1> gravity(0.0, 0.0, -9.81);
  Eigen::Quaternion<T> q_heading;
  Eigen::Quaternion<T> q_orientation;
  bool quaternion_norm_ok(true);

  if (reference_trajectory.points.size() == 1) {
    //only one point
    q_heading = Eigen::Quaternion<T>(Eigen::AngleAxis<T>(
        reference_trajectory.points.front().heading,
        Eigen::Matrix<T, 3, 1>::UnitZ()));
    

    //get thrust from reference_state
    quadrotor_msgs::ControlCommand reference_inputs;
    reference_inputs = computeReferenceInputs(reference_trajectory.points.front(),estimated_state);

    Eigen::Quaternion<T> ref_orientation(reference_inputs.orientation.w, reference_inputs.orientation.x, 
                                         reference_inputs.orientation.y, reference_inputs.orientation.z);
    q_orientation = q_heading * ref_orientation;
    const Eigen::Matrix<T, 3, 1> reference_pos(reference_trajectory.points.front().pose.position.x,
                                   reference_trajectory.points.front().pose.position.y,
                                   reference_trajectory.points.front().pose.position.z);
    const Eigen::Matrix<T, 3, 1> reference_vel(reference_trajectory.points.front().velocity.linear.x,
                                  reference_trajectory.points.front().velocity.linear.y,
                                  reference_trajectory.points.front().velocity.linear.z);
    const Eigen::Matrix<T, 3, 1> reference_bodyrates(reference_inputs.bodyrates.x + 0, reference_inputs.bodyrates.y + 0,
                                                    reference_inputs.bodyrates.z + 0);
    ///calculate desired bodyrate and thrust from the trajectory
    // std::cout << "ref_pos: " << reference_pos << std::endl;
    // std::cout << "ref_q_orientation: " << q_orientation.w() << " " << q_orientation.x() << " " << q_orientation.y() << " " << 
    // q_orientation.z()<< std::endl;
    // std::cout << "ref_vel: " << reference_vel << std::endl;
    // std::cout << "ref_bodyrates: " << reference_bodyrates << std::endl;
    
    reference_states_ = (Eigen::Matrix<T, kStateSize, 1>()
        <<reference_pos,
          q_orientation.w(),
          q_orientation.x(),
          q_orientation.y(),
          q_orientation.z(),
          reference_vel
    ).finished().replicate(1, kSamples + 1);


    Eigen::Vector3d body_torques;
    //get torques
    Eigen::Vector3d reference_angular_accelerations(reference_inputs.angular_accelerations.x, 
                                                    reference_inputs.angular_accelerations.y,
                                                    reference_inputs.angular_accelerations.z);
    Eigen::Vector3d ref_bodyrates(reference_inputs.bodyrates.x, 
                                  reference_inputs.bodyrates.y,
                                  reference_inputs.bodyrates.z);
    body_torques = inertia_ * reference_angular_accelerations + 
                   ref_bodyrates.cross(inertia_ * ref_bodyrates);

    TorquesAndThrust torques_and_thrust;
    torques_and_thrust.body_torques = body_torques;
    torques_and_thrust.collective_thrust = reference_inputs.collective_thrust;
    Eigen::VectorXd rotor_thrust = thrust_mixer(torques_and_thrust);

    // std::cout << "ref_collective_thrust: " << torques_and_thrust.collective_thrust << std::endl;
    // std::cout << "ref_body_torques: " << body_torques << std::endl;
    // std::cout << "ref_rotor_thrust: " << rotor_thrust << std::endl;

    reference_inputs_ = (Eigen::Matrix<T, kInputSize, 1>() << reference_inputs.collective_thrust, 
    reference_inputs.bodyrates.x, reference_inputs.bodyrates.y, reference_inputs.bodyrates.z
    ).finished().replicate(1, kSamples + 1);
  } 
  
  else {

    //a trajectory with Ksamples+1 points
    auto iterator(reference_trajectory.points.begin());
    ros::Duration t_start = reference_trajectory.points.begin()->time_from_start;
    auto last_element = reference_trajectory.points.end();
    last_element = std::prev(last_element);

    for (int i = 0; i < kSamples + 1; i++) {
      // while ((iterator->time_from_start - t_start).toSec() <= (i * dt + 1e-4) &&
      //        iterator != last_element) {
      //   iterator++;
      // }

    // std::cout << "iter time!" << (iterator->time_from_start - t_start).toSec() << std::endl;
    quadrotor_msgs::TrajectoryPoint iter_traj_point = *iterator;

    //get thrust from reference_state
    quadrotor_msgs::ControlCommand reference_inputs;
    reference_inputs = computeReferenceInputs(iter_traj_point,estimated_state);

    q_heading = Eigen::Quaternion<T>(Eigen::AngleAxis<T>(
        iterator->heading, Eigen::Matrix<T, 3, 1>::UnitZ()));
    Eigen::Quaternion<T> ref_orientation(reference_inputs.orientation.w, reference_inputs.orientation.x, 
                                        reference_inputs.orientation.y, reference_inputs.orientation.z);
    


    // q_orientation = q_heading * ref_orientation;
    q_orientation = ref_orientation;
    const Eigen::Matrix<T, 3, 1> reference_pos(iterator->pose.position.x,
                                        iterator->pose.position.y,
                                        iterator->pose.position.z);
    const Eigen::Matrix<T, 3, 1> reference_vel(iterator->velocity.linear.x,
                                        iterator->velocity.linear.y,
                                        iterator->velocity.linear.z);
    const Eigen::Matrix<T, 3, 1> reference_bodyrates(reference_inputs.bodyrates.x, reference_inputs.bodyrates.y,
                                                    reference_inputs.bodyrates.z);
    ///calculate desired bodyrate and thrust from the trajectory
    // std::cout << "ref_pos: " << reference_pos << std::endl;
    // std::cout << "ref_q_orientation: " << q_orientation.w() << " " << q_orientation.x() << " " << q_orientation.y() << " " << 
    // q_orientation.z()<< std::endl;
    // std::cout << "ref_vel: " << reference_vel << std::endl;
    // std::cout << "ref_bodyrates: " << reference_bodyrates << std::endl;

    // double yaw_angle_degree = atan2(2 * (q_orientation.w() * q_orientation.z() + q_orientation.x() * q_orientation.y()), 
    // 1 - 2 * (q_orientation.y() * q_orientation.y() + q_orientation.z() * q_orientation.z()))
		// 						* 180 / 3.1415926 ;
    // std::cout << "out_yaw_angle_degree: " << yaw_angle_degree << std::endl;

      reference_states_.col(i) 
      <<  reference_pos,
          q_orientation.w(),
          q_orientation.x(),
          q_orientation.y(),
          q_orientation.z(),
          reference_vel;
      if (reference_states_.col(i).segment(kOriW, 4).dot(
          estimated_state.segment(kOriW, 4)) < 0.0){
            reference_states_.block(kOriW, i, 4, 1) =
            -reference_states_.block(kOriW, i, 4, 1);
          }


      // acceleration << iterator->acceleration.template cast<T>() - gravity;

      Eigen::Vector3d body_torques;
      //get torques
    Eigen::Vector3d reference_angular_accelerations(reference_inputs.angular_accelerations.x, 
                                                    reference_inputs.angular_accelerations.y,
                                                    reference_inputs.angular_accelerations.z);
    Eigen::Vector3d ref_bodyrates(reference_inputs.bodyrates.x, 
                                  reference_inputs.bodyrates.y,
                                  reference_inputs.bodyrates.z);
      body_torques = inertia_ * reference_angular_accelerations + 
                    ref_bodyrates.cross(inertia_ * ref_bodyrates);

      TorquesAndThrust torques_and_thrust;
      torques_and_thrust.body_torques = body_torques;
      torques_and_thrust.collective_thrust = reference_inputs.collective_thrust;
      Eigen::VectorXd rotor_thrust = thrust_mixer(torques_and_thrust);


    double yaw_angle_degree = atan2(2 * (q_orientation.w() * q_orientation.z() + q_orientation.x() * q_orientation.y()), 
    1 - 2 * (q_orientation.y() * q_orientation.y() + q_orientation.z() * q_orientation.z()))
								* 180 / 3.1415926 ;
    double roll_angle_degree = atan2(2 * (q_orientation.w() * q_orientation.x() + q_orientation.z() * q_orientation.y()), 
    1 - 2 * (q_orientation.y() * q_orientation.y() + q_orientation.x() * q_orientation.x()))
								* 180 / 3.1415926 ;
    double pitch_angle_degree = asin(2*(q_orientation.w()*q_orientation.y()-q_orientation.z()*q_orientation.x()))
								* 180 / 3.1415926 ;


    // std::cout << "********************* iter:" << i <<std::endl;
    // std::cout << "ref_collective_thrust: " << torques_and_thrust.collective_thrust << std::endl;
    // std::cout << "ref_body_torques: " << body_torques << std::endl;
    // std::cout << "ref_rotor_thrust: " << rotor_thrust << std::endl;
    // std::cout << "ref_quaternion: " << q_orientation.w() << " " << q_orientation.x() << " " 
    // << q_orientation.y() << " " << q_orientation.z() <<   std::endl;
    // // ROS_INFO_STREAM("ref_quaternion: " << q_orientation );
    // std::cout << "yaw_angle_degree: " << yaw_angle_degree << std::endl;
    // std::cout << "roll_angle_degree: " << roll_angle_degree << std::endl;
    // std::cout << "pitch_angle_degree: " << pitch_angle_degree << std::endl;

      // add input reference
      reference_inputs_.col(i) << reference_inputs.collective_thrust,
                                  reference_inputs.bodyrates.x, reference_inputs.bodyrates.y, reference_inputs.bodyrates.z;

      // add no input reference
      // reference_inputs_.col(i) << 0, 0, 0, 0;
      quaternion_norm_ok &= abs(estimated_state.segment(kOriW, 4).norm() - 1.0) < 0.1;

      iterator++;
    }
  }
  return quaternion_norm_ok;
}

template<typename T>
void MpcController<T>::preparationThread() {
  const clock_t start = clock();

  mpc_wrapper_.prepare();

  // Timing
  const clock_t end = clock();
  timing_preparation_ = 0.9 * timing_preparation_ +
                        0.1 * double(end - start) / CLOCKS_PER_SEC;
}

template<typename T>
bool MpcController<T>::setNewParams(MpcParams<T>& params) {
  mpc_wrapper_.setCosts(params.Q_, params.R_);
  // std::cout << "param_thrust!" << params.min_thrust << "params.max_thrust:" << params.max_thrust << std::endl;
  mpc_wrapper_.setLimits(
      params.min_thrust, params.max_thrust,
      params.max_bodyrate_xy, params.max_bodyrate_z);
  params.changed_ = false;
  return true;
}

template<typename T>
bool MpcController<T>::estimateThrustModel(
    const Eigen::Vector3d &est_a,
    const double voltage,
    const Eigen::Vector3d &est_v,
    const MpcParams<T> &param)
{

  ros::Time t_now = ros::Time::now();
  while (timed_thrust.size() >= 1)
  {
    // Choose data before 35~45ms ago
    std::pair<ros::Time, double> t_t = timed_thrust.front();
    double time_passed = (t_now - t_t.first).toSec();
    if (time_passed > 0.045) // 45ms
    {
      // printf("continue, time_passed=%f\n", time_passed);
      timed_thrust.pop();
      continue;
    }
    if (time_passed < 0.035) // 35ms
    {
      // printf("skip, time_passed=%f\n", time_passed);
      return false;
    }

    /***********************************************************/
    /* Recursive least squares algorithm with vanishing memory */
    /***********************************************************/
    // double thr = t_t.second;
    // timed_thrust.pop();

    // double gamma = 1 / (rho2 + thr * P * thr);
    // double K = gamma * P * thr;
    // std::cout << "thr" << thr << std::endl;
    // std::cout << "P" << P << std::endl;
    // std::cout << "rho2" << rho2 << std::endl;
    // std::cout << "est_a" << est_a << std::endl;
    // std::cout << "K" << K << std::endl;
    // std::cout << "thr" << thr << std::endl;
    // thr2acc = thr2acc + K * (est_a(2) - thr * thr2acc);
    // P = (1 - K * thr) * P / rho2;
    //printf("%6.3f,%6.3f,%6.3f,%6.3f\n", thr2acc, gamma, K, P);
    //fflush(stdout);



    return true;
  }

  return false;
}

template<typename T>
void MpcController<T>::resetThrustMapping(void)
{
  thr2acc = 9.81 / params_.thr_map.hover_percentage;
  thr_scale_compensate = 1.0;
  P = 1e6;
}

template
class MpcController<float>;

template
class MpcController<double>;

} // namespace rpg_mpc
