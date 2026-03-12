#include "CtrlFSM.h"
#include <uav_utils/converters.h>

using namespace Eigen;
using std::cout;
using std::endl;
using namespace uav_utils;

CtrlFSM::CtrlFSM(Parameter_t& param_, Controller& controller_, HovThrKF& hov_thr_kf_,rpg_mpc::MpcController<float> & mpc_ctrl):
	param(param_), controller(controller_), hov_thr_kf(hov_thr_kf_),mpc_controller(mpc_ctrl),
	mpc_predicted_states_(Eigen::Matrix<float, rpg_mpc::kStateSize, rpg_mpc::kSamples + 1>::Zero()),
    mpc_predicted_inputs_(Eigen::Matrix<float, rpg_mpc::kInputSize, rpg_mpc::kSamples>::Zero()),
	est_state_((Eigen::Matrix<float, rpg_mpc::kStateSize, 1>() << 0, 0, 0, 1,0,0,0, 0, 0, 0).finished())
{
	state = MANUAL_CTRL;
	hover_pose.setZero();
}

void CtrlFSM::process()
{
	ros::Time now_time = ros::Time::now();
	Controller_Output_t u;
	SO3_Controller_Output_t u_so3;
	/*mpc control*/
	{
		est_state_(rpg_mpc::kPosX) = odom_data.p[0];
  		est_state_(rpg_mpc::kPosY) = odom_data.p[1];
  		est_state_(rpg_mpc::kPosZ) = odom_data.p[2];
		est_state_(rpg_mpc::kOriW) = odom_data.q.w();
		est_state_(rpg_mpc::kOriX) = odom_data.q.x();
		est_state_(rpg_mpc::kOriY) = odom_data.q.y();
		est_state_(rpg_mpc::kOriZ) = odom_data.q.z();  
  		est_state_(rpg_mpc::kVelX) = odom_data.v[0];
  		est_state_(rpg_mpc::kVelY) = odom_data.v[1];
  		est_state_(rpg_mpc::kVelZ) = odom_data.v[2];
		// std::cout << "11111111111111111111111111111111111111\n";
		// ROS_INFO("Ref traj");
		// for(const auto it : trajectory_data.mpc_traj.points){
		// 	std::cout << "p: "<< it.pose.position.x << " "<<it.pose.position.y<<" "<<it.pose.position.z<<std::endl;
		// 	std::cout << "v: "<< it.velocity.linear.x << " "<<it.velocity.linear.y<<" "<<it.velocity.linear.z<<std::endl;
		// 	std::cout << "a: "<< it.acceleration.linear.x<< " "<<it.acceleration.linear.y<<" "<<it.acceleration.linear.z<<std::endl;
		// 	std::cout << "j: "<< it.jerk.linear.x<< " "<<it.jerk.linear.y<<" "<<it.jerk.linear.z<<std::endl;
		// 	std::cout <<"yaw: "<< it.heading<<"\n";
		// 	break;
		// }
		// ROS_INFO_STREAM("est: "<<est_state_.transpose());
		double collect_thrust;
		mpc_controller.execMPC(trajectory_data.mpc_traj,est_state_,mpc_predicted_states_,mpc_predicted_inputs_,collect_thrust);
		// std::cout<<"prestate: \n"<<mpc_predicted_states_<<"\n";
		// std::cout<<"preinput: \n"<<mpc_predicted_inputs_<<"\n";
		// std::cout <<"pre state: " << mpc_predicted_states_.col(0).transpose()<<std::endl;
		// std::cout << "control input: " << mpc_predicted_inputs_.col(0).transpose()<<std::endl;
		collect_thrust = mpc_predicted_inputs_.col(0)[0] * param.mass / param.full_thrust;
		//publish trajectory cmd
		u.thrust = 	collect_thrust; //thrust,w,x,y,z
		u.roll_rate = mpc_predicted_inputs_.col(0)[1];
		u.pitch_rate = mpc_predicted_inputs_.col(0)[2];
		u.yaw_rate = mpc_predicted_inputs_.col(0)[3];
		// u.orientation.w() = 	mpc_predicted_inputs_(1,0); 
		// u.orientation.x() = 	mpc_predicted_inputs_(2,0); 
		// u.orientation.y() = 	mpc_predicted_inputs_(3,0); 
		// u.orientation.z() = 	mpc_predicted_inputs_(4,0); 
	}
	/*PID Ctrl*/
	
	// {
	// 	controller.config_gain(param.track_gain);
	// 	process_cmd_control(u, u_so3);
	// }
	controller.publish_ctrl(u, now_time);
	// hov_thr_kf.simple_update(u.des_v_real, odom_data.v );
	// param.config_full_thrust(hov_thr_kf.get_hov_thr());
}

void CtrlFSM::publish_trigger(const nav_msgs::Odometry& odom_msg)
{
	geometry_msgs::PoseStamped msg;
	msg.header.frame_id = "world";
	msg.pose = odom_msg.pose.pose;
	
    traj_start_trigger_pub.publish(msg);
}

bool CtrlFSM::rc_is_received(const ros::Time& now_time)
{
	return (now_time - rc_data.rcv_stamp).toSec() < param.msg_timeout.rc;
}

bool CtrlFSM::cmd_is_received(const ros::Time& now_time)
{
	return (now_time - cmd_data.rcv_stamp).toSec() < param.msg_timeout.cmd;
}

bool CtrlFSM::odom_is_received(const ros::Time& now_time)
{
	return (now_time - odom_data.rcv_stamp).toSec() < param.msg_timeout.odom;
}

bool CtrlFSM::imu_is_received(const ros::Time& now_time)
{
	return (now_time - imu_data.rcv_stamp).toSec() < param.msg_timeout.imu;
}

double CtrlFSM::get_yaw_from_odom()
{
	return get_yaw_from_quaternion(odom_data.q);
}

void CtrlFSM::process_hover_control(Controller_Output_t& u, SO3_Controller_Output_t& u_so3)
{
	Desired_State_t des;
	des.p = hover_pose.head<3>();
	des.v = Vector3d::Zero();
	des.yaw = hover_pose(3);
	des.a = Vector3d::Zero();
	des.jerk = Vector3d::Zero();
	controller.update(des, odom_data, u, u_so3);

	//publish_desire(des);
}

void CtrlFSM::process_cmd_control(Controller_Output_t& u, SO3_Controller_Output_t& u_so3)
{
	Desired_State_t des;
	des.p = cmd_data.p;
	des.v = cmd_data.v;
	des.yaw = cmd_data.yaw;
	des.a = cmd_data.a;
	des.jerk = cmd_data.jerk;
	des.head_rate = cmd_data.head_rate;
	controller.update(des, odom_data, u, u_so3);

	//publish_desire(des);	
}

void CtrlFSM::align_with_imu(Controller_Output_t& u)
{
	double imu_yaw = get_yaw_from_quaternion(imu_data.q); 
	double odom_yaw = get_yaw_from_odom();
	double des_yaw = u.yaw;
	// ROS_INFO_STREAM("imu yaw: "<<imu_yaw<<" odom_yaw: "<<odom_yaw);
	u.yaw = yaw_add(yaw_add(des_yaw, -odom_yaw), imu_yaw); 

	//out << "imu_yaw=" << imu_yaw << " odom_yaw=" << odom_yaw << " des_yaw=" << des_yaw << " u.yaw=" << u.yaw << endl;
};

void CtrlFSM::set_hov_with_odom()
{
	hover_pose.head<3>() = odom_data.p;
	hover_pose(3) = get_yaw_from_odom();
}

// void CtrlFSM::publish_desire(const Desired_State_t& des)
// {
// 	geometry_msgs::PoseStamped msg;
// 	msg.header = odom_data.msg.header;

// 	msg.pose.position.x = des.p(0);
// 	msg.pose.position.y = des.p(1);
// 	msg.pose.position.z = des.p(2);

// 	Eigen::Quaterniond q = yaw_to_quaternion(des.yaw);

// 	msg.pose.orientation.w = q.w();
// 	msg.pose.orientation.x = q.x();
// 	msg.pose.orientation.y = q.y();
// 	msg.pose.orientation.z = q.z();

// 	des_pose_pub.publish(msg);
// }

void CtrlFSM::toggle_offboard_mode(bool on_off)
{	
	mavros_msgs::SetMode offb_set_mode;
	ros::Time last_request = ros::Time::now();

	if ( on_off )
	{
		offb_set_mode.request.custom_mode = "OFFBOARD";
		controller.set_FCU_mode.call(offb_set_mode);
		// int count = 0;
		// while(count < 5 && ros::ok())
		// {
		// 	offb_set_mode.request.custom_mode = "OFFBOARD";
		// 	if( state_data.current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(1.0)))
		// 	{
		// 		if( controller.set_FCU_mode.call(offb_set_mode) && offb_set_mode.response.mode_sent)
		// 		{
		// 			ROS_INFO("Offboard enabled");
		// 			return;
		// 		}
		// 		last_request = ros::Time::now();
		// 		ROS_WARN("on Again.");
		// 		count++;
		// 	}
        // 	ros::spinOnce();
		// }
		// ROS_WARN("Toggle OFFBOARD mode on failed.");
	}
	else
	{
		offb_set_mode.request.custom_mode = "ALTCTL";
		controller.set_FCU_mode.call(offb_set_mode);
		// int count = 0;
		// while(count < 5 && ros::ok())
		// {
		// 	if ( state_data.current_state.mode == "OFFBOARD" )
		// 	{
		// 		ROS_INFO("Not in OFFBOARD mode");
		// 		return;
		// 	}
		// 	offb_set_mode.request.custom_mode = "ALTCTL";
		// 	if( state_data.current_state.mode == "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(1.0)))
		// 	{
		// 		if( controller.set_FCU_mode.call(offb_set_mode) && offb_set_mode.response.mode_sent)
		// 		{
		// 			ROS_INFO("Return from OFFBOARD mode");
		// 			return;
		// 		}
		// 		ROS_WARN("off Again.");
		// 		last_request = ros::Time::now();
		// 		count++;
		// 	}
        // 	ros::spinOnce();
		// }
		// ROS_ERROR("Toggle OFFBOARD mode off failed. EMERGENCY!!!!!");
	}
	
}
bool CtrlFSM::px4_init(){
	return (odom_data.odom_init&&imu_data.imu_init&&cmd_data.cmd_init);
}
