#include "rpg_mpc/mpc_wrapper.h"
#include "rpg_mpc/mpc_fsm.h"
#include "rpg_mpc/mpc_controller.h"
#include <ros/ros.h>
#include <signal.h>

// void mySigintHandler(int sig)
// {
//     ROS_INFO("[MPCctrl] exit...");
//     ros::shutdown();
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MPCctrl");
    ros::NodeHandle nh("~");
    ros::NodeHandle pnh("~");

    // signal(SIGINT, mySigintHandler);
    // ros::Duration(1.0).sleep();

    // std::cout << "asdfsadf " << std::endl;
    rpg_mpc::MpcParams<float> param;
    param.config_from_ros_handle(nh);
    // std::cout << "11111 " << std::endl;
    rpg_mpc::MpcController<float> controller(param);
    // std::cout << "22222 " << std::endl;
    rpg_mpc::MPCFSM<float> fsm(nh, pnh, param, controller);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state",
                                                                 10,
                                                                 boost::bind(&State_Data_t::feed, &fsm.state_data, _1));

    ros::Subscriber extended_state_sub = nh.subscribe<mavros_msgs::ExtendedState>("/mavros/extended_state",
                                                                 10,
                                                                 boost::bind(&ExtendedState_Data_t::feed, &fsm.extended_state_data, _1));

    ros::Subscriber odom_sub =
        nh.subscribe<nav_msgs::Odometry>("odom",
                                         100,
                                         boost::bind(&Odom_Data_t::feed, &fsm.odom_data, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());

    ros::Subscriber start_trig_sub =
        nh.subscribe<geometry_msgs::PoseStamped>("start_trigger",
                                                10,
                                                boost::bind(&Start_Trigger_Data_t::feed, &fsm.start_trigger_data, _1));

    ros::Subscriber cmd_trig_sub =
        nh.subscribe<geometry_msgs::PoseStamped>("cmd_trigger",
                                                10,
                                                boost::bind(&Cmd_Trigger_Data_t::feed, &fsm.cmd_trigger_data, _1));

    ros::Subscriber mpc_traj_sub =
        nh.subscribe<quadrotor_msgs::Trajectory>("traj",
                                                      100,
                                                      boost::bind(&Trajectory_Data_t::feed, &fsm.trajectory_data, _1),
                                                      ros::VoidConstPtr(),
                                                      ros::TransportHints().tcpNoDelay());

    ros::Subscriber cmd_sub =
        nh.subscribe<quadrotor_msgs::PositionCommand>("cmd",
                                                      100,
                                                      boost::bind(&Command_Data_t::feed, &fsm.cmd_data, _1),
                                                      ros::VoidConstPtr(),
                                                      ros::TransportHints().tcpNoDelay());

    ros::Subscriber imu_sub =
        nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data",
                                       100,
                                       boost::bind(&Imu_Data_t::feed, &fsm.imu_data, _1),
                                       ros::VoidConstPtr(),
                                       ros::TransportHints().tcpNoDelay());
    
    ros::Subscriber rc_sub;
    // if ( !param.takeoff_land.no_RC ) // mavros will still publish wrong rc messages although no RC is connected
    {
        rc_sub = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in",
                                            10,
                                            boost::bind(&RC_Data_t::feed, &fsm.rc_data, _1));
    }

    ros::Subscriber bat_sub =
        nh.subscribe<sensor_msgs::BatteryState>("/mavros/battery",
                                                100,
                                                boost::bind(&Battery_Data_t::feed, &fsm.bat_data, _1),
                                                ros::VoidConstPtr(),
                                                ros::TransportHints().tcpNoDelay());

    ros::Subscriber takeoff_land_sub =
        nh.subscribe<quadrotor_msgs::TakeoffLand>("/takeoff_land",
                                                100,
                                                boost::bind(&Takeoff_Land_Data_t::feed, &fsm.takeoff_land_data, _1),
                                                ros::VoidConstPtr(),
                                                ros::TransportHints().tcpNoDelay());

    // revised by wyz
    ros::Subscriber rpm_sub =
        nh.subscribe<mavros_msgs::ESCStatus>("/mavros/esc_status",
                                                100,
                                                boost::bind(&Rpm_Data_t::feed, &fsm.rpm_data, _1),
                                                ros::VoidConstPtr(),
                                                ros::TransportHints().tcpNoDelay());

    fsm.planning_stop_pub_ = nh.advertise<std_msgs::Empty>("/planning_stop_trigger", 10);
    fsm.planning_restart_pub_ = nh.advertise<std_msgs::Empty>("/planning_restart_trigger", 10);
    fsm.ctrl_FCU_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
    fsm.traj_start_trigger_pub = nh.advertise<geometry_msgs::PoseStamped>("/traj_start_trigger", 10);
    fsm.des_yaw_pub = nh.advertise<nav_msgs::Odometry>("/des_yaw_pub", 10);
    
    // fsm.debug_pub = nh.advertise<quadrotor_msgs::Px4ctrlDebug>("/debugPx4ctrl", 10); // debug

    fsm.set_FCU_mode_srv = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    fsm.arming_client_srv = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    fsm.reboot_FCU_srv = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");

    ros::Duration(0.5).sleep();
    // std::cout << "3333 " << std::endl;
    if ( param.takeoff_land.no_RC )
    {
        ROS_WARN("PX4CTRL] Remote controller disabled, be careful!");
    }
    else
    {
        ROS_INFO("PX4CTRL] Waiting for RC");
        while (ros::ok())
        {
            ros::spinOnce();
            if (fsm.rc_is_received(ros::Time::now()))
            {
                ROS_INFO("[PX4CTRL] RC received.");
                break;
            }
            ros::Duration(0.1).sleep();
        }
    }

    int trials = 0;
    while (ros::ok() && !fsm.state_data.current_state.connected)
    {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        if (trials++ > 5)
            ROS_ERROR("Unable to connnect to PX4!!!");
    }

    ros::Rate r(param.ctrl_freq_max);
    while (ros::ok())
    {
        r.sleep();
        ros::spinOnce();
        fsm.process();  // We DO NOT rely on feedback as trigger, since there is no significant performance difference through our test.
    }

    return 0;
}

