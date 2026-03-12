#include <plan_manage/plan_manage.h>

namespace planner
{
PlanManager::PlanManager(ros::NodeHandle& nh):nh_(nh), initState_(3,3), finState_(3,3)
{
    waypoints_sub_ = nh_.subscribe("/goal", 1, &PlanManager::rcvGoalCallback, this);
    circle_sub_ = nh_.subscribe("circle_odom", 1, &PlanManager::rcvCircleCallback, this);
    odom_sub_ = nh_.subscribe("odom", 1, &PlanManager::rcvOdometryCallback, this);
    pg_to_vio_sub_ = nh_.subscribe("pg_T_vio", 1, &PlanManager::rcvPgToVioCallback, this);

    traj_pub_ = nh_.advertise<quadrotor_msgs::PolynomialTrajectory>("trajectory", 50);
    desired_yaw_pub_ = nh_.advertise<std_msgs::Float32>("desired_yaw", 1);
    circle_vis_pub_ = nh.advertise<sensor_msgs::PointCloud2>("moving_circle_vis", 1);
    obs_vis_pub_ = nh.advertise<sensor_msgs::PointCloud2>("moving_obs_vis", 1);
    process_timer_ = nh_.createTimer(ros::Duration(0.04), &PlanManager::TimerCallback, this);
    replan_timer_ = nh_.createTimer(ros::Duration(0.1), &PlanManager::ReplanCallback, this);

    config_.load(nh_);
    visPtr_ = std::make_shared<visualization::Visualization>(nh_);
    globTrajOptPtr_ = std::make_shared<traj_opt::GlobTrajOpt>(nh_, config_);
    globTrajOptPtr_->setVisualizer(visPtr_);
    lcTrajOptPtr_ = std::make_shared<traj_opt::GlbTrajReplanOpt>(nh_, config_);
    lcTrajOptPtr_->setVisualizer(visPtr_);
    lcTrajOptPtr_2_ = std::make_shared<traj_opt::RmTrajOptUniform>(nh_, config_);
    lcTrajOptPtr_2_->setVisualizer(visPtr_);

    circle_12_ = std::make_shared<Circle12>(nh_);
    circle_13_ = std::make_shared<Circle13>(nh_);
    moving_obs_0_ = std::make_shared<MovingObs0>();
    moving_obs_1_ = std::make_shared<MovingObs0>();
    moving_obs_2_ = std::make_shared<MovingObs0>();
    moving_obs_3_ = std::make_shared<MovingObs0>();
    lcTrajOptPtr_->setCircle12(circle_12_);
    lcTrajOptPtr_->setCircle13(circle_13_);
    lcTrajOptPtr_->setMovingObs0(moving_obs_0_);

    moving_obs_ = std::make_shared<DynObs>(nh_);
    lcTrajOptPtr_->setMovingObs(moving_obs_);

    visPtr_->registe<nav_msgs::Path>("global_traj");
    visPtr_->registe<nav_msgs::Path>("local_traj");
    visPtr_->registe<nav_msgs::Path>("init_local_traj");
    visPtr_->registe<sensor_msgs::PointCloud2>("global_traj_wayPts");
    visPtr_->registe<sensor_msgs::PointCloud2>("local_traj_wayPts");
    visPtr_->registe<sensor_msgs::PointCloud2>("init_local_traj_wayPts");

    nh_.getParam("gate_names", gate_names_);
    nh_.param<double>("piece_length", piece_length_, 10.0);
    nh_.param<double>("replan_duration", replan_duration_, 0.1);
    nh_.param<int>("local_replan_type", local_replan_type_, 0);
    nh_.param<double>("alpha_gate_update", alpha_gate_update_, 1.0);
    nh_.param<double>("min_replan_dist", min_replan_dist_, 1.0);
    nh_.param<double>("min_replan_duration", min_replan_duration_, 0.15);
    nh_.param<double>("min_circle_update_dist", min_circle_update_dist_, 2.0);

    std::vector<double> tmp_gate_pos_vec;
    nh_.getParam("gate_pos_vec", tmp_gate_pos_vec);

    for(int i = 0; i < tmp_gate_pos_vec.size() / 3; i++) {
        Eigen::Vector3d tmp_position;
        tmp_position << tmp_gate_pos_vec[3 * i + 1], tmp_gate_pos_vec[3 * i], tmp_gate_pos_vec[3 * i + 2];
        gate_pos_list_.push_back(tmp_position);
    }
}

void PlanManager::TimerCallback(const ros::TimerEvent& event)
{
    publish_desired_yaw();
    visualize_traj();

    visualize_rviz_obs();
    visualize_rviz_moving_circle();
}

void PlanManager::ReplanCallback(const ros::TimerEvent& event)
{
    if(!get_lc_traj_)
    {
        return;
    }
    if(lc_traj_.getTotalDuration() - (ros::Time::now() - lc_start_timestamp_).toSec() < 1.0)
    {
        return;
    }

    int N_1;
    nh_.getParam("N_1", N_1);

    double duration = lc_traj_.getDurations().head(N_1 + 1).sum();
    int replan_gate_index;

    if (((ros::Time::now() - lc_start_timestamp_).toSec() + replan_duration_) > duration)
    {
        replan_gate_index = gate_index_ + 1;
        gate_pos_list_[replan_gate_index] = gate_pos_list_[replan_gate_index] + pg_to_vio_;
    }
    else
    {
        replan_gate_index = gate_index_;
    }

    if (replan_gate_index == gate_pos_list_.size())
    {
        return;
    }

    if (replan_gate_index == gate_pos_list_.size() - 1)
    {
        global_se3_replan(replan_gate_index);
    }
    else
    {
        global_replan(replan_gate_index);
    }
}

void PlanManager::rcvOdometryCallback(const nav_msgs::Odometry & odom)
{
    odom_timestamp_ = odom.header.stamp;

    odom_p_(0) = odom.pose.pose.position.x;
    odom_p_(1) = odom.pose.pose.position.y;
    odom_p_(2) = odom.pose.pose.position.z;
    odom_v_(0) = odom.twist.twist.linear.x;
    odom_v_(1) = odom.twist.twist.linear.y;
    odom_v_(2) = odom.twist.twist.linear.z;

    if (!get_glb_traj_ && odom_p_.norm() < 10.0 && odom_p_.norm() > 1.5 && odom_v_.norm() < 14.0)
    {
        global_plan();
    }
}

void PlanManager::rcvGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if (!get_glb_traj_)
    {
        global_plan();
    }
}

void PlanManager::rcvCircleCallback(const geometry_msgs::PoseStamped& msg)
{
    Eigen::Vector3d circle_pos;

    circle_pos << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
    // std::cout << "[planner] rcv circle odom = " << circle_pos.transpose() << std::endl;

    int nearest_gate_index = 0;
    double min_dist = 999.0, dist_temp;

    for (int i = 0; i < gate_pos_list_.size(); i++)
    {
        dist_temp = (gate_pos_list_[i] - circle_pos).norm();
        if (dist_temp < min_dist)
        {
            nearest_gate_index = i;
            min_dist = dist_temp;
        }
    }
    // std::cout << "[planner] nearest_gate_index = " << nearest_gate_index << std::endl;
    // std::cout << "[planner] min_dist = " << min_dist << std::endl;

    if (nearest_gate_index < gate_index_)
    {
        ROS_WARN("[planner] nearest_gate_index < gate_index_!!! give up this time replan.");
        return;
    }

    if (min_dist >= min_circle_update_dist_)
    {
        return;
    }
    else
    {
        if (nearest_gate_index == gate_pos_list_.size() - 1)
        {
            double x, y, z;
            nh_.getParam("final_circle_bias_x", x);
            nh_.getParam("final_circle_bias_y", y);
            nh_.getParam("final_circle_bias_z", z);
            circle_pos += Eigen::Vector3d(x,y,z);
        }
        gate_pos_list_[nearest_gate_index] = circle_pos * alpha_gate_update_ + gate_pos_list_[nearest_gate_index] * (1 - alpha_gate_update_);
    }

    if((nearest_gate_index == gate_index_ && (odom_p_ - circle_pos).norm() < min_replan_dist_) || odom_p_.norm() < min_replan_dist_)
    {
        return;
    }
    else
    {
        if (0 == local_replan_type_)
        {
            local_replan(nearest_gate_index);
        }
        else
        {
            if (nearest_gate_index == gate_pos_list_.size() - 1)
            {
                if (get_lc_traj_)
                {
                    return;
                }
                global_se3_replan(nearest_gate_index);
            }
            else
            {
                if (get_lc_traj_)
                {
                    return;
                }
                global_replan(nearest_gate_index);
            }
        }
    }
}

void PlanManager::rcvPgToVioCallback(const geometry_msgs::Pose& msg)
{
    pg_to_vio_[0] = -msg.position.x;
    pg_to_vio_[1] = -msg.position.y;
    pg_to_vio_[2] = -msg.position.z;
}

void PlanManager::global_plan()
{
    start_p_ = odom_p_;
    start_v_ << odom_v_;
    start_a_ << 0, 0, 0;

    std::vector<double> tmp_target_p;
    nh_.getParam("target_pos", tmp_target_p);

    target_p_ << tmp_target_p[1], tmp_target_p[0], tmp_target_p[2];
    target_v_ << 0, 0, 0;
    target_a_ << 0, 0, 0;

    initState_ << start_p_, start_v_, start_a_;
    finState_ << target_p_, target_v_, target_a_;

    // N is piece num of global traj
    int N = gate_pos_list_.size() + 1;

    // waypoints include start_p and target_p
    Eigen::MatrixXd waypoints;
    Eigen::VectorXd Ts;

    waypoints.resize(3, N + 1);
    waypoints.col(0) = start_p_;
    for (int i = 0; i < N - 1; i++)
    {
      waypoints.col(i + 1) = gate_pos_list_[i];
    }
    waypoints.col(N) = target_p_;

    Ts.resize(N);
    for (int i = 0; i < N; ++i) {
      Ts[i] = (waypoints.col(i + 1) - waypoints.col(i)).norm() / config_.max_vel;
    }

    bool res = globTrajOptPtr_->generate_traj(initState_, finState_, waypoints.block(0, 1, 3, N-1), Ts, glb_traj_);

    if (res) {
        glb_start_timestamp_ = odom_timestamp_;
        get_glb_traj_ = true;

        publish_traj(glb_traj_, glb_start_timestamp_);
        visPtr_->visualize_traj(glb_traj_, "global_traj");
    }
}

void PlanManager::local_replan(const int& gate_index)
{
    if(!get_glb_traj_)
    {
        return;
    }
    if((odom_p_ - target_p_).norm() < 1.0)
    {
        return;
    }

    std::cout << "gate_index = " << gate_index << std::endl;

    ros::Time t1, t2, t3;
    t1 = ros::Time::now();

    ros::Time replan_timestamp;
    Trajectory tmp_traj;
    Eigen::MatrixXd initState(3,3), finState(3,3), midState(3,3);
    double replan_t;

    get_replan_state(replan_timestamp, initState, finState, tmp_traj, gate_index, replan_t);

    int N_1, N_2;
    nh_.getParam("N_1", N_1);
    nh_.getParam("N_2", N_2);
    int N = N_1 + N_2;

    Eigen::MatrixXd waypoints(3, N);
    Trajectory cruise_traj = get_lc_traj_ ? lc_traj_ : glb_traj_;

    double dt_1, dt_2;

    if (gate_index != gate_index_)
    {
        dt_1 = (cruise_traj.getDurations().sum() - cruise_traj.getDurations().tail(gate_pos_list_.size() - gate_index).sum() - replan_t) / N_1;
        dt_2 = glb_traj_.getDurations()[gate_index + 1] / N_2;
    }
    else
    {
        dt_1 = (cruise_traj.getDurations().head(N_1 + 1).sum() - replan_t) / N_1;
        dt_2 = (cruise_traj.getDurations().segment(N_1 + 1, N_2).sum()) / N_2;
    }

    if (dt_1 * N_1 < min_replan_duration_)
    {
        ROS_WARN_STREAM("[local_replan] dt < min_replan_duration! give up this replan.");
        return;
    }

    double tmp_t = replan_t;
    for (int i = 0; i < N; i++)
    {
        if (i < N_1)
        {
            waypoints.col(i) = cruise_traj.getPos(tmp_t);
            tmp_t += dt_1;
        }
        else if (i == N_1)
        {
            waypoints.col(i) = cruise_traj.getPos(tmp_t);
            double tmp_tmp_t = tmp_t + 0.01;
            int index = cruise_traj.locatePieceIdx(tmp_tmp_t);
            midState << cruise_traj.getJuncPos(index),
                        cruise_traj.getJuncVel(index),
                        cruise_traj.getJuncAcc(index);

            tmp_t += dt_2;
        }
        else
        {
            waypoints.col(i) = cruise_traj.getPos(tmp_t);
            tmp_t += dt_2;
        }
    }

    t2 = ros::Time::now();

    bool res = lcTrajOptPtr_2_->generate_traj(initState, midState, finState, waypoints.block(0, 1, 3, N_1 - 1), waypoints.block(0, N_1 - 1 + 2, 3, N_2 - 1), dt_1, dt_2, gate_pos_list_[gate_index], lc_traj_);

    t3 = ros::Time::now();

    std::cout << "[planner] t2 - t1 = " << (t2 - t1).toSec()*1000 << "ms." << std::endl;
    std::cout << "[planner] t3 - t2 = " << (t3 - t2).toSec()*1000 << "ms." << std::endl;

    if (res) {
        tmp_traj.append(lc_traj_);
        std::vector<Piece> rest_glb_traj_pieces;
        for (int i = gate_index + 2; i < glb_traj_.getDurations().size(); i++)
        {
            rest_glb_traj_pieces.push_back(glb_traj_[i]);
        }
        tmp_traj.append(Trajectory(rest_glb_traj_pieces));
        lc_traj_ = tmp_traj;
        get_lc_traj_ = true;
        lc_start_timestamp_ = replan_timestamp;
        publish_traj(lc_traj_, lc_start_timestamp_);
        visPtr_->visualize_traj(lc_traj_, "local_traj");

        gate_index_ = gate_index;
    }
}

void PlanManager::global_replan(const int& gate_index)
{
    if(!get_glb_traj_)
    {
        return;
    }
    if((odom_p_ - target_p_).norm() < 1.0)
    {
        return;
    }

    Trajectory cruise_traj = get_lc_traj_ ? lc_traj_ : glb_traj_;
    ros::Time cruise_traj_start_timestamp = get_lc_traj_ ? lc_start_timestamp_ : glb_start_timestamp_;

    ros::Time replan_timestamp = ros::Time::now();
    double replan_t = (replan_timestamp - cruise_traj_start_timestamp).toSec() + replan_duration_;

    BoundaryCond boundary;
    boundary << cruise_traj.getPos(replan_t - replan_duration_),
                cruise_traj.getVel(replan_t - replan_duration_),
                cruise_traj.getAcc(replan_t - replan_duration_),
                cruise_traj.getPos(replan_t),
                cruise_traj.getVel(replan_t),
                cruise_traj.getAcc(replan_t);

    Trajectory tmp_traj = Trajectory(std::vector<Piece>{Piece(boundary, replan_duration_)});

    Eigen::MatrixXd initState(3,3), finState(3,3);

    initState << cruise_traj.getPos(replan_t),
                 cruise_traj.getVel(replan_t),
                 cruise_traj.getAcc(replan_t);

    finState << cruise_traj.getPos(cruise_traj.getTotalDuration()) + (pg_to_vio_ - last_pg_to_vio_),
                Eigen::Vector3d::Zero(),
                Eigen::Vector3d::Zero();

    int N_1, N_2;
    nh_.getParam("N_1", N_1);
    nh_.getParam("N_2", N_2);
    int N = N_1 + N_2 * (gate_pos_list_.size() - gate_index);

    Eigen::MatrixXd waypoints(3, N + 1);
    Eigen::VectorXd Ts(N);
    Eigen::VectorXd fixed(N - 1);

    waypoints.col(0) = cruise_traj.getPos(replan_t);
    double tmp_t = replan_t;
    Eigen::VectorXd dt(gate_pos_list_.size() - gate_index + 1);


    dt[0] = get_lc_traj_
          ? (cruise_traj.getDurations().sum() - cruise_traj.getDurations().tail((gate_pos_list_.size() - gate_index) * N_2).sum() - replan_t) / N_1
          : (cruise_traj.getDurations().sum() - cruise_traj.getDurations().tail(gate_pos_list_.size() - gate_index).sum() - replan_t) / N_1;
    for (int i = 1; i < gate_pos_list_.size() - gate_index + 1; i++)
    {
        dt[i] = get_lc_traj_
              ? cruise_traj.getDurations().tail((gate_pos_list_.size() - gate_index - i + 1) * N_2).head(N_2).sum() / N_2
              : cruise_traj.getDurations().tail(gate_pos_list_.size() - gate_index - i + 1).head(1).sum() / N_2;
    }

    if (dt[0] * N_1 < min_replan_duration_)
    {
        ROS_WARN_STREAM("[global_replan] dt < min_replan_duration! give up this replan.");
        return;
    }

    std::cout << "durations : " << cruise_traj.getDurations().transpose() << std::endl;
    std::cout << "dt : " << dt.transpose() << std::endl;

    waypoints.col(0) = cruise_traj.getPos(replan_t);
    for (int i = 0; i < N; i++)
    {
        int j;
        if (i < N_1)
        {
            j = 0;
        }
        else
        {
            j = (i - N_1) / N_2 + 1;
        }
        Ts[i] = dt[j];
        tmp_t += Ts[i];
        waypoints.col(i + 1) = cruise_traj.getPos(tmp_t) + (pg_to_vio_ - last_pg_to_vio_);
    }

    fixed.setZero();
    for (int i = 0; i < N - 1; i++)
    {
        fixed[i] = ((i + 1 - N_1) % N_2) == 0;
    }
    std::cout << "fixed : " << fixed.transpose() << std::endl;

    lcTrajOptPtr_->setTimestamp(replan_timestamp + ros::Duration(replan_duration_));
    bool res = lcTrajOptPtr_->generate_traj_r2(initState, finState, waypoints.block(0, 1, 3, N-1), Ts, fixed, gate_pos_list_[gate_index], N_1 - 1, lc_traj_);

    if (res) {
        tmp_traj.append(lc_traj_);
        lc_traj_ = tmp_traj;
        get_lc_traj_ = true;
        lc_start_timestamp_ = replan_timestamp;
        last_pg_to_vio_ = pg_to_vio_;
        publish_traj(lc_traj_, lc_start_timestamp_);
        visPtr_->visualize_traj(lc_traj_, "local_traj");

        gate_index_ = gate_index;
    }
}

void PlanManager::global_se3_replan(const int& gate_index)
{
    Trajectory cruise_traj = lc_traj_;
    ros::Time cruise_traj_start_timestamp = lc_start_timestamp_;

    ros::Time replan_timestamp = ros::Time::now();
    double replan_t = (replan_timestamp - cruise_traj_start_timestamp).toSec() + replan_duration_;

    BoundaryCond boundary;
    boundary << cruise_traj.getPos(replan_t - replan_duration_),
                cruise_traj.getVel(replan_t - replan_duration_),
                cruise_traj.getAcc(replan_t - replan_duration_),
                cruise_traj.getPos(replan_t),
                cruise_traj.getVel(replan_t),
                cruise_traj.getAcc(replan_t);

    Trajectory tmp_traj = Trajectory(std::vector<Piece>{Piece(boundary, replan_duration_)});

    int N_1, N_2;
    nh_.getParam("N_1", N_1);
    nh_.getParam("N_2", N_2);

    int N = N_1;
    Eigen::MatrixXd initState(3,3), finState(3,3);
    Eigen::MatrixXd waypoints(3, N + 1);
    Eigen::VectorXd Ts(N);
    double dt;

    initState << cruise_traj.getPos(replan_t),
                 cruise_traj.getVel(replan_t),
                 cruise_traj.getAcc(replan_t);
    
    if (!get_se3_traj_)
    {
        finState << gate_pos_list_[gate_index],
                    cruise_traj.getJuncVel(cruise_traj.getPieceNum() - N_2),
                    cruise_traj.getJuncAcc(cruise_traj.getPieceNum() - N_2);
        dt = (cruise_traj.getDurations().sum() - cruise_traj.getDurations().tail(N_2).sum() - replan_t) / N;
    }
    else
    {
        finState << gate_pos_list_[gate_index],
                    cruise_traj.getJuncVel(cruise_traj.getPieceNum() - 2),
                    cruise_traj.getJuncAcc(cruise_traj.getPieceNum() - 2);
        dt = (cruise_traj.getDurations().sum() - cruise_traj.getDurations().tail(2).sum() - replan_t) / N;
    }

    if (dt * N < min_replan_duration_)
    {
        ROS_WARN_STREAM("dt = " << dt);
        ROS_WARN_STREAM("[global_se3_reaplan]dt < min_replan_duration! give up this replan.");
        return;
    }

    double tmp_t = replan_t;
    waypoints.col(0) = cruise_traj.getPos(tmp_t);
    for (int i = 0; i < N; i++)
    {
        Ts[i] = dt;
        tmp_t += dt;
        waypoints.col(i + 1) = cruise_traj.getPos(tmp_t) + (pg_to_vio_ - last_pg_to_vio_);
    }

    std::vector<double> tmp_target_p;
    nh_.getParam("target_pos", tmp_target_p);
    Eigen::Vector3d tmp_target_p_(tmp_target_p[1], tmp_target_p[0], tmp_target_p[2]);
    tmp_target_p_ = tmp_target_p_ + pg_to_vio_;

    bool res = lcTrajOptPtr_->generate_traj_se3(initState, finState, waypoints.block(0, 1, 3, N - 1), Ts, lc_traj_, tmp_target_p_);

    if (res) {
        tmp_traj.append(lc_traj_);
        lc_traj_ = tmp_traj;
        get_se3_traj_ = true;
        lc_start_timestamp_ = replan_timestamp;
        last_pg_to_vio_ = pg_to_vio_;
        publish_traj(lc_traj_, lc_start_timestamp_);
        visPtr_->visualize_traj(lc_traj_, "local_traj");

        gate_index_ = gate_index;
    }
}

void PlanManager::get_replan_state(ros::Time& replan_timestamp, Eigen::MatrixXd& initState, Eigen::MatrixXd& finState, Trajectory& tmp_traj, const int& gate_index, double& replan_t)
{
    replan_timestamp = ros::Time::now();

    Trajectory cruise_traj = get_lc_traj_ ? lc_traj_ : glb_traj_;
    ros::Time cruise_traj_start_timestamp = get_lc_traj_ ? lc_start_timestamp_ : glb_start_timestamp_;

    replan_t = (replan_timestamp - cruise_traj_start_timestamp).toSec() + replan_duration_;

    start_p_ = cruise_traj.getPos(replan_t);
    start_v_ = cruise_traj.getVel(replan_t);
    start_a_ = cruise_traj.getAcc(replan_t);

    initState << start_p_, start_v_, start_a_;

    target_p_ = glb_traj_.getJuncPos(gate_index + 2);
    target_v_ = glb_traj_.getJuncVel(gate_index + 2);
    target_a_ = glb_traj_.getJuncAcc(gate_index + 2);

    finState << target_p_, target_v_, target_a_;

    BoundaryCond boundary;
    boundary << cruise_traj.getPos(replan_t - replan_duration_),
                cruise_traj.getVel(replan_t - replan_duration_),
                cruise_traj.getAcc(replan_t - replan_duration_),
                cruise_traj.getPos(replan_t),
                cruise_traj.getVel(replan_t),
                cruise_traj.getAcc(replan_t);

    tmp_traj = Trajectory(std::vector<Piece>{Piece(boundary, replan_duration_)});

    return;
}

void PlanManager::visualize_traj()
{
    if (get_glb_traj_)
    {
        visPtr_->visualize_traj(glb_traj_, "global_traj");
    }
    if (get_lc_traj_)
    {
        visPtr_->visualize_traj(lc_traj_, "local_traj");
    }
}

void PlanManager::visualize_rviz_obs()
{
    pcl::PointCloud<pcl::PointXYZ> cloudMap;
    sensor_msgs::PointCloud2 globalMap_pcd;

    Eigen::Vector3d moving_obs_0_pos = moving_obs_0_->getPos(ros::Time::now());
    Eigen::Vector3d moving_obs_1_pos = moving_obs_1_->getPos(ros::Time::now());
    Eigen::Vector3d moving_obs_2_pos = moving_obs_2_->getPos(ros::Time::now());
    Eigen::Vector3d moving_obs_3_pos = moving_obs_3_->getPos(ros::Time::now());
    Eigen::Vector3d moving_obs_0_vel = moving_obs_0_->getVel(ros::Time::now());
    Eigen::Vector3d moving_obs_1_vel = moving_obs_1_->getVel(ros::Time::now());
    Eigen::Vector3d moving_obs_2_vel = moving_obs_2_->getVel(ros::Time::now());
    Eigen::Vector3d moving_obs_3_vel = moving_obs_3_->getVel(ros::Time::now());

    pcl::PointXYZ pt;
    pt.x = moving_obs_0_pos[0];
    pt.y = moving_obs_0_pos[1];
    pt.z = moving_obs_0_pos[2];
    cloudMap.points.push_back(pt);

    pt.x = moving_obs_1_pos[0];
    pt.y = moving_obs_1_pos[1];
    pt.z = moving_obs_1_pos[2];
    cloudMap.points.push_back(pt);

    pt.x = moving_obs_2_pos[0];
    pt.y = moving_obs_2_pos[1];
    pt.z = moving_obs_2_pos[2];
    cloudMap.points.push_back(pt);

    pt.x = moving_obs_3_pos[0];
    pt.y = moving_obs_3_pos[1];
    pt.z = moving_obs_3_pos[2];
    cloudMap.points.push_back(pt);

    cloudMap.width = cloudMap.points.size();
    cloudMap.height = 1;
    cloudMap.is_dense = true;

    pcl::toROSMsg(cloudMap, globalMap_pcd);

    globalMap_pcd.header.frame_id = "world";

    obs_vis_pub_.publish(globalMap_pcd);

    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> vels;
    vels.push_back(std::make_pair(moving_obs_0_pos, moving_obs_0_pos + moving_obs_0_vel));
    vels.push_back(std::make_pair(moving_obs_1_pos, moving_obs_1_pos + moving_obs_1_vel));
    vels.push_back(std::make_pair(moving_obs_2_pos, moving_obs_2_pos + moving_obs_2_vel));
    vels.push_back(std::make_pair(moving_obs_3_pos, moving_obs_3_pos + moving_obs_3_vel));

    visPtr_->visualize_arrows(vels, "obs_vel",visualization::Color::blue);
}

void PlanManager::visualize_rviz_moving_circle()
{
    pcl::PointCloud<pcl::PointXYZ> cloudMap;
    sensor_msgs::PointCloud2 globalMap_pcd;

    double radius = 0.9;
    Eigen::Vector3d circle_12_position = circle_12_->getPos(ros::Time::now());
    Eigen::Vector3d circle_13_position = circle_13_->getPos(ros::Time::now());
    Eigen::Vector3d circle_12_vel = circle_12_->getVel(ros::Time::now());
    Eigen::Vector3d circle_13_vel = circle_13_->getVel(ros::Time::now());

    Eigen::Matrix3d circle_12_body2world, circle_13_body2world;

    circle_12_body2world.transpose() << -1,  0,  0,
                                         0,  1,  0,
                                         0,  0, -1;

    circle_13_body2world.transpose() << -0.866023,    0.500004, 4.18444e-05,
                                         0.473041,    0.819349,   -0.323882,
                                        -0.161976,   -0.280469,   -0.946098;

    for(u_int64_t j=0; j<1000; j++)
    {
        double obs_rad = (double)(j) / 1000.0 * 2 * M_PI;

        Eigen::Vector3d obs_body, obs_world;
        pcl::PointXYZ pt;

        obs_body << 0, radius * cos(obs_rad), radius * sin(obs_rad);

        obs_world = circle_12_body2world * obs_body + circle_12_position;
        pt.x = obs_world[0];
        pt.y = obs_world[1];
        pt.z = obs_world[2];
        cloudMap.points.push_back(pt);

        obs_world = circle_13_body2world * obs_body + circle_13_position;
        pt.x = obs_world[0];
        pt.y = obs_world[1];
        pt.z = obs_world[2];
        cloudMap.points.push_back(pt);
    }

    cloudMap.width = cloudMap.points.size();
    cloudMap.height = 1;
    cloudMap.is_dense = true;

    pcl::toROSMsg(cloudMap, globalMap_pcd);

    globalMap_pcd.header.frame_id = "world";

    circle_vis_pub_.publish(globalMap_pcd);

    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> vels;
    vels.push_back(std::make_pair(circle_12_position, circle_12_position + circle_12_vel));
    vels.push_back(std::make_pair(circle_13_position, circle_13_position + circle_13_vel));

    visPtr_->visualize_arrows(vels, "circle_vel",visualization::Color::blue);
}

void PlanManager::publish_traj(const Trajectory& traj, const ros::Time& traj_timestamp)
{
    std::cout << "[planner] publish_timestamp - traj_timestamp = " << (ros::Time::now() - traj_timestamp).toSec() << std::endl;

    double max_v = traj.getMaxVelRate();
    double max_a = traj.getMaxAccRate();
    double max_j = traj.getMaxJerkRate();
    ROS_ERROR_STREAM("[planner] max vel: " << max_v);
    ROS_ERROR_STREAM("[planner] max acc: " << max_a);
    ROS_ERROR_STREAM("[planner] max jerk: " << max_j);

    quadrotor_msgs::PolynomialTrajectory traj_msg;
    traj_msg = traj2msg(traj, traj_timestamp);
    traj_pub_.publish(traj_msg);
}

quadrotor_msgs::PolynomialTrajectory PlanManager::traj2msg(const Trajectory& traj, const ros::Time& traj_timestamp)
{
    static int count=0;
    quadrotor_msgs::PolynomialTrajectory traj_msg;
    traj_msg.header.seq = count;
    traj_msg.header.stamp = traj_timestamp;
    traj_msg.trajectory_id = count;
    traj_msg.action = quadrotor_msgs::PolynomialTrajectory::ACTION_ADD;
    traj_msg.num_order = traj[0].getOrder(); // the order of polynomial
    traj_msg.num_segment = traj.getPieceNum();
    traj_msg.start_yaw = 0;
    traj_msg.final_yaw = 0;
    for(unsigned int i=0; i<traj_msg.num_segment; i++)
    {
        for (int j = 0; j <= traj[i].getOrder(); j++)
        {
          CoefficientMat coemat = traj[i].getCoeffMat(true);
          traj_msg.coef_x.push_back(coemat(0,j));
          traj_msg.coef_y.push_back(coemat(1,j));
          traj_msg.coef_z.push_back(coemat(2,j));
        }
        traj_msg.time.push_back(traj[i].getDuration());
        traj_msg.order.push_back(traj[i].getOrder());
    }
    traj_msg.mag_coeff = 1;
    count++;
    return traj_msg;
}

void PlanManager::publish_desired_yaw()
{
    if ((ros::Time::now() - glb_start_timestamp_).toSec() >= glb_traj_.getTotalDuration())
    {
        return;
    }
    std_msgs::Float32 desired_yaw;
    desired_yaw.data = calculate_desired_yaw();
    desired_yaw_pub_.publish(desired_yaw);
}

double PlanManager::calculate_desired_yaw()
{
    double glb_t = (ros::Time::now() - glb_start_timestamp_).toSec();
    Eigen::Vector3d vel = glb_traj_.getVel(glb_t);
    return atan2(vel(1), vel(0));
}

}//namespace planner
