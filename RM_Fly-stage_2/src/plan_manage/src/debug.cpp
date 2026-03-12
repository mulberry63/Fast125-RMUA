#include <plan_manage/plan_manage.h>

traj_opt::Config config_;
std::shared_ptr<traj_opt::RmTrajOptUniform> lcTrajOptPtr_2;
std::shared_ptr<visualization::Visualization> visPtr_;

Eigen::MatrixXd initState(3,3), midState(3,3), finState(3,3);
Eigen::MatrixXd waypoints_1(3,1), waypoints_2(3,3);
double dt_1, dt_2;
Eigen::Vector3d gate_pos;
Trajectory lc_traj;

ros::Subscriber debug_sub;

void debug_callback(const geometry_msgs::PoseStamped& msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle nh("~");

    int N_1, N_2;
    nh.getParam("N_1", N_1);
    nh.getParam("N_2", N_2);

    waypoints_1.resize(3, N_1 - 1);
    waypoints_2.resize(3, N_2 - 1);

    debug_sub = nh.subscribe("/goal", 1, debug_callback);

    config_.load(nh);
    lcTrajOptPtr_2 = std::make_shared<traj_opt::RmTrajOptUniform>(nh, config_);
    visPtr_ = std::make_shared<visualization::Visualization>(nh);

    lcTrajOptPtr_2->setVisualizer(visPtr_);

    visPtr_->registe<nav_msgs::Path>("debug_traj");
    visPtr_->registe<sensor_msgs::PointCloud2>("debug_traj_wayPts");
    visPtr_->registe<nav_msgs::Path>("vnorm_init");
    visPtr_->registe<nav_msgs::Path>("vx_init");
    visPtr_->registe<nav_msgs::Path>("vy_init");
    visPtr_->registe<nav_msgs::Path>("vz_init");
    visPtr_->registe<nav_msgs::Path>("anorm_init");
    visPtr_->registe<nav_msgs::Path>("ax_init");
    visPtr_->registe<nav_msgs::Path>("ay_init");
    visPtr_->registe<nav_msgs::Path>("az_init");
    visPtr_->registe<nav_msgs::Path>("jnorm_init");
    visPtr_->registe<nav_msgs::Path>("jx_init");
    visPtr_->registe<nav_msgs::Path>("jy_init");
    visPtr_->registe<nav_msgs::Path>("jz_init");

    initState.transpose() <<  -32.2589,    112.595,  -0.493742,
                              -4.62707,    1.88817,   0.165182,
                            -0.0892659,  -0.219028,   0.131604;

    midState.transpose() <<  -35.704,   113.951,     -0.34,
                            -4.65898,   1.80035,  0.233986,
                           0.0284545, -0.167416,  0.098184;

    finState.transpose() << -67.7286,   121.769,      3.83,
                            -4.60825,  0.330561,  0.847891,
                           -0.130847, -0.200436, 0.0120621;

    waypoints_1.transpose() << -33.9786,   113.281, -0.423703;
                            //    -0.52951,   13.8133,   2.69568,
                            //   -0.655635,   14.3414,   2.71944;

    waypoints_2.transpose() <<-43.8969,  116.878, 0.237303,
                              -51.8536,  119.203,   1.1499,
                              -59.7071,  120.843,  2.37854;
                            //   -61.2437,  121.044,  2.65408;

    dt_1 = 0.370596;
    dt_2 = 1.77809;

    gate_pos <<-35.6705,  113.786, 0.468732;

    ros::spin();

    return 0;
}

void debug_callback(const geometry_msgs::PoseStamped& msg)
{
    ros::Time t1, t2;
    t1 = ros::Time::now();
    bool res = lcTrajOptPtr_2->generate_traj(initState, midState, finState, waypoints_1, waypoints_2, dt_1, dt_2, gate_pos, lc_traj);
    t2 = ros::Time::now();
    ROS_WARN_STREAM("t2 - t1 = " << (t2 - t1).toSec());
}