#include <Eigen/Eigen>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>
#include "circle_detector/rotation_circle.h"
#include "circle_detector/transition_circle.h"

class DynObs
{
private:
    Eigen::Vector3d obs1_waypt1 = Eigen::Vector3d(33.05, 56.72, 1.50);
    Eigen::Vector3d obs1_waypt2 = Eigen::Vector3d(33.05, 78.31, 1.50);
    Eigen::Vector3d obs2_waypt1 = Eigen::Vector3d(42.05, 60.06, 2.30);
    Eigen::Vector3d obs2_waypt2 = Eigen::Vector3d(38.58, 64.42, 2.30);
    Eigen::Vector3d obs2_waypt3 = Eigen::Vector3d(38.58, 76.07, 2.30);
    Eigen::Vector3d vec_ob1_wp1_to_wp2 = obs1_waypt2 - obs1_waypt1;
    Eigen::Vector3d vec_ob2_wp1_to_wp2 = obs2_waypt2 - obs2_waypt1;
    Eigen::Vector3d vec_ob2_wp2_to_wp3 = obs2_waypt3 - obs2_waypt2;
    double obs1_seg_len = vec_ob1_wp1_to_wp2.norm();
    double obs2_seg1_len = vec_ob2_wp1_to_wp2.norm();
    double obs2_seg2_len = vec_ob2_wp2_to_wp3.norm();
    Eigen::Vector3d nor_vec_ob1_wp1_to_wp2 = vec_ob1_wp1_to_wp2 / obs1_seg_len;
    Eigen::Vector3d nor_vec_ob2_wp1_to_wp2 = vec_ob2_wp1_to_wp2 / obs2_seg1_len;
    Eigen::Vector3d nor_vec_ob2_wp2_to_wp3 = vec_ob2_wp2_to_wp3 / obs2_seg2_len;
    const double obs1_vel = 3.65;
    const double obs2_vel = 2.85;

    ros::NodeHandle nh_;
    ros::Subscriber obs_state_sub;
    ros::Publisher vis_obs_pos_pub; // for debugging
    ros::Timer process_timer_;

    Eigen::Vector3d obs1_waypt1_ = obs1_waypt1;
    Eigen::Vector3d obs1_waypt2_ = obs1_waypt2;
    Eigen::Vector3d obs2_waypt1_ = obs2_waypt1;
    Eigen::Vector3d obs2_waypt2_ = obs2_waypt2;
    Eigen::Vector3d obs2_waypt3_ = obs2_waypt3;
    ros::Subscriber loop_odom_sub;
    Eigen::Vector3d loop_transition = Eigen::Vector3d(0,0,0);

    const int obs_filter_size = 1;
    std::deque<std::pair<std_msgs::Header, double>> obs1_state_queue;
    std::deque<std::pair<std_msgs::Header, double>> obs2_state_queue;

    void UpdateParam(const std_msgs::Header &header, double obs1_state, double obs2_state)
    {
        if (obs1_state >= 0)
        {
            see_obs1 = true;
            if (obs1_state_queue.size() < obs_filter_size)
                obs1_state_queue.push_back(std::make_pair(header, obs1_state));
            else
            {
                obs1_state_queue.push_back(std::make_pair(header, obs1_state));
                obs1_state_queue.pop_front();
            }
        }
        if (obs2_state >= 0)
        {
            see_obs2 = true;
            if (obs2_state_queue.size() < obs_filter_size)
                obs2_state_queue.push_back(std::make_pair(header, obs2_state));
            else
            {
                obs2_state_queue.push_back(std::make_pair(header, obs2_state));
                obs2_state_queue.pop_front();
            }
        }
        return;
    }

    void RecvStateCb(const geometry_msgs::PoseStampedConstPtr &obs_state_msg)
    {
        loop_transition(0) = obs_state_msg->pose.orientation.x;
        loop_transition(1) = obs_state_msg->pose.orientation.y;
        loop_transition(2) = obs_state_msg->pose.orientation.z;
        update();
        UpdateParam(obs_state_msg->header, obs_state_msg->pose.position.x, obs_state_msg->pose.position.y);
    }

    // for debugging
    void TimerCb(const ros::TimerEvent& event)
    {
        ros::Time pred_time = ros::Time::now();
        Eigen::Vector3d obs1_pos, obs1_vel, obs2_pos, obs2_vel;
        getObs1(pred_time, obs1_pos, obs1_vel, 0.0);
        getObs2(pred_time, obs2_pos, obs2_vel, 0.0);

        visualization_msgs::Marker pred_obs_pos_marker;
        if (see_obs1)
        {
            pred_obs_pos_marker.id = 1;
            pred_obs_pos_marker.header.frame_id = "world";
            pred_obs_pos_marker.type = visualization_msgs::Marker::SPHERE;
            pred_obs_pos_marker.action = visualization_msgs::Marker::ADD;
            pred_obs_pos_marker.pose.orientation.w = 1;
            // pred_obs_pos_marker.scale.x = 0.5;
            // pred_obs_pos_marker.scale.y = 0.5;
            // pred_obs_pos_marker.scale.z = 0.5;
            nh_.getParam("obs_radius", pred_obs_pos_marker.scale.x);
            nh_.getParam("obs_radius", pred_obs_pos_marker.scale.y);
            nh_.getParam("obs_radius", pred_obs_pos_marker.scale.z);
            pred_obs_pos_marker.color.a = 1;
            pred_obs_pos_marker.color.r = 233.0 / 255;
            pred_obs_pos_marker.color.g = 185.0 / 255;
            pred_obs_pos_marker.color.b = 110.0 / 255;
            pred_obs_pos_marker.pose.position.x = obs1_pos[0];
            pred_obs_pos_marker.pose.position.y = obs1_pos[1];
            pred_obs_pos_marker.pose.position.z = obs1_pos[2];
            vis_obs_pos_pub.publish(pred_obs_pos_marker);
        }
        if (see_obs2)
        {
            pred_obs_pos_marker.id = 2;
            pred_obs_pos_marker.header.frame_id = "world";
            pred_obs_pos_marker.type = visualization_msgs::Marker::SPHERE;
            pred_obs_pos_marker.action = visualization_msgs::Marker::ADD;
            pred_obs_pos_marker.pose.orientation.w = 1;
            // pred_obs_pos_marker.scale.x = 0.5;
            // pred_obs_pos_marker.scale.y = 0.5;
            // pred_obs_pos_marker.scale.z = 0.5;
            nh_.getParam("obs_radius", pred_obs_pos_marker.scale.x);
            nh_.getParam("obs_radius", pred_obs_pos_marker.scale.y);
            nh_.getParam("obs_radius", pred_obs_pos_marker.scale.z);
            pred_obs_pos_marker.color.a = 1;
            pred_obs_pos_marker.color.r = 233.0 / 255;
            pred_obs_pos_marker.color.g = 185.0 / 255;
            pred_obs_pos_marker.color.b = 110.0 / 255;
            pred_obs_pos_marker.pose.position.x = obs2_pos[0];
            pred_obs_pos_marker.pose.position.y = obs2_pos[1];
            pred_obs_pos_marker.pose.position.z = obs2_pos[2];
            vis_obs_pos_pub.publish(pred_obs_pos_marker);
        }
    }

    inline Eigen::Vector3d state2pt_obs1(double state)
    {
        if (state < obs1_seg_len)
        {
            return state * nor_vec_ob1_wp1_to_wp2 + obs1_waypt1;
        }
        else
        {
            return (2 * obs1_seg_len - state) * nor_vec_ob1_wp1_to_wp2 + obs1_waypt1;
        }
    }

    inline Eigen::Vector3d state2vel_obs1(double state)
    {
        if (state < obs1_seg_len)
        {
            return obs1_vel * nor_vec_ob1_wp1_to_wp2;
        }
        else
        {
            return -obs1_vel * nor_vec_ob1_wp1_to_wp2;
        }
    }

    inline Eigen::Vector3d state2pt_obs2(double state)
    {
        if (state < obs2_seg1_len)
        {
            return state * nor_vec_ob2_wp1_to_wp2 + obs2_waypt1;
        }
        else
        {
            return (state - obs2_seg1_len) * nor_vec_ob2_wp2_to_wp3 + obs2_waypt2;
        }
    }

    inline Eigen::Vector3d state2vel_obs2(double state)
    {
        if (state < obs2_seg1_len)
        {
            return obs2_vel * nor_vec_ob2_wp1_to_wp2;
        }
        else
        {
            return obs2_vel * nor_vec_ob2_wp2_to_wp3;
        }
    }

    inline void update()
    {
        obs1_waypt1 = obs1_waypt1_ - loop_transition;
        obs1_waypt2 = obs1_waypt2_ - loop_transition;
        obs2_waypt1 = obs2_waypt1_ - loop_transition;
        obs2_waypt2 = obs2_waypt2_ - loop_transition;
        obs2_waypt3 = obs2_waypt3_ - loop_transition;

        nor_vec_ob1_wp1_to_wp2 = obs1_waypt2 - obs1_waypt1;
        obs1_seg_len = nor_vec_ob1_wp1_to_wp2.norm();
        nor_vec_ob1_wp1_to_wp2.normalize();

        nor_vec_ob2_wp1_to_wp2 = obs2_waypt2 - obs2_waypt1;
        obs2_seg1_len = nor_vec_ob2_wp1_to_wp2.norm();
        nor_vec_ob2_wp1_to_wp2.normalize();

        nor_vec_ob2_wp2_to_wp3 = obs2_waypt3 - obs2_waypt2;
        obs2_seg2_len = nor_vec_ob2_wp2_to_wp3.norm();
        nor_vec_ob2_wp2_to_wp3.normalize();
    }

public:
    bool see_obs1 = false;
    bool see_obs2 = false;

    DynObs(const ros::NodeHandle &nh) : nh_(nh)
    {
        obs_state_sub = nh_.subscribe("/dyn_obs_state", 10, &DynObs::RecvStateCb, this, ros::TransportHints().tcpNoDelay());

        // for debugging
        vis_obs_pos_pub = nh_.advertise<visualization_msgs::Marker>("vis_pred_obs_pos", 10);
        process_timer_ = nh_.createTimer(ros::Duration(0.04), &DynObs::TimerCb, this);
    }

    inline void getObs1(const ros::Time &time, Eigen::Vector3d &pos, Eigen::Vector3d &vel, double t)
    {
        if (!see_obs1)
        {
            pos = Eigen::Vector3d(-1, -1, -1);
            vel = Eigen::Vector3d(-1, -1, -1);
        }

        Eigen::Vector3d pred_pt_avg(0, 0, 0);
        Eigen::Vector3d pred_vel_avg(0, 0, 0);

        int iter = 0;
        for (auto itr = obs1_state_queue.begin(); itr != obs1_state_queue.end(); itr++)
        {
            iter++;
            ros::Time hist_time = (*itr).first.stamp;
            double hist_state = (*itr).second;

            double pred_state = hist_state + obs1_vel * ((time - hist_time).toSec() + t);
            while (pred_state > 2 * obs1_seg_len)
            {
                pred_state -= 2 * obs1_seg_len;
            }

            Eigen::Vector3d pred_pt = state2pt_obs1(pred_state);
            Eigen::Vector3d pred_vel = state2vel_obs1(pred_state);
            pred_pt_avg = (pred_pt_avg * (iter - 1) + pred_pt) / iter;
            pred_vel_avg = (pred_vel_avg * (iter - 1) + pred_vel) / iter;
        }

        pos = pred_pt_avg;
        vel = pred_vel_avg;
    }

    inline void getObs2(const ros::Time &time, Eigen::Vector3d &pos, Eigen::Vector3d &vel, double t)
    {
        if (!see_obs2)
        {
            pos = Eigen::Vector3d(-1, -1, -1);
            vel = Eigen::Vector3d(-1, -1, -1);
        }

        Eigen::Vector3d pred_pt_avg(0, 0, 0);
        Eigen::Vector3d pred_vel_avg(0, 0, 0);

        int iter = 0;
        for (auto itr = obs2_state_queue.begin(); itr != obs2_state_queue.end(); itr++)
        {
            iter++;
            ros::Time hist_time = (*itr).first.stamp;
            double hist_state = (*itr).second;

            double pred_state = hist_state + obs2_vel * ((time - hist_time).toSec() + t);
            while (pred_state > obs2_seg1_len + obs2_seg2_len)
            {
                pred_state -= obs2_seg1_len + obs2_seg2_len;
            }

            Eigen::Vector3d pred_pt = state2pt_obs2(pred_state);
            Eigen::Vector3d pred_vel = state2vel_obs2(pred_state);
            pred_pt_avg = (pred_pt_avg * (iter - 1) + pred_pt) / iter;
            pred_vel_avg = (pred_vel_avg * (iter - 1) + pred_vel) / iter;
        }

        pos = pred_pt_avg;
        vel = pred_vel_avg;
    }
};

class MovingObs0 {
    private:
        ros::Time start_timestamp = ros::Time::now();
        double t0_ = 0;

        Eigen::Vector3d p_left_ = Eigen::Vector3d(32.91, 55.72, 1.3);
        Eigen::Vector3d p_right_ = Eigen::Vector3d(33.18, 78.31, 1.7);
        double cycle_ = 12.0;
    public:
        MovingObs0() {}
        ~MovingObs0() {}

        Eigen::Vector3d getPos(const ros::Time &time)
        {
            return getPos((time - start_timestamp).toSec());
        }

        Eigen::Vector3d getVel(const ros::Time &time)
        {
            return getVel((time - start_timestamp).toSec());
        }

        Eigen::Vector3d getPos(const double &t)
        {
            Eigen::Vector3d pos;
            int nT = floor((t + t0_) / cycle_);
            double x = (t + t0_) - nT * cycle_;
            double k = x < cycle_ / 2.0 ? x / (cycle_ / 2.0) : 2.0 - x / (cycle_ / 2.0);
            pos = (1.0 - k) * p_left_ + k * p_right_;
            return pos;
        }

        Eigen::Vector3d getVel(const double &t)
        {
            Eigen::Vector3d vel;
            int nT = floor((t + t0_) / cycle_);
            double x = (t + t0_) - nT * cycle_;
            vel = x < cycle_ / 2.0 ? (p_right_ - p_left_) / (cycle_ / 2.0) : (p_left_ - p_right_) / (cycle_ / 2.0);
            return vel;
        }

        ros::Time get_timestamp()
        {
            return start_timestamp;
        }

        void update_param(const double &t0, const ros::Time &time_stamp)
        {
            t0_ = t0;
            start_timestamp = time_stamp;
            return;
        }
};

class Circle12 {
    private:
        ros::Time start_timestamp = ros::Time::now();

        ros::NodeHandle nh_;
        ros::Subscriber transition_circle_sub;
        Eigen::Vector3d rec_pos = Eigen::Vector3d(54.31, 67.32, 3.52);
        double vel_y = 0;
        double left_endpoint_y = 1.0e10;
    public:
        bool rec_flag = false;
        bool Istruth = false;

        Circle12() {}

        Circle12(const ros::NodeHandle &nh): nh_(nh)
        {
            transition_circle_sub = nh_.subscribe("/transition_circle",1, &Circle12::transition_circle_cbk, this, ros::TransportHints().tcpNoDelay());
        }
        ~Circle12() {}

        void transition_circle_cbk(const circle_detector::transition_circleConstPtr &msg)
        {
            rec_pos(0) = msg->x;
            rec_pos(1) = msg->y;
            rec_pos(2) = msg->z;
            vel_y = msg->vel;
            Istruth = msg->istruth || Istruth;
            start_timestamp = msg->header.stamp;
            left_endpoint_y = msg->left_endpoint_y;
            rec_flag = true;
        }

        Eigen::Vector3d getPos(const ros::Time &time)
        {
            return getPos((time - start_timestamp).toSec());
        }

        Eigen::Vector3d getVel(const ros::Time &time)
        {
            return getVel((time - start_timestamp).toSec());
        }

        Eigen::Vector3d getPos(const double &t)
        {
            double estimate_y = rec_pos(1) + vel_y * t;

            if (estimate_y > left_endpoint_y)
            {
                estimate_y = left_endpoint_y - (estimate_y - left_endpoint_y);
            }
            else if (estimate_y < (left_endpoint_y - 6.54))
            {
                estimate_y = (left_endpoint_y - 6.54) + ((left_endpoint_y - 6.54) - estimate_y);
            }
            
            return Eigen::Vector3d(rec_pos(0),estimate_y,rec_pos(2));
        }

        Eigen::Vector3d getVel(const double &t)
        {
            double vel = vel_y;
            double estimate_y = rec_pos(1) + vel_y * t;

            if (estimate_y > left_endpoint_y || (estimate_y < (left_endpoint_y - 6.54)))
            {
                vel = -vel_y;
            }

            return Eigen::Vector3d(0,vel,0);
        }

        ros::Time get_timestamp()
        {
            return start_timestamp;
        }
};

class Circle13 {
    private:
        ros::Time start_timestamp = ros::Time::now();
        double t0_ = 0;

        Eigen::Vector3d center_ = Eigen::Vector3d(65.85, 63.29, 2.65);
        Eigen::Matrix3d body2world_;
        double radius_ = 0;
        double cycle_ = 8.0;
        ros::Subscriber rotation_circle_info_sub;
        ros::NodeHandle nh_;
        ros::Timer vis_timer;
        ros::Publisher vis_circle_pre_pub;
        Eigen::Vector3d e1 = Eigen::Vector3d(-0.5,-0.866025,0);
        Eigen::Vector3d e2 = Eigen::Vector3d(0,0,1);

    public:
        bool rec_flag = false;

        Circle13(){}

        Circle13(const ros::NodeHandle &nh):nh_(nh)
        {
            body2world_ <<  -0.866023,    0.500004, 4.18444e-05,
                             0.473041,    0.819349,   -0.323882,
                            -0.161976,   -0.280469,   -0.946098;
            rotation_circle_info_sub = nh_.subscribe("/rotation_circle_info",1, &Circle13::rotation_circle_cbk, this, ros::TransportHints().tcpNoDelay());
            vis_timer = nh_.createTimer(ros::Duration(0.02),&Circle13::vis_timer_cbk,this);
            vis_circle_pre_pub = nh_.advertise<visualization_msgs::Marker>("/circle_pre", 10);
        }
        ~Circle13() {}

        void rotation_circle_cbk(const circle_detector::rotation_circleConstPtr &msg)
        {
            e1 = Eigen::Vector3d(msg->e11,msg->e12,msg->e13);
            center_ = Eigen::Vector3d(msg->e21,msg->e22,msg->e23);
            t0_ = msg->theta0;
            radius_ = msg->radius;
            start_timestamp = msg->time_offset.stamp;
            rec_flag = true;
        }

        void vis_timer_cbk(const ros::TimerEvent &event)
        {
            if(!rec_flag) return;
            ros::Time t = ros::Time::now();
            Eigen::Vector3d circle_pos = getPos(t);
            visualization_msgs::Marker  circle_pre_vis_msg;
            
            circle_pre_vis_msg.id = 1;
            circle_pre_vis_msg.header.frame_id = "world";
            circle_pre_vis_msg.type = visualization_msgs::Marker::SPHERE;
            circle_pre_vis_msg.action = visualization_msgs::Marker::ADD;
            circle_pre_vis_msg.pose.orientation.w = 1;
            circle_pre_vis_msg.scale.x = 0.5;
            circle_pre_vis_msg.scale.y = 0.5;
            circle_pre_vis_msg.scale.z = 0.5;
            circle_pre_vis_msg.color.a = 1;
            circle_pre_vis_msg.color.r = 233.0 / 255;
            circle_pre_vis_msg.color.g = 185.0 / 255;
            circle_pre_vis_msg.color.b = 110.0 / 255;
            circle_pre_vis_msg.pose.position.x = circle_pos[0];
            circle_pre_vis_msg.pose.position.y = circle_pos[1];
            circle_pre_vis_msg.pose.position.z = circle_pos[2];
            vis_circle_pre_pub.publish(circle_pre_vis_msg);
        }

        Eigen::Vector3d getPos(const ros::Time &time)
        {
            return getPos((time - start_timestamp).toSec());
        }

        Eigen::Vector3d getVel(const ros::Time &time)
        {
            return getVel((time - start_timestamp).toSec());
        }

        Eigen::Vector3d getPos(const double &t)
        {
            Eigen::Vector3d pos;
            pos = center_ + radius_ * (e1 * std::cos(t * 2 * M_PI /cycle_ + t0_) + e2 * std::sin(t * 2 *M_PI /cycle_ + t0_));
            return pos;
        }

        Eigen::Vector3d getVel(const double &t)
        {
            Eigen::Vector3d vel;
            vel = radius_ * ( - 2 * M_PI /cycle_ * e1 * std::sin(t * 2 * M_PI /cycle_ + t0_) + 2 * M_PI /cycle_ * e2 * std::cos(t* 2 * M_PI /cycle_ + t0_));
            return vel;
        }

        ros::Time get_timestamp()
        {
            return start_timestamp;
        }
};