// get dyn obs example for planning module

#include <deque>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>

class DynObs
{
private:
    const Eigen::Vector3d obs1_waypt1 = Eigen::Vector3d(56.72, -33.05, 1.50);
    const Eigen::Vector3d obs1_waypt2 = Eigen::Vector3d(78.31, -33.05, 1.50);
    const Eigen::Vector3d obs2_waypt1 = Eigen::Vector3d(60.06, -42.05, 2.30);
    const Eigen::Vector3d obs2_waypt2 = Eigen::Vector3d(64.42, -38.58, 2.30);
    const Eigen::Vector3d obs2_waypt3 = Eigen::Vector3d(76.07, -38.58, 2.30);
    const Eigen::Vector3d vec_ob1_wp1_to_wp2 = obs1_waypt2 - obs1_waypt1;
    const Eigen::Vector3d vec_ob2_wp1_to_wp2 = obs2_waypt2 - obs2_waypt1;
    const Eigen::Vector3d vec_ob2_wp2_to_wp3 = obs2_waypt3 - obs2_waypt2;
    const double obs1_seg_len = vec_ob1_wp1_to_wp2.norm();
    const double obs2_seg1_len = vec_ob2_wp1_to_wp2.norm();
    const double obs2_seg2_len = vec_ob2_wp2_to_wp3.norm();
    const Eigen::Vector3d nor_vec_ob1_wp1_to_wp2 = vec_ob1_wp1_to_wp2 / obs1_seg_len;
    const Eigen::Vector3d nor_vec_ob2_wp1_to_wp2 = vec_ob2_wp1_to_wp2 / obs2_seg1_len;
    const Eigen::Vector3d nor_vec_ob2_wp2_to_wp3 = vec_ob2_wp2_to_wp3 / obs2_seg2_len;
    const double obs1_vel = 3.65;
    const double obs2_vel = 2.85;

    ros::NodeHandle nh_;
    ros::Subscriber obs_state_sub;
    ros::Subscriber depth_sub; // for debugging
    ros::Publisher vis_obs_pos_pub; // for debugging

    const int obs_filter_size = 5;
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

    void RecvStateCb(const geometry_msgs::PointStampedConstPtr &obs_state_msg)
    {
        UpdateParam(obs_state_msg->header, obs_state_msg->point.x, obs_state_msg->point.y);
    }

    // for debugging
    void DepthCb(const sensor_msgs::ImageConstPtr &depth_msg)
    {
        ros::Time pred_time = depth_msg->header.stamp;
        Eigen::Vector3d obs1_pos, obs1_vel, obs2_pos, obs2_vel;
        getObs1(pred_time, obs1_pos, obs1_vel);
        getObs2(pred_time, obs2_pos, obs2_vel);

        visualization_msgs::Marker pred_obs_pos_marker;
        if (see_obs1)
        {
            pred_obs_pos_marker.id = 1;
            pred_obs_pos_marker.header.frame_id = "world";
            pred_obs_pos_marker.type = visualization_msgs::Marker::SPHERE;
            pred_obs_pos_marker.action = visualization_msgs::Marker::ADD;
            pred_obs_pos_marker.pose.orientation.w = 1;
            pred_obs_pos_marker.scale.x = 0.5;
            pred_obs_pos_marker.scale.y = 0.5;
            pred_obs_pos_marker.scale.z = 0.5;
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
            pred_obs_pos_marker.scale.x = 0.5;
            pred_obs_pos_marker.scale.y = 0.5;
            pred_obs_pos_marker.scale.z = 0.5;
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

public:
    bool see_obs1 = false;
    bool see_obs2 = false;

    DynObs(const ros::NodeHandle &nh) : nh_(nh)
    {
        obs_state_sub = nh_.subscribe("/dyn_obs_state", 10, &DynObs::RecvStateCb, this, ros::TransportHints().tcpNoDelay());

        // for debugging
        vis_obs_pos_pub = nh_.advertise<visualization_msgs::Marker>("vis_pred_obs_pos", 10);
        depth_sub = nh_.subscribe("/airsim_node/drone_1/front_center/DepthPlanar", 1, &DynObs::DepthCb, this, ros::TransportHints().tcpNoDelay());
    }

    inline void getObs1(const ros::Time &time, Eigen::Vector3d &pos, Eigen::Vector3d &vel)
    {
        if (!see_obs1)
        {
            pos = Eigen::Vector3d(-1, -1, -1);
            vel = Eigen::Vector3d(-1, -1, -1);
        }

        double pred_time = time.toSec();
        Eigen::Vector3d pred_pt_avg(0, 0, 0);
        Eigen::Vector3d pred_vel_avg(0, 0, 0);

        int iter = 0;
        for (auto itr = obs1_state_queue.begin(); itr != obs1_state_queue.end(); itr++)
        {
            iter++;
            double hist_time = (*itr).first.stamp.toSec();
            double hist_state = (*itr).second;

            double pred_state = hist_state + obs1_vel * (pred_time - hist_time);
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

    inline void getObs2(const ros::Time &time, Eigen::Vector3d &pos, Eigen::Vector3d &vel)
    {
        if (!see_obs2)
        {
            pos = Eigen::Vector3d(-1, -1, -1);
            vel = Eigen::Vector3d(-1, -1, -1);
        }

        double pred_time = time.toSec();
        Eigen::Vector3d pred_pt_avg(0, 0, 0);
        Eigen::Vector3d pred_vel_avg(0, 0, 0);

        int iter = 0;
        for (auto itr = obs2_state_queue.begin(); itr != obs2_state_queue.end(); itr++)
        {
            iter++;
            double hist_time = (*itr).first.stamp.toSec();
            double hist_state = (*itr).second;

            double pred_state = hist_state + obs2_vel * (pred_time - hist_time);
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_dyn_obs_node");
    ros::NodeHandle nh("~");

    DynObs get_dyn_obs(nh);
    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::waitForShutdown();

    return 0;
}