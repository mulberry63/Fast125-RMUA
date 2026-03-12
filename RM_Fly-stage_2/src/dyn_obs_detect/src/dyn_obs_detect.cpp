#include <iostream>
#include <string>
#include <queue>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

class DynObsDetector
{
public:
    DynObsDetector(const ros::NodeHandle &nh) : nh_(nh), recv_first_obs1_obsv(false), fsm_state(DETECTING_OBS_1)
    {
        nh_.param("fx", fx, -1.0);
        nh_.param("fy", fy, -1.0);
        nh_.param("cx", cx, -1.0);
        nh_.param("cy", cy, -1.0);

        nh_.param("obs1_waypt1_x", obs1_waypt1(0), -1.0);
        nh_.param("obs1_waypt1_y", obs1_waypt1(1), -1.0);
        nh_.param("obs1_waypt1_z", obs1_waypt1(2), -1.0);
        nh_.param("obs1_waypt2_x", obs1_waypt2(0), -1.0);
        nh_.param("obs1_waypt2_y", obs1_waypt2(1), -1.0);
        nh_.param("obs1_waypt2_z", obs1_waypt2(2), -1.0);
        nh_.param("obs2_waypt1_x", obs2_waypt1(0), -1.0);
        nh_.param("obs2_waypt1_y", obs2_waypt1(1), -1.0);
        nh_.param("obs2_waypt1_z", obs2_waypt1(2), -1.0);
        nh_.param("obs2_waypt2_x", obs2_waypt2(0), -1.0);
        nh_.param("obs2_waypt2_y", obs2_waypt2(1), -1.0);
        nh_.param("obs2_waypt2_z", obs2_waypt2(2), -1.0);
        nh_.param("obs2_waypt3_x", obs2_waypt3(0), -1.0);
        nh_.param("obs2_waypt3_y", obs2_waypt3(1), -1.0);
        nh_.param("obs2_waypt3_z", obs2_waypt3(2), -1.0);

        obs1_waypt1_ = obs1_waypt1;
        obs1_waypt2_ = obs1_waypt2;
        obs2_waypt1_ = obs2_waypt1;
        obs2_waypt2_ = obs2_waypt2;
        obs2_waypt3_ = obs2_waypt3;

        nor_vec_ob1_wp1_to_wp2 = obs1_waypt2 - obs1_waypt1;
        obs1_seg_len = nor_vec_ob1_wp1_to_wp2.norm();
        nor_vec_ob1_wp1_to_wp2.normalize();

        nor_vec_ob2_wp1_to_wp2 = obs2_waypt2 - obs2_waypt1;
        obs2_seg1_len = nor_vec_ob2_wp1_to_wp2.norm();
        nor_vec_ob2_wp1_to_wp2.normalize();

        nor_vec_ob2_wp2_to_wp3 = obs2_waypt3 - obs2_waypt2;
        obs2_seg2_len = nor_vec_ob2_wp2_to_wp3.norm();
        nor_vec_ob2_wp2_to_wp3.normalize();

        nh_.param("obs1_vel", obs1_vel, -1.0);
        nh_.param("obs2_vel", obs2_vel, -1.0);

        nh_.param("loop_topic", loop_topic, std::string(""));
        nh_.param("odom_topic", odom_topic, std::string(""));
        nh_.param("odom_queue_size", odom_queue_size, -1);

        nh_.param("depth_topic", depth_topic, std::string(""));
        nh_.param("skip_pix", skip_pix, -1);
        nh_.param("obs_dist_thr", obs_dist_thr, -1.0);
        nh_.param("obs_filter_size", obs_filter_size, -1);
        nh_.param("local_obs1_obsv_dt_thr_min", local_obs1_obsv_dt_thr_min, -1.0);
        nh_.param("local_obs1_obsv_dt_thr_max", local_obs1_obsv_dt_thr_max, -1.0);
        nh_.param("obs1_obsv_deadzone_thr", obs1_obsv_deadzone_thr, -1.0);
        nh_.param("pred_time_step", pred_time_step, -1.0);

        cam2bodyR << 0, 0, 1,
            -1, 0, 0,
            0, -1, 0;
        cam2bodyT[0] = 0.26;
        cam2bodyT[1] = 0.0;
        cam2bodyT[2] = 0.0;
        body2worldR.setIdentity();
        body2worldT.setZero();
        odomTran << 0, 1, 0,
            -1, 0, 0,
            0, 0, 1;

        obs_state_pub = nh_.advertise<geometry_msgs::PoseStamped>("/dyn_obs_state", 10);
        vis_pc_pub = nh_.advertise<sensor_msgs::PointCloud2>("vis_pc", 3);
        vis_obs_pc_pub = nh_.advertise<sensor_msgs::PointCloud2>("vis_obs_pc", 3);
        vis_obs_pub = nh_.advertise<visualization_msgs::Marker>("vis_obs", 3);
        vis_pred_pub = nh_.advertise<visualization_msgs::Marker>("vis_pred", 30);
        odom_pub = nh_.advertise<nav_msgs::Odometry>("odom_recv", 10);
        odom_sub = nh_.subscribe(odom_topic, 10, &DynObsDetector::OdomCb, this, ros::TransportHints().tcpNoDelay());
        depth_sub = nh_.subscribe(depth_topic, 1, &DynObsDetector::DepthCb, this, ros::TransportHints().tcpNoDelay());
        loop_odom_sub = nh_.subscribe(loop_topic, 1, &DynObsDetector::LoopCbk, this, ros::TransportHints().tcpNoDelay());
    }

    inline bool IsObs1(const Eigen::Vector3d &proj_pt)
    {
        return obs1_waypt1(1) < proj_pt(1) &&
               proj_pt(1) < obs1_waypt2(1) &&
               std::max(abs(proj_pt(0) - obs1_waypt1(0)), abs(proj_pt(2) - obs1_waypt1(2))) < obs_dist_thr;
    }

    inline bool InObs2Seg1(const Eigen::Vector3d &proj_pt)
    {
        Eigen::Vector3d vec_ob2_wp1_to_p = proj_pt - obs2_waypt1;
        double proj = vec_ob2_wp1_to_p.dot(nor_vec_ob2_wp1_to_wp2);
        Eigen::Vector3d dist_vec = vec_ob2_wp1_to_p - proj * nor_vec_ob2_wp1_to_wp2;
        return proj_pt(0) < obs2_waypt1(0) &&
               obs2_waypt2(0) < proj_pt(0) &&
               obs2_waypt1(1) < proj_pt(1) &&
               proj_pt(1) < obs2_waypt2(1) &&
               dist_vec.norm() < obs_dist_thr;
    }

    inline bool InObs2Seg2(const Eigen::Vector3d &proj_pt)
    {
        return obs2_waypt2(1) < proj_pt(1) &&
               proj_pt(1) < obs2_waypt3(1) &&
               std::max(abs(proj_pt(0) - obs2_waypt2(0)), abs(proj_pt(2) - obs2_waypt2(2))) < obs_dist_thr;
    }

    inline bool IsObs2(const Eigen::Vector3d &proj_pt)
    {
        return InObs2Seg1(proj_pt) || InObs2Seg2(proj_pt);
    }

    // !!! assume pt belongs to obs1
    inline double pt2state_obs1(const Eigen::Vector3d &pt, const Eigen::Vector3d &hist_pt)
    {
        double pos = pt(1) - obs1_waypt1(1);
        double hist_pos = hist_pt(1) - obs1_waypt1(1);
        if (pos - hist_pos > 0)
        {
            return pos;
        }
        else
        {
            return 2 * obs1_seg_len - pos;
        }
    }

    // !!! assume pt belongs to obs2
    inline double pt2state_obs2(const Eigen::Vector3d &pt)
    {
        if (pt(1) < obs2_waypt2(1))
        {
            Eigen::Vector3d vec_ob2_wp1_to_p = pt - obs2_waypt1;
            return vec_ob2_wp1_to_p.dot(nor_vec_ob2_wp1_to_wp2);
        }
        else if (obs2_waypt2(1) < pt(1))
        {
            return pt(1) - obs2_waypt2(1) + obs2_seg1_len;
        }
        else
        {
            std::cout << "shiiiiiiiiiiiiiiiiiiiiiiiiiiit !!!" << std::endl;
            return -1;
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

    Eigen::Matrix3d odomTran;
    void OdomCb(const nav_msgs::OdometryConstPtr &odom_msg)
    {
        Eigen::Matrix3d body2worldR_latest = Eigen::Quaterniond(odom_msg->pose.pose.orientation.w,
                                                                odom_msg->pose.pose.orientation.x,
                                                                odom_msg->pose.pose.orientation.y,
                                                                odom_msg->pose.pose.orientation.z)
                                                 .toRotationMatrix();
        // body2worldR_latest = odomTran * body2worldR_latest;

        if (body2worldR_queue.size() < odom_queue_size && body2worldT_queue.size() < odom_queue_size)
        {
            body2worldR_queue.push(body2worldR_latest);
            body2worldT_queue.emplace(odom_msg->pose.pose.position.x,
                                      odom_msg->pose.pose.position.y,
                                      odom_msg->pose.pose.position.z);
        }
        else if (body2worldR_queue.size() == odom_queue_size && body2worldT_queue.size() == odom_queue_size)
        {
            body2worldR_queue.pop();
            body2worldT_queue.pop();
            body2worldR_queue.push(body2worldR_latest);
            body2worldT_queue.emplace(odom_msg->pose.pose.position.x,
                                      odom_msg->pose.pose.position.y,
                                      odom_msg->pose.pose.position.z);
        }
        else
        {
            std::cout << "fxxxxxxxxxxxxxxxxxk !!!!!" << std::endl;
        }

        body2worldR = body2worldR_queue.front();
        body2worldT = body2worldT_queue.front();

        nav_msgs::Odometry odom_recv;
        odom_recv.header.frame_id = "world";
        odom_recv.pose.pose.position.x = body2worldT[0];
        odom_recv.pose.pose.position.y = body2worldT[1];
        odom_recv.pose.pose.position.z = body2worldT[2];
        Eigen::Quaterniond odom_recv_q(body2worldR);
        odom_recv.pose.pose.orientation.w = odom_recv_q.w();
        odom_recv.pose.pose.orientation.x = odom_recv_q.x();
        odom_recv.pose.pose.orientation.y = odom_recv_q.y();
        odom_recv.pose.pose.orientation.z = odom_recv_q.z();
        odom_pub.publish(odom_recv);
    }

    void DepthCb(const sensor_msgs::ImageConstPtr &depth_msg)
    {
        cv_bridge::CvImagePtr depth_cv;
        depth_cv = cv_bridge::toCvCopy(depth_msg, depth_msg->encoding);

        pcl::PointCloud<pcl::PointXYZ> pc;
        pcl::PointCloud<pcl::PointXYZ> obs_pc;
        float *row_ptr;
        double depth;
        int cols = depth_cv->image.cols;
        int rows = depth_cv->image.rows;
        bool no_obs2_in_this_frame = true;   // for debugging, not needed in actual races
        bool has_obs1_in_this_frame = false; // for debugging, not needed in actual races
        for (int v = 0; v < rows; v += skip_pix)
        {
            row_ptr = depth_cv->image.ptr<float>(v);
            for (int u = 0; u < cols; u += skip_pix)
            {
                depth = *row_ptr;
                row_ptr += skip_pix;

                if (depth < 0.1 ||depth > 10.1)
                    continue;

                Eigen::Vector3d proj_pt;
                proj_pt(0) = (u - cx) * depth / fx;
                proj_pt(1) = (v - cy) * depth / fy;
                proj_pt(2) = depth;
                proj_pt = cam2bodyR * proj_pt + cam2bodyT;
                proj_pt = body2worldR * proj_pt + body2worldT;

                pc.points.emplace_back(proj_pt(0), proj_pt(1), proj_pt(2));

                if (fsm_state == FSM_STATE::DETECTING_OBS_1)
                {
                    if (IsObs2(proj_pt))
                    {
                        fsm_state = FSM_STATE::DETECTING_OBS_2;
                        std::cout << "FSM_STATE: DETECTING_OBS_1 -> DETECTING_OBS_2" << std::endl;
                        recv_first_obs1_obsv = false;
                        obs_state_queue.clear();
                        obs_pc.points.clear();
                        obs_pc.points.emplace_back(proj_pt(0), proj_pt(1), proj_pt(2));
                    }
                    else if (IsObs1(proj_pt))
                        obs_pc.points.emplace_back(proj_pt(0), proj_pt(1), proj_pt(2));
                }
                else if (fsm_state == FSM_STATE::DETECTING_OBS_2)
                {
                    if (IsObs2(proj_pt))
                    {
                        obs_pc.points.emplace_back(proj_pt(0), proj_pt(1), proj_pt(2));
                        no_obs2_in_this_frame = false; // for debugging, not needed in actual races
                    }
                    else if (IsObs1(proj_pt))
                        has_obs1_in_this_frame = true; // for debugging, not needed in actual races
                }
            }
        }

        if (fsm_state == FSM_STATE::DETECTING_OBS_2 &&
            no_obs2_in_this_frame && has_obs1_in_this_frame) // for debugging, not needed in actual races
        {
            fsm_state = FSM_STATE::DETECTING_OBS_1;
            std::cout << "FSM_STATE: DETECTING_OBS_2 -> DETECTING_OBS_1" << std::endl;
            obs_state_queue.clear();
        }

        sensor_msgs::PointCloud2 vis_pc_msg;
        pcl::toROSMsg(pc, vis_pc_msg);
        vis_pc_msg.header.frame_id = "world";
        vis_pc_pub.publish(vis_pc_msg);

        if (!obs_pc.points.empty())
        {
            int iter = 0;
            Eigen::Vector3d barycenter(0, 0, 0);
            for (auto itr = obs_pc.points.begin(); itr != obs_pc.points.end(); itr++)
            {
                iter++;
                barycenter = (barycenter * (iter - 1) + Eigen::Vector3d((*itr).x, (*itr).y, (*itr).z)) / iter;
            }

            sensor_msgs::PointCloud2 vis_obs_pc_msg;
            pcl::toROSMsg(obs_pc, vis_obs_pc_msg);
            vis_obs_pc_msg.header.frame_id = "world";
            vis_obs_pc_pub.publish(vis_obs_pc_msg);

            visualization_msgs::Marker obs_marker;
            obs_marker.header.frame_id = "world";
            obs_marker.type = visualization_msgs::Marker::SPHERE;
            obs_marker.action = visualization_msgs::Marker::ADD;
            obs_marker.pose.orientation.w = 1;
            obs_marker.scale.x = 0.25;
            obs_marker.scale.y = 0.25;
            obs_marker.scale.z = 0.25;
            obs_marker.color.a = 0.9;
            obs_marker.color.r = 92.0 / 255;
            obs_marker.color.g = 133.0 / 255;
            obs_marker.color.b = 255.0 / 255;
            obs_marker.pose.position.x = barycenter[0];
            obs_marker.pose.position.y = barycenter[1];
            obs_marker.pose.position.z = barycenter[2];
            vis_obs_pub.publish(obs_marker);

            if (fsm_state == FSM_STATE::DETECTING_OBS_1) // implies barycenter is a bc of an obs1
            {
                if (!recv_first_obs1_obsv)
                {
                    recv_first_obs1_obsv = true;
                    hist_obs1_obsv.push_back(std::make_pair(depth_msg->header, barycenter));
                }
                else
                {
                    for (auto itr = hist_obs1_obsv.end(); itr != hist_obs1_obsv.begin(); itr--)
                    {
                        if (depth_msg->header.stamp.toSec() - (*itr).first.stamp.toSec() < local_obs1_obsv_dt_thr_max &&
                            depth_msg->header.stamp.toSec() - (*itr).first.stamp.toSec() > local_obs1_obsv_dt_thr_min &&
                            abs(barycenter[1] - obs1_waypt1(1)) > obs1_obsv_deadzone_thr &&
                            abs(barycenter[1] - obs1_waypt2(1)) > obs1_obsv_deadzone_thr)
                        {
                            if (obs_state_queue.size() < obs_filter_size)
                                obs_state_queue.push_back(std::make_pair(depth_msg->header, pt2state_obs1(barycenter, (*itr).second)));
                            else
                            {
                                obs_state_queue.push_back(std::make_pair(depth_msg->header, pt2state_obs1(barycenter, (*itr).second)));
                                obs_state_queue.pop_front();
                            }

                            // to planning
                            geometry_msgs::PoseStamped obs_state_msg;
                            obs_state_msg.header = depth_msg->header;
                            obs_state_msg.pose.position.x = pt2state_obs1(barycenter, (*itr).second);
                            obs_state_msg.pose.position.y = -1;
                            obs_state_msg.pose.position.z = -1;
                            obs_state_msg.pose.orientation.x = loop_transition(0);
                            obs_state_msg.pose.orientation.y = loop_transition(1);
                            obs_state_msg.pose.orientation.z = loop_transition(2);
                            obs_state_pub.publish(obs_state_msg);
                        }
                    }

                    if (hist_obs1_obsv.size() < 10)
                        hist_obs1_obsv.push_back(std::make_pair(depth_msg->header, barycenter));
                    else
                    {
                        hist_obs1_obsv.push_back(std::make_pair(depth_msg->header, barycenter));
                        hist_obs1_obsv.pop_front();
                    }
                }
            }
            else if (fsm_state == FSM_STATE::DETECTING_OBS_2)
            {
                if (obs_state_queue.size() < obs_filter_size)
                    obs_state_queue.push_back(std::make_pair(depth_msg->header, pt2state_obs2(barycenter)));
                else
                {
                    obs_state_queue.push_back(std::make_pair(depth_msg->header, pt2state_obs2(barycenter)));
                    obs_state_queue.pop_front();
                }

                // to planning
                geometry_msgs::PoseStamped obs_state_msg;
                obs_state_msg.header = depth_msg->header;
                obs_state_msg.pose.position.x = -1;
                obs_state_msg.pose.position.y = pt2state_obs2(barycenter);
                obs_state_msg.pose.position.z = -1;
                obs_state_msg.pose.orientation.x = loop_transition(0);
                obs_state_msg.pose.orientation.y = loop_transition(1);
                obs_state_msg.pose.orientation.z = loop_transition(2);
                obs_state_pub.publish(obs_state_msg);
            }
        }

        if (!obs_state_queue.empty())
        {
            for (uint pred_step = 0; pred_step < 5; pred_step++)
            {
                double pred_time = depth_msg->header.stamp.toSec() + pred_step * pred_time_step;
                Eigen::Vector3d pred_pt_avg(0, 0, 0);
                int iter = 0;
                for (auto itr = obs_state_queue.begin(); itr != obs_state_queue.end(); itr++)
                {
                    iter++;
                    double hist_time = (*itr).first.stamp.toSec();
                    double hist_state = (*itr).second;

                    double pred_state;
                    Eigen::Vector3d pred_pt;
                    if (fsm_state == FSM_STATE::DETECTING_OBS_1)
                    {
                        pred_state = hist_state + obs1_vel * (pred_time - hist_time);
                        while (pred_state > 2 * obs1_seg_len)
                        {
                            pred_state -= 2 * obs1_seg_len;
                        }
                        pred_pt = state2pt_obs1(pred_state);
                    }
                    else if (fsm_state == FSM_STATE::DETECTING_OBS_2)
                    {
                        pred_state = hist_state + obs2_vel * (pred_time - hist_time);
                        while (pred_state > obs2_seg1_len + obs2_seg2_len)
                        {
                            pred_state -= obs2_seg1_len + obs2_seg2_len;
                        }
                        pred_pt = state2pt_obs2(pred_state);
                    }

                    pred_pt_avg = (pred_pt_avg * (iter - 1) + pred_pt) / iter;
                }

                visualization_msgs::Marker pred_pt_marker;
                pred_pt_marker.id = pred_step;
                pred_pt_marker.header.frame_id = "world";
                pred_pt_marker.type = visualization_msgs::Marker::SPHERE;
                pred_pt_marker.action = visualization_msgs::Marker::ADD;
                pred_pt_marker.pose.orientation.w = 1;
                pred_pt_marker.scale.x = 0.5;
                pred_pt_marker.scale.y = 0.5;
                pred_pt_marker.scale.z = 0.5;
                pred_pt_marker.color.a = 1 - static_cast<float>(pred_step) / 7;
                pred_pt_marker.color.r = 233.0 / 255;
                pred_pt_marker.color.g = 185.0 / 255;
                pred_pt_marker.color.b = 110.0 / 255;
                pred_pt_marker.pose.position.x = pred_pt_avg[0];
                pred_pt_marker.pose.position.y = pred_pt_avg[1];
                pred_pt_marker.pose.position.z = pred_pt_avg[2];
                vis_pred_pub.publish(pred_pt_marker);
            }
        }
    }

    void LoopCbk(const geometry_msgs::PoseConstPtr &msg)
    {
        loop_transition(0) = msg->position.x;
        loop_transition(1) = msg->position.y;
        loop_transition(2) = msg->position.z;
        update();
    }

    void update()
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
private:
    enum FSM_STATE
    {
        WAITING_FOR_RING_5,
        DETECTING_OBS_1,
        DETECTING_OBS_2
    };
    FSM_STATE fsm_state;

    ros::NodeHandle nh_;

    double fx, fy, cx, cy;

    Eigen::Vector3d obs1_waypt1, obs1_waypt2, obs2_waypt1, obs2_waypt2, obs2_waypt3,
        nor_vec_ob1_wp1_to_wp2, nor_vec_ob2_wp1_to_wp2, nor_vec_ob2_wp2_to_wp3;
    Eigen::Vector3d obs1_waypt1_, obs1_waypt2_, obs2_waypt1_, obs2_waypt2_, obs2_waypt3_;
    double obs1_seg_len, obs2_seg1_len, obs2_seg2_len, obs1_vel, obs2_vel;

    std::string odom_topic;
    ros::Subscriber odom_sub;
    ros::Publisher odom_pub;

    std::string loop_topic;
    ros::Subscriber loop_odom_sub;

    int odom_queue_size;
    std::queue<Eigen::Matrix3d> body2worldR_queue;
    std::queue<Eigen::Vector3d> body2worldT_queue;
    Eigen::Matrix3d cam2bodyR; // should be const
    Eigen::Vector3d cam2bodyT; // should be const
    Eigen::Matrix3d body2worldR;
    Eigen::Vector3d body2worldT;

    Eigen::Vector3d loop_transition = Eigen::Vector3d(0,0,0);

    std::string depth_topic;
    ros::Subscriber depth_sub;
    ros::Publisher obs_state_pub;
    ros::Publisher vis_pc_pub;
    ros::Publisher vis_obs_pc_pub;
    ros::Publisher vis_obs_pub;
    ros::Publisher vis_pred_pub;
    int skip_pix;
    double obs_dist_thr;
    int obs_filter_size;
    bool recv_first_obs1_obsv;
    double local_obs1_obsv_dt_thr_min;
    double local_obs1_obsv_dt_thr_max;
    double obs1_obsv_deadzone_thr;
    double pred_time_step;
    std::deque<std::pair<std_msgs::Header, Eigen::Vector3d>> hist_obs1_obsv;
    std::deque<std::pair<std_msgs::Header, double>> obs_state_queue; // states of obs barycenter after projection to the obs segments
};

ros::Publisher obs_seg_pub;
geometry_msgs::Point obs1_waypt1, obs1_waypt2, obs2_waypt1, obs2_waypt2, obs2_waypt3;
visualization_msgs::Marker obs1_seg, obs2_seg;
void PubObsSegCb(const ros::TimerEvent &)
{
    obs_seg_pub.publish(obs1_seg);
    obs_seg_pub.publish(obs2_seg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dyn_obs_detect_node");
    ros::NodeHandle nh("~");

    DynObsDetector detector(nh);
    ros::AsyncSpinner spinner(3);
    spinner.start();

    obs_seg_pub = nh.advertise<visualization_msgs::Marker>("obs_seg", 10);

    nh.param("obs1_waypt1_x", obs1_waypt1.x, -1.0);
    nh.param("obs1_waypt1_y", obs1_waypt1.y, -1.0);
    nh.param("obs1_waypt1_z", obs1_waypt1.z, -1.0);
    nh.param("obs1_waypt2_x", obs1_waypt2.x, -1.0);
    nh.param("obs1_waypt2_y", obs1_waypt2.y, -1.0);
    nh.param("obs1_waypt2_z", obs1_waypt2.z, -1.0);
    nh.param("obs2_waypt1_x", obs2_waypt1.x, -1.0);
    nh.param("obs2_waypt1_y", obs2_waypt1.y, -1.0);
    nh.param("obs2_waypt1_z", obs2_waypt1.z, -1.0);
    nh.param("obs2_waypt2_x", obs2_waypt2.x, -1.0);
    nh.param("obs2_waypt2_y", obs2_waypt2.y, -1.0);
    nh.param("obs2_waypt2_z", obs2_waypt2.z, -1.0);
    nh.param("obs2_waypt3_x", obs2_waypt3.x, -1.0);
    nh.param("obs2_waypt3_y", obs2_waypt3.y, -1.0);
    nh.param("obs2_waypt3_z", obs2_waypt3.z, -1.0);

    obs1_seg.id = 1;
    obs1_seg.header.frame_id = "world";
    obs1_seg.type = visualization_msgs::Marker::LINE_STRIP;
    obs1_seg.action = visualization_msgs::Marker::ADD;
    obs1_seg.pose.orientation.w = 1;
    obs1_seg.scale.x = 0.25;
    obs1_seg.scale.y = 0.25;
    obs1_seg.scale.z = 0.25;
    obs1_seg.color.a = 0.75;
    obs1_seg.color.r = 114.0 / 255;
    obs1_seg.color.g = 159.0 / 255;
    obs1_seg.color.b = 207.0 / 255;
    obs1_seg.points.push_back(obs1_waypt1);
    obs1_seg.points.push_back(obs1_waypt2);

    obs2_seg.id = 2;
    obs2_seg.header.frame_id = "world";
    obs2_seg.type = visualization_msgs::Marker::LINE_STRIP;
    obs2_seg.action = visualization_msgs::Marker::ADD;
    obs2_seg.pose.orientation.w = 1;
    obs2_seg.scale.x = 0.25;
    obs2_seg.scale.y = 0.25;
    obs2_seg.scale.z = 0.25;
    obs2_seg.color.a = 0.75;
    obs2_seg.color.r = 114.0 / 255;
    obs2_seg.color.g = 159.0 / 255;
    obs2_seg.color.b = 207.0 / 255;
    obs2_seg.points.push_back(obs2_waypt1);
    obs2_seg.points.push_back(obs2_waypt2);
    obs2_seg.points.push_back(obs2_waypt3);

    ros::Timer pub_obs_seg_timer = nh.createTimer(ros::Duration(2), PubObsSegCb);

    ros::waitForShutdown();

    return 0;
}