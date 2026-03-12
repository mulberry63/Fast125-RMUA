#ifndef _CIRCLE_DETECTOR_
#define _CIRCLE_DETECTOR_


#include <ros/ros.h>
#include <iostream>
#include<cmath>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h> 
#include<pcl/filters/extract_indices.h>

#include <eigen3/Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include "circle_detector/visualization.hpp"
#include "circle_detector/config2.hpp"
#include "circle_detector/rotation_circle.h"
#include "circle_detector/transition_circle.h"

#include <tf/tf.h>

class Circle_Detector
{

public:
    ros::NodeHandle nh_;
    Config2 config;
    Visualizer visualizer;

    ros::Publisher all_pt_vis;
    ros::Publisher general_circle_region_pt_vis;
    ros::Publisher transition_circle_region_pt_vis;
    ros::Publisher rotation_circle_region_pt_vis;
    ros::Publisher alien_circle_region_pt_vis;

    ros::Publisher general_result_circle_pt_vis;
    ros::Publisher transition_result_circle_pt_vis;
    ros::Publisher rotation_result_circle_pt_vis;
    ros::Publisher alien_result_circle_pt_vis;

    ros::Publisher clusters_pt_vis;

    ros::Publisher circle_orientation_vis;
    ros::Publisher circle_base1_vis;
    ros::Publisher circle_base2_vis;
    ros::Publisher rotation_fake_circle_vis;
    ros::Publisher partial_detect_result_vis;
    ros::Publisher camera_point_vis;

    ros::Publisher digit_vis_pub;
    ros::Publisher vis_R_circle_pub;


    ros::Publisher debug_pub;
    ros::Publisher debug_transition_pub;

    ros::Publisher detect_loop_pub_for_dyo;
    ros::Publisher detect_loop_pub;
    ros::Subscriber depth_sub;
    ros::Subscriber rgb_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber loop_odom_sub;
    ros::Publisher  circle_pos_pub;
    ros::Publisher rotation_circle_pub;
    ros::Publisher transition_circle_pub;
    ros::Publisher rotation_circle_info_pub;
    // ros::Timer esti_timer;

    std::vector<bool> circle_flag;
    std::vector<Eigen::Vector3d> circle_relative_position;

    std::vector<Eigen::Vector3d> gate_list_;
    Eigen::Vector3d transition_gate_;
    Eigen::Vector3d rotation_gate_;
    Eigen::Vector3d alien_gate_;
    Eigen::Vector3d middle_of_transition_;

    std::vector<Eigen::Vector3d> gate_list;
    Eigen::Vector3d transition_gate;
    Eigen::Vector3d rotation_gate;
    Eigen::Vector3d alien_gate;
    Eigen::Vector3d middle_of_transition;
    double truth_region;
    int confirm_num;

    Eigen::Matrix3d cam_k_, cam_k_inv_;
    Eigen::Matrix3d cam2drone_;
    Eigen::Matrix3d loop_odom_R ;
    Eigen::Vector3d loop_odom_T; 
    Eigen::Vector3d current_odom;   

    
    bool have_depth_msg = false;
    bool have_rgb_msg = false;
    bool have_odom_msg = false;
    sensor_msgs::Image rgb_rec_msg; 
    
    //below params are for dealing with rotation circle and should be calibrated carefully
    Eigen::Vector3d transition_circle_relative_2_circle_7_;
    Eigen::Vector3d rotation_circle_relative_2_circle_7_;
    Eigen::Vector3d alien_circle_relative_2_rotation_circle_;
    Eigen::Vector3d rotation_origin = Eigen::Vector3d(65.9,  63.34,  2.5);
    Eigen::Vector3d e1 = Eigen::Vector3d(-0.5,-0.866025,0);
    Eigen::Vector3d e2 = Eigen::Vector3d(0,0,1);
    Eigen::Vector3d R_normal = Eigen::Vector3d(-0.866025,0.5,0);
    double rotation_radius = 2.0;   
    double theta0;
    const double w = 2* M_PI / 8.0;

    ros::Time t_fisrt_rotation;
    bool first_rotation = true;
    const double time_offset = -0.0;
    bool have_alien_flag_depth = false;

    int image_length_, image_height_;
    std::vector<Eigen::Vector3d> rotation_circle_pos_vec;
    std::vector<ros::Time> R_time_stamp_vec;
    std::vector<Eigen::Vector3d> transtion_circle_pos_vec;
    std::vector<ros::Time> T_time_stamp_vec;
    std::vector<nav_msgs::Odometry> odom_vec;

    Eigen::Vector3d drift = Eigen::Vector3d(0,0,0);


    Circle_Detector (const ros::NodeHandle &nh):nh_(nh)
    {
        config.init(nh);

        loop_odom_R = Eigen::Quaterniond(1,0,0,0).toRotationMatrix();
        loop_odom_T = Eigen::Vector3d(0,0,0);

        cam_k_ << config.fx,      0,          config.cx,
                    0,            config.fy,  config.cy,
                    0,            0,          1;

        cam_k_inv_ = cam_k_.inverse();
        cam2drone_ << 0,    0,    1, 
                    -1,     0,    0,
                    0,    -1,    0;

        image_height_ = 240;
        image_length_ = 320;

        e1 << config.e1[0], config.e1[1], config.e1[2];
        R_normal << config.normal[0] ,config.normal[1] ,config.normal[2];
        rotation_radius = config.rotation_radius;

        circle_flag = std::vector<bool>(10,true);
        for (size_t i = 0 ;i < config.circle_relative_postion.size() / 3 ;i++)
        {
            Eigen::Vector3d temp_point;
            temp_point << config.circle_relative_postion[3 * i], config.circle_relative_postion[3 * i + 1], config.circle_relative_postion[3 * i + 2];
            circle_relative_position.push_back(temp_point);
        }

        for (size_t i = 0 ;i < config.gate_list_.size() / 3 ;i++)
        {
            Eigen::Vector3d temp_point;
            temp_point << config.gate_list_[3 * i], config.gate_list_[3 * i + 1], config.gate_list_[3 * i + 2];
            gate_list.push_back(temp_point);
        }

        transition_circle_relative_2_circle_7_<< config.transition_circle_relative_2_circle_7[0], config.transition_circle_relative_2_circle_7[1], 
                                                 config.transition_circle_relative_2_circle_7[2];
        rotation_circle_relative_2_circle_7_<< config.rotation_circle_relative_2_circle_7[0], config.rotation_circle_relative_2_circle_7[1],
                                               config.rotation_circle_relative_2_circle_7[2];
        alien_circle_relative_2_rotation_circle_ << config.alien_circle_relative_2_rotation_circle[0], config.alien_circle_relative_2_rotation_circle[1],
                                                    config.alien_circle_relative_2_rotation_circle[2];

        transition_gate << config.transition_gate[0] ,config.transition_gate[1] ,config.transition_gate[2];
        rotation_gate << config.rotation_gate[0] ,config.rotation_gate[1] ,config.rotation_gate[2];
        
        alien_gate << config.alien_gate[0] ,config.alien_gate[1] ,config.alien_gate[2];
        rotation_origin << config.rotation_origin[0] ,config.rotation_origin[1] ,config.rotation_origin[2];
        middle_of_transition << config.middle_of_transition_line[0], config.middle_of_transition_line[1], config.middle_of_transition_line[2];
        truth_region = config.truth_region;


        gate_list_ = gate_list;
        transition_gate_ = transition_gate;
        rotation_gate_ = rotation_gate;
        middle_of_transition_ = middle_of_transition;
        alien_gate_ = alien_gate;

        // esti_timer = nh_.createTimer(ros::Duration(0.01),&Circle_Detector::vis_rotation_circle,this);
        
        depth_sub = nh_.subscribe(config.depth_topic,1,&Circle_Detector::Depth_Img_Cbk,this,ros::TransportHints().tcpNoDelay());
        rgb_sub = nh_.subscribe(config.rgb_topic,1,&Circle_Detector::Rgb_Img_Cbk,this,ros::TransportHints().tcpNoDelay());
        odom_sub = nh_.subscribe(config.odom_topic,1,&Circle_Detector::Odom_Cbk,this,ros::TransportHints().tcpNoDelay());
        loop_odom_sub = nh_.subscribe(config.loop_odom_topic,1,&Circle_Detector::Loop_odom_Cbk,this,ros::TransportHints().tcpNoDelay());
        detect_loop_pub = nh_.advertise<geometry_msgs::Pose>("/detect_loop",1);
        detect_loop_pub_for_dyo = nh_.advertise<geometry_msgs::Pose>("/detect_loop_for_dyo",1);

        circle_pos_pub = nh_.advertise<geometry_msgs::PoseStamped>("/circle_odom", 1);
        transition_circle_pub = nh_.advertise<circle_detector::transition_circle>("/transition_circle",1);
        rotation_circle_info_pub = nh_.advertise<circle_detector::rotation_circle>("/rotation_circle_info",1); 

        debug_pub = nh_.advertise<geometry_msgs::PoseStamped>("/debug_",1);
        debug_transition_pub = nh_.advertise<visualization_msgs::MarkerArray>("/debug_transition", 1);

        general_circle_region_pt_vis = nh_.advertise<sensor_msgs::PointCloud2>("general_circle_region_pt_vis",100);
        transition_circle_region_pt_vis = nh_.advertise<sensor_msgs::PointCloud2>("transition_circle_region_pt_vis",100);
        rotation_circle_region_pt_vis = nh_.advertise<sensor_msgs::PointCloud2>("rotation_circle_region_pt_vis",100);
        alien_circle_region_pt_vis = nh_.advertise<sensor_msgs::PointCloud2>("alien_circle_region_pt_vis",100);
        all_pt_vis = nh_.advertise<sensor_msgs::PointCloud2>("all_pt_vis",100);
        clusters_pt_vis = nh_.advertise<sensor_msgs::PointCloud2>("clusters_pt_vis",100);

        general_result_circle_pt_vis = nh_.advertise<sensor_msgs::PointCloud2>("general_result_circle_pt_vis",100);
        transition_result_circle_pt_vis = nh_.advertise<sensor_msgs::PointCloud2>("transition_result_circle_pt_vis",100);
        rotation_result_circle_pt_vis = nh_.advertise<sensor_msgs::PointCloud2>("rotation_result_circle_pt_vis",100);
        alien_result_circle_pt_vis = nh_.advertise<sensor_msgs::PointCloud2>("alien_result_circle_pt_vis",100);

        circle_orientation_vis = nh_.advertise<visualization_msgs::Marker>("visualizer/mesh", 1000);
        circle_base1_vis = nh_.advertise<visualization_msgs::Marker>("visualizer/base1", 1000);
        circle_base2_vis = nh_.advertise<visualization_msgs::Marker>("visualizer/base2", 1000);
        camera_point_vis = nh_.advertise<visualization_msgs::Marker>("camera_point",100);
        rotation_fake_circle_vis = nh_.advertise<sensor_msgs::PointCloud2>("rotation_fake_circle_vis",100);
        partial_detect_result_vis = nh_.advertise<sensor_msgs::PointCloud2>("partial_detect_result_vis",100);

        digit_vis_pub = nh_.advertise<sensor_msgs::PointCloud2>("digit_vis",100);
        vis_R_circle_pub = nh_.advertise<sensor_msgs::PointCloud2>("R_circle_vis",100);
    }



    void Loop_odom_Cbk(const geometry_msgs::PoseConstPtr &loop_odom_msg)
    {
        loop_odom_T(0) = loop_odom_msg->position.x;
        loop_odom_T(1) = loop_odom_msg->position.y;
        loop_odom_T(2) = loop_odom_msg->position.z;
    }



    void convert_world_prior_info_2_local(void)
    {
        if (!config.enable_loopclosture) return;
        std::cout << "[circle odom debug loop Transition]   :   " << loop_odom_T.transpose() << std::endl;
        for (size_t k = 0 ; k < gate_list.size() ; k++)
        {
            gate_list[k] =  (gate_list_[k] - loop_odom_T);
        }
        transition_gate =  (transition_gate_ - loop_odom_T);
        rotation_gate =  (rotation_gate_ - loop_odom_T);
        alien_gate =  (alien_gate_ - loop_odom_T);
        middle_of_transition =   (middle_of_transition_ - loop_odom_T);
    }



    void Rgb_Img_Cbk(const sensor_msgs::ImageConstPtr &rgb_img_msg)
    {
        have_rgb_msg = true;
        rgb_rec_msg = *rgb_img_msg;

    }



    void Depth_Img_Cbk (const sensor_msgs::ImageConstPtr &depth_img_msg)
    {   
        if (odom_vec.size() < config.min_odom_vec_size) return;

        ros::Time t_start = ros::Time::now();
        
        if (have_odom_msg)
        {
            have_odom_msg = false;

            cv_bridge::CvImagePtr cv_ptr;
            try 
            {
                cv_ptr = cv_bridge::toCvCopy(depth_img_msg,depth_img_msg->encoding.c_str());
            } 
            catch (cv_bridge::Exception& e) 
            {
                ROS_ERROR("cv_bridge execption: %s", e.what());
                return;
            }

            cv::Mat depth_image; 
            cv_ptr->image.copyTo(depth_image);

            Eigen::Vector3d drone_odom_p_D;
            Eigen::Quaterniond drone_odom_q_D;
            nav_msgs::Odometry syn_odom_D;
            double min_time_gap = 1e10;
            for (size_t k = odom_vec.size() ; k > 0 ;k--)
            {
                double temp_t = std::abs(odom_vec[k-1].header.stamp.toSec() - (depth_img_msg->header.stamp.toSec() - config.frame_delay) );
                if (temp_t < min_time_gap)  
                {
                    min_time_gap = temp_t;
                    syn_odom_D = odom_vec[k-1];
                }
            }
            std::cout << "[circle odom debug depth min_time_gap]    :   " << min_time_gap << std::endl;
            drone_odom_p_D(0) = syn_odom_D.pose.pose.position.x;
            drone_odom_p_D(1) = syn_odom_D.pose.pose.position.y;
            drone_odom_p_D(2) = syn_odom_D.pose.pose.position.z;
            drone_odom_q_D.w() = syn_odom_D.pose.pose.orientation.w;
            drone_odom_q_D.x() = syn_odom_D.pose.pose.orientation.x;
            drone_odom_q_D.y() = syn_odom_D.pose.pose.orientation.y;
            drone_odom_q_D.z() = syn_odom_D.pose.pose.orientation.z;

            tf::Quaternion tf_q(drone_odom_q_D.x(),
                                drone_odom_q_D.y(),
                                drone_odom_q_D.z(),
                                drone_odom_q_D.w());

            tf::Matrix3x3 tf_R;
            tf_R.setRotation(tf_q);

            tfScalar roll,pitch ,yaw;
            tf_R.getEulerYPR(yaw,pitch,roll);

            std::cout << "[yaw] :   " << yaw << std::endl;
            Eigen::Matrix3d cam2world = drone_odom_q_D * cam2drone_;
            Eigen::Vector3d cam2world_trans = drone_odom_q_D * Eigen::Vector3d(0.25,0,0);
            Eigen::Vector3d trans = cam2world_trans + drone_odom_p_D;
            pcl::PointCloud<pcl::PointXYZ> general_gate_points;
            pcl::PointCloud<pcl::PointXYZ> transtion_gate_points;
            pcl::PointCloud<pcl::PointXYZ> rotation_gate_points;
            pcl::PointCloud<pcl::PointXYZ> alien_gate_points;
            pcl::PointCloud<pcl::PointXYZ> vis_all_points;

            convert_world_prior_info_2_local();

            for (int u = 0; u < image_length_; u = u + config.skip_pixel) //x
            {
                for (int v = 0; v < image_height_; v=v + config.skip_pixel)//y
                {
                    double depth_pixel = depth_image.at<float>(v,u);
                    if(depth_pixel < config.min_depth || depth_pixel > config.max_depth) continue;

                    Eigen::Vector3d pixel_point = Eigen::Vector3d(u, v, 1);
                    Eigen::Vector3d camera_vec = cam_k_inv_ * pixel_point;
                    Eigen::Vector3d camera_point = camera_vec * depth_pixel;
                    camera_point = cam2world * camera_point + trans;

                    pcl::PointXYZ temp;
                    temp.x = camera_point(0); 
                    temp.y = camera_point(1);
                    temp.z = camera_point(2);
                    vis_all_points.push_back(temp);

                    pcl::PointXYZ single_point;
                    single_point.x = camera_point(0); 
                    single_point.y = camera_point(1);
                    single_point.z = camera_point(2);

                    // transition circle
                    //todo  remove prior info
                    // if (std::abs(syn_odom_D.twist.twist.linear.x) < 0.1 &&
                    //     std::abs(syn_odom_D.twist.twist.linear.y) < 0.1 &&
                    //     std::abs(yaw) <  0.05 * M_PI
                    //     )
                    {
                        if (std::abs(camera_point(0) - transition_gate(0)) < config.transition_circle_x_tolerance &&
                        std::abs(camera_point(1) - transition_gate(1)) < config.transition_circle_y_tolerance &&
                        std::abs(camera_point(2) - transition_gate(2)) < config.transition_circle_z_tolerance)
                        {
                            transtion_gate_points.push_back(single_point);   
                        }
                    }


                    //rotation circle                                  //4.0
                    if ((camera_point - rotation_gate).norm() < config.r_circle_min_search_radius /* && camera_point(2) > -0.8*/ )
                    {
                        rotation_gate_points.push_back(single_point); 
                        continue;  
                    }

                    //alien circle                               
                    if ((camera_point - alien_gate).norm() < config.a_circle_min_search_radius) //3.0
                    {
                        alien_gate_points.push_back(single_point); 
                        continue;  
                    }
                    
                    //general circle
                    for (size_t k = 0; k < gate_list.size();k++)
                    {
                        double dis2ciecle = (camera_point -gate_list[k]).norm();
                        if (dis2ciecle < config.circle_min_search_radius) //3.0
                        {
                            general_gate_points.push_back(single_point);
                            break;
                        }
                    }
                }
            }//end  depth cal

            if (config.vis_debug_enable)
            {
                visualizer.vis_pointcloud(vis_all_points,all_pt_vis);
                visualizer.vis_pointcloud(general_gate_points,general_circle_region_pt_vis);
                visualizer.vis_pointcloud(transtion_gate_points,transition_circle_region_pt_vis);
                visualizer.vis_pointcloud(rotation_gate_points,rotation_circle_region_pt_vis);
                visualizer.vis_pointcloud(alien_gate_points,alien_circle_region_pt_vis);

                std::cout << "[general_gate_points size]    :   " << general_gate_points.size() << std::endl;
                std::cout << "[alien_gate_points size]  :   " << alien_gate_points.size() << std::endl;
                std::cout << "[rotation_gate_points size]   :   " << rotation_gate_points.size() << std::endl;
                std::cout << "[transtion_gate_points size]  :   " << transtion_gate_points.size() << std::endl;
            }
            

            //general circles
            Eigen::Vector3d G_result_odom;
            std::vector<Eigen::Vector3d> G_result_cluster;
            bool G_have_result = false;
            if (!general_gate_points.empty())
            G_have_result = getCircleclusterPos(general_gate_points,G_result_cluster,
                                                G_result_odom,
                                                config.ratio1_l,          //0.7
                                                config.ratio1_u,          //1.3
                                                config.ratio2,            //50
                                                config.min_accept_radius, //0.65
                                                config.cluster_leaf_size, //0.1
                                                config.cluster_tolerance, //0.4
                                                config.cluster_min_size,  //100
                                                config.cluster_max_size); //5000
            
            if (G_have_result)
            {
                visualizer.vis_pointcloud(G_result_cluster,general_result_circle_pt_vis);
                
                update_circle_prior_position_by_realtive_position(G_result_odom);
                geometry_msgs::PoseStamped circle_pos_msg;
                circle_pos_msg.pose.position.x = G_result_odom(0);
                circle_pos_msg.pose.position.y = G_result_odom(1);
                circle_pos_msg.pose.position.z = G_result_odom(2);
                circle_pos_pub.publish(circle_pos_msg);

                std::cout << "[general pos] :   " << G_result_odom.transpose() <<std::endl;
                ROS_ERROR_STREAM("GENERAL  GENERAL GENERAL");
            }

            //transition circle
            Eigen::Vector3d T_result_odom;
            std::vector<Eigen::Vector3d> T_result_cluster;
            bool T_have_result = false;
            if (!transtion_gate_points.empty())
            T_have_result = getCircleclusterPos(transtion_gate_points, //tunable params same as general circle
                                                T_result_cluster,
                                                T_result_odom,
                                                config.t_ratio1_l,
                                                config.t_ratio1_u,
                                                config.t_ratio2,
                                                config.t_min_accept_radius,
                                                config.cluster_leaf_size,
                                                config.cluster_tolerance,
                                                config.cluster_min_size,
                                                config.cluster_max_size);
            if (T_have_result)
            {
                geometry_msgs::PoseStamped circle_pos_msg;
                circle_pos_msg.pose.position.x = T_result_odom(0);
                circle_pos_msg.pose.position.y = T_result_odom(1);
                circle_pos_msg.pose.position.z = T_result_odom(2);
                circle_pos_pub.publish(circle_pos_msg);

                visualizer.vis_pointcloud(T_result_cluster,transition_result_circle_pt_vis);

                Determine_circle_center_by_pointcloud(T_result_cluster,T_result_odom);

                static std::vector<std::pair<Eigen::Vector3d,ros::Time> > T_pos_stamp_pair_vec;
                T_pos_stamp_pair_vec.push_back(std::make_pair(T_result_odom,syn_odom_D.header.stamp));
                std::cout << "[dakeai rm] circle_pos : " << T_result_odom.transpose() << std::endl;
                std::cout << "[dakeai rm] timestamp : " << syn_odom_D.header.stamp << std::endl;
                // T_pos_stamp_pair_vec.push_back(std::make_pair(T_result_odom,depth_img_msg->header.stamp));
                if (T_pos_stamp_pair_vec.size() > config.transition_vec_size)
                {
                    // T_pos_stamp_pair_vec.erase(T_pos_stamp_pair_vec.begin());

                    double vel = 0.0;
                    static bool only_once = false;
                    double left_endpoint_y = 0.0;
                    bool IsTruth = dealWithTransition(T_pos_stamp_pair_vec,vel,left_endpoint_y); 

                    std::cout << "[circle odom debug] T_pos_stamp_pair_vec.size : " << T_pos_stamp_pair_vec.size() << std::endl;
                    std::cout << "[circle odom debug] T_pos_stamp_pair_vec.size : " << T_pos_stamp_pair_vec.size() << std::endl;
                    std::cout << "[circle odom debug] T_pos_stamp_pair_vec.size : " << T_pos_stamp_pair_vec.size() << std::endl;
                    std::cout << "[circle odom debug] T_pos_stamp_pair_vec.size : " << T_pos_stamp_pair_vec.size() << std::endl;
                    std::cout << "[circle odom debug] T_pos_stamp_pair_vec.size : " << T_pos_stamp_pair_vec.size() << std::endl;
                    std::cout << "[circle odom debug] T_pos_stamp_pair_vec.size : " << T_pos_stamp_pair_vec.size() << std::endl;
                    std::cout << "[circle odom debug] T_pos_stamp_pair_vec.size : " << T_pos_stamp_pair_vec.size() << std::endl;
                    std::cout << "[circle odom debug] T_pos_stamp_pair_vec.size : " << T_pos_stamp_pair_vec.size() << std::endl;
                    std::cout << "[circle odom debug] T_pos_stamp_pair_vec.size : " << T_pos_stamp_pair_vec.size() << std::endl;
                    std::cout << "[circle odom debug] T_pos_stamp_pair_vec.size : " << T_pos_stamp_pair_vec.size() << std::endl;

                    if (IsTruth && !only_once)
                    {
                        circle_detector::transition_circle transition_circle_msg;
                        transition_circle_msg.x = T_result_odom(0);
                        transition_circle_msg.y = T_result_odom(1);
                        transition_circle_msg.z = T_result_odom(2);
                        transition_circle_msg.istruth = IsTruth;
                        transition_circle_msg.vel = vel;
                        transition_circle_msg.header.stamp = syn_odom_D.header.stamp;
                        transition_circle_msg.left_endpoint_y = left_endpoint_y;
                        transition_circle_pub.publish(transition_circle_msg);
                        only_once = true;
                        std::cout << "[circle odom debug] left_endpoint_y : " << left_endpoint_y << std::endl;
                    }

                    // transition_circle_pub
                    // geometry_msgs::PoseStamped circle_pos_msg;
                    // circle_pos_msg.pose.position.x = T_result_odom(0);
                    // circle_pos_msg.pose.position.y = T_result_odom(1);
                    // circle_pos_msg.pose.position.z = T_result_odom(2);
                    // circle_pos_pub.publish(circle_pos_msg);

                    std::cout << "[transition IsTruth]  :   " << IsTruth << std::endl;
                    std::cout << "[transition pos]  :   " << T_result_odom.transpose() << std::endl; 
                    std::cout << "[transition vel]  :   " << vel << std::endl;
                    ROS_ERROR_STREAM("TRANSITION  TRANSITION TRANSITION");
                }

                //wd
                {
                    visualization_msgs::Marker clear_previous_msg;
                    clear_previous_msg.action = visualization_msgs::Marker::DELETEALL;
                    static visualization_msgs::Marker path_msg;
                    path_msg.type = visualization_msgs::Marker::LINE_STRIP;
                    path_msg.action = visualization_msgs::Marker::ADD;
                    path_msg.header.frame_id = "world";
                    path_msg.id = 0;
                    path_msg.pose.position.x = 0;
                    path_msg.pose.position.y = 0;
                    path_msg.pose.position.z = 0;
                    path_msg.pose.orientation.w = 1;
                    path_msg.pose.orientation.x = 0;
                    path_msg.pose.orientation.y = 0;
                    path_msg.pose.orientation.z = 0;
                    path_msg.scale.x = 0.1;
                    path_msg.scale.y = 0.1;
                    path_msg.scale.z = 0.1;
                    path_msg.color.r = 1.0;
                    path_msg.color.g = 0.0;
                    path_msg.color.b = 0.0;
                    path_msg.color.a = 1.0;
                    static visualization_msgs::MarkerArray path_list_msg;
                    // path_list_msg.markers.reserve(1);
                    // path_list_msg.markers.push_back(clear_previous_msg);

                    geometry_msgs::Point p_msg;
                    p_msg.x = T_result_odom(0);
                    p_msg.y = T_result_odom(1);
                    p_msg.z = T_result_odom(2);
                    path_msg.points.push_back(p_msg);

                    p_msg.x = drone_odom_p_D(0);
                    p_msg.y = drone_odom_p_D(1);
                    p_msg.z = drone_odom_p_D(2);
                    path_msg.points.push_back(p_msg);
                    path_list_msg.markers.push_back(path_msg);
                    path_msg.id += 1;

                    debug_transition_pub.publish(path_list_msg);
                }
            }


            //ratation circle     these param should be careful tuned
            Eigen::Vector3d R_result_odom;
            std::vector<Eigen::Vector3d> R_result_cluster;
            bool R_have_result = false;
            if (!rotation_gate_points.empty())
            R_have_result = getCircleclusterPos(rotation_gate_points,
                                                R_result_cluster,
                                                R_result_odom,
                                                config.r_ratio1_l,
                                                config.r_ratio1_u,
                                                config.r_ratio2,
                                                config.r_min_accept_radius,
                                                config.r1_cluster_leaf_size,//0.01
                                                config.r1_cluster_tolerance,//0.4
                                                config.r1_cluster_min_size,//100
                                                config.cluster_max_size);
            if (R_have_result)
            {
                visualizer.vis_pointcloud(R_result_cluster,rotation_result_circle_pt_vis);

                pcl::PointCloud<pcl::PointXYZ> input_pc;
                for (size_t k = 0 ; k < R_result_cluster.size() ; k++)
                {
                    Eigen::Vector3d tmp = R_result_cluster[k];
                    pcl::PointXYZ pt;
                    pt.x = tmp(0);
                    pt.y = tmp(1);
                    pt.z = tmp(2);
                    input_pc.push_back(pt);
                }

                bool flag_sperate = Determine_the_center_of_the_rotation_by_clustered_pointcloud(input_pc,rotation_origin,e1);
                if(flag_sperate)
                {
                    rotation_gate = rotation_origin;
                    rotation_circle_pos_vec.push_back(R_result_odom);
                    R_time_stamp_vec.push_back(syn_odom_D.header.stamp);


                    std::vector<Eigen::Vector3d> test;
                    test.push_back(R_result_odom);
                    std::vector<ros::Time> t_vec;
                    t_vec.push_back(syn_odom_D.header.stamp );
                    
                    dealWithRotation(test,t_vec);
                    circle_detector::rotation_circle rotation_circle_msg_;
                    rotation_circle_msg_.x = R_result_odom(0);
                    rotation_circle_msg_.y = R_result_odom(1);
                    rotation_circle_msg_.z = R_result_odom(2);
                    rotation_circle_msg_.e11 = e1(0);
                    rotation_circle_msg_.e12 = e1(1);
                    rotation_circle_msg_.e13 = e1(2);
                    rotation_circle_msg_.e21 = rotation_origin(0);
                    rotation_circle_msg_.e22 = rotation_origin(1);
                    rotation_circle_msg_.e23 = rotation_origin(2);
                    rotation_circle_msg_.radius = rotation_radius;
                    rotation_circle_msg_.theta0 = theta0;
                    rotation_circle_msg_.time_offset.stamp = t_fisrt_rotation;
                    rotation_circle_info_pub.publish(rotation_circle_msg_);

                    update_circle_prior_position_by_realtive_position(rotation_origin);
                    geometry_msgs::PoseStamped circle_pos_msg;
                    circle_pos_msg.pose.position.x = R_result_odom(0);
                    circle_pos_msg.pose.position.y = R_result_odom(1);
                    circle_pos_msg.pose.position.z = R_result_odom(2);
                    circle_pos_pub.publish(circle_pos_msg);

                    std::cout << "[rotation pos]    :   " << R_result_odom.transpose() << std::endl;
                    ROS_ERROR_STREAM("ROTATION  ROTATION  ROTATION");
                }
                else
                {
                    ROS_ERROR_STREAM("[Current rotation circle params can't distinguish digit and circle]");
                }

            }


            //alien circle
            Eigen::Vector3d A_result_odom;
            std::vector<Eigen::Vector3d> A_result_cluster;
            bool A_have_result = false;
            if(!alien_gate_points.empty())
            A_have_result = getCircleclusterPos(alien_gate_points,
                                                A_result_cluster,
                                                A_result_odom,
                                                config.a_ratio1_l,
                                                config.a_ratio1_u,
                                                config.a_ratio2,
                                                config.a_min_accept_radius,
                                                config.a_cluster_leaf_size,
                                                config.a_cluster_tolerance,
                                                50,10000);

            if(A_have_result)
            {
                visualizer.vis_pointcloud(A_result_cluster,alien_result_circle_pt_vis);

                have_alien_flag_depth = true;
                geometry_msgs::PoseStamped circle_pos_msg;
                circle_pos_msg.pose.position.x = A_result_odom(0);
                circle_pos_msg.pose.position.y = A_result_odom(1);
                circle_pos_msg.pose.position.z = A_result_odom(2);
                circle_pos_pub.publish(circle_pos_msg);

                std::cout << "[alien pos]   :   " << A_result_odom.transpose() <<std::endl;
                ROS_ERROR_STREAM("ALIEN  ALIEN ALIEN");
            }

        }//end if(have_odom_msg)


        if (have_rgb_msg && !have_alien_flag_depth)
        {
            if (std::abs(current_odom(0)-alien_gate(0)) < 10 && std::abs(current_odom(1) - alien_gate(1)) < 40)
            {
                have_rgb_msg = false;
                
                cv_bridge::CvImagePtr cv_ptr;
                try 
                {
                    cv_ptr = cv_bridge::toCvCopy(rgb_rec_msg,rgb_rec_msg.encoding.c_str());
                } 
                catch (cv_bridge::Exception& e) 
                {
                    ROS_ERROR("cv_bridge execption: %s", e.what());
                    return;
                }

                cv::Mat rgb_image; 
                cv_ptr->image.copyTo(rgb_image);

                Eigen::Vector3d drone_odom_p__R;
                Eigen::Quaterniond drone_odom_q_R;
                nav_msgs::Odometry syn_odom_R;
                double min_time_gap = 1e10;
                for (size_t k = odom_vec.size() ; k > 0 ;k--)
                {
                    double temp_t = std::abs(odom_vec[k-1].header.stamp.toSec() - (rgb_rec_msg.header.stamp.toSec() - config.frame_delay) );
                    if (temp_t < min_time_gap)  
                    {
                        min_time_gap = temp_t;
                        syn_odom_R = odom_vec[k-1];
                    }
                }
                std::cout << "[circle odom debug rgb min_time_gap]    :   " << min_time_gap << std::endl;

                drone_odom_p__R(0) = syn_odom_R.pose.pose.position.x;
                drone_odom_p__R(1) = syn_odom_R.pose.pose.position.y;
                drone_odom_p__R(2) = syn_odom_R.pose.pose.position.z;
                drone_odom_q_R.w() = syn_odom_R.pose.pose.orientation.w;
                drone_odom_q_R.x() = syn_odom_R.pose.pose.orientation.x;
                drone_odom_q_R.y() = syn_odom_R.pose.pose.orientation.y;
                drone_odom_q_R.z() = syn_odom_R.pose.pose.orientation.z;



                cv::Mat image = rgb_image.clone();
                //rgb filter
                cv::Mat img_inrange_0;
                cv::Mat img_inrange_1;
                cv::Mat img_inrange_2;
                std::vector<cv::Mat> img_inrange_vec;
                
                cv::Scalar lower_0 = cv::Scalar(0,0,144);
                cv::Scalar lower_1 = cv::Scalar(0,98,153);
                cv::Scalar lower_2 = cv::Scalar(0,0,91);
                std::vector<cv::Scalar> lower_vec;
                lower_vec.push_back(lower_0);
                lower_vec.push_back(lower_1);
                lower_vec.push_back(lower_2);
                

                cv::Scalar upper_0 = cv::Scalar(147,99,167);
                cv::Scalar upper_1 = cv::Scalar(255,132,246);
                cv::Scalar upper_2 = cv::Scalar(73,71,255);
                std::vector<cv::Scalar> upper_vec;
                upper_vec.push_back(upper_0);
                upper_vec.push_back(upper_1);
                upper_vec.push_back(upper_2);

                std::vector<cv::Mat> edge_img_vec;
                cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3), cv::Point(-1, -1));

                Eigen::Vector3d alien_result_pos;
                double min_dis2alien_circle = 1.0e10;
                for (size_t k = 0 ; k < 3 ;k++)
                {
                    cv::Mat img_for_proc = rgb_image.clone();
                    cv::Mat img_inrange;

                    bool first = true;
                    cv::inRange(img_for_proc,lower_vec[k],upper_vec[k],img_inrange);
                    // cv::imshow("img_inrange",img_inrange);

                    medianBlur(img_inrange,img_inrange,3);
                    morphologyEx(img_inrange, img_inrange, cv::MORPH_CLOSE, kernel,cv::Point(-1,-1),2);

                    cv::Mat imageI = img_inrange.clone();
                    GaussianBlur(imageI, imageI, cv::Size(5, 5), 1.0);
                    cv::Mat1b E;
                    cv::Canny(imageI,E,50,150);
                    cv::Mat edge_fit = E.clone();
                    // cv::imshow("E",E);
                    pcl::PointCloud<pcl::PointXYZI> candidate_pixel_vec;
                    for (size_t u = 0;u < edge_fit.cols; u++)
                    {
                        for (size_t v = 0 ; v < edge_fit.rows; v++)
                        {
                            if (edge_fit.at<uchar>(v,u) == 255)
                            {
                                pcl::PointXYZI pt;
                                pt.x = u;
                                pt.y = v;
                                pt.z = 0;
                                candidate_pixel_vec.push_back(pt);
                            }
                        }
                    }
                    if (!candidate_pixel_vec.empty())
                    {
                        // clustering by euclidean-distance
                        pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_ptr = candidate_pixel_vec.makeShared();
                        pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZI>);
                        kdtree->setInputCloud(input_cloud_ptr);

                        pcl::EuclideanClusterExtraction<pcl::PointXYZI> clustering;
                        clustering.setClusterTolerance(2);
                        clustering.setMinClusterSize(50);
                        clustering.setMaxClusterSize(5000);
                        clustering.setSearchMethod(kdtree);
                        clustering.setInputCloud(input_cloud_ptr);
                        std::vector<pcl::PointIndices> clusters;
                        clustering.extract(clusters);

                        Eigen::Matrix3d pMat;

                        // pMat << 318.5118713378906,     0,                   320,
                        //                     0,         318.5118713378906,   240,
                        //                     0,         0,                   1;

                        pMat << 320.0,     0,                   320,
                                0,         320.0,   240,
                                0,         0,                   1;
                                
                        Eigen::Matrix3d cam2drone_R;
                        cam2drone_R <<  0, 0, 1, 
                                        -1, 0, 0,
                                        0,-1, 0;
                        //only the largest size cluster will be used to do ellipse fitting 
                        //todo overlap
                        if (!clusters.empty())
                        {
                            std::vector<cv::Point>ellipse_candidate;

                            for (auto iter = clusters.begin();iter!=clusters.end();iter++)
                            {   
                                cv::Scalar scalar(0);
                                cv::Mat cluster_result_img(cv::Size(640,480),CV_8U,scalar);
                                for (auto index = iter->indices.begin();index != iter->indices.end(); index++)
                                {
                                    cv::Point pt;
                                    pt.x = input_cloud_ptr->points[*index].x;
                                    pt.y = input_cloud_ptr->points[*index].y;
                                    ellipse_candidate.push_back(pt);
                                    cluster_result_img.at<uchar>(pt.y,pt.x) = 255;
                                }
                                // cv::imshow("cluster img",cluster_result_img);
                                cv::RotatedRect obb = cv::fitEllipse(ellipse_candidate);
                                //detection visualization
                                cv::Mat ellipse_by_cluster = image.clone();
                                cv::Scalar color(50,50,168);
                                cv::ellipse(ellipse_by_cluster,obb,color,3);
                                //圆心
                                cv::circle(ellipse_by_cluster,cv::Point(obb.center.x,obb.center.y),3,color,3);
                                //长轴
                                double x = obb.center.x + obb.size.height * cos((obb.angle-90)*M_PI/180.0) / 2;
                                double y = obb.center.y + obb.size.height * sin((obb.angle-90)*M_PI/180.0) / 2;
                                cv::Scalar color_1(10,10,168);
                                cv::line(ellipse_by_cluster,cv::Point(obb.center.x,obb.center.y),cv::Point(x,y),color_1,3);
                                //短轴
                                x = obb.center.x + -obb.size.width * sin((obb.angle-90)*M_PI/180.0) / 2;
                                y = obb.center.y + obb.size.width * cos((obb.angle-90)*M_PI/180.0) / 2;
                                cv::line(ellipse_by_cluster,cv::Point(obb.center.x,obb.center.y),cv::Point(x,y),color_1,3);

                                //circle_pose
                                Eigen::Vector3d pos1;
                                Eigen::Vector3d normal1;
                                Eigen::Vector3d pos2;
                                Eigen::Vector3d normal2;
                                Eigen::Vector2d center(obb.center.x,obb.center.y);
                                Eigen::Vector2d semi_axis(obb.size.height , obb.size.width);
                                double angle = obb.angle - 90;

                                Eigen::Matrix3d ellipseMat = params2EllipseMat(center,semi_axis,angle,ellipse_candidate);

                                static double max_dis = 0;
                                if (get_circle_pose(pMat,ellipseMat,pos1,normal1,pos2,normal2))
                                {	
                                    Eigen::Vector3d alien_circle_pos;
                                    alien_circle_pos = pos1(2) > 0 ? drone_odom_q_R * (cam2drone_R * pos1 + Eigen::Vector3d(0.25,0,0)) + drone_odom_p__R
                                                                : drone_odom_q_R * (cam2drone_R * pos2 + Eigen::Vector3d(0.25,0,0)) + drone_odom_p__R;
                                    
                                    // std::cout << "[circle odom debug ellipse result] :   " << alien_result_pos.transpose() << std::endl;
                                    double dis2prior_pos = (alien_circle_pos - alien_gate).norm(); 
                                    if (dis2prior_pos < min_dis2alien_circle)
                                    {
                                        alien_result_pos = alien_circle_pos;
                                        min_dis2alien_circle = dis2prior_pos;
                                    }
                                    // cv::imshow("cluster_ellipse",ellipse_by_cluster);
                                    // cv::waitKey(0);
                                }

                                Eigen::Vector3d pt_at_normal_plane_body = pMat.inverse() * Eigen::Vector3d(obb.center.x,obb.center.y,1);
                                Eigen::Vector3d pt_at_normal_plane_world = drone_odom_q_R * (cam2drone_R * pt_at_normal_plane_body + Eigen::Vector3d(0.25,0,0)) + drone_odom_p__R;
                                Eigen::Vector3d camera_origin = drone_odom_q_R * Eigen::Vector3d(0.26,0,0) + drone_odom_p__R;
                                Eigen::Vector3d beam_vec = (pt_at_normal_plane_world - camera_origin).normalized();
                                double t_ = (Eigen::Vector3d(69.54,42.36,2.0) - camera_origin)(1) / beam_vec(1);
                                Eigen::Vector3d alien_result_pos_beam = camera_origin + beam_vec * t_;

                                // std::cout << "[circle odom debug beam result]   :   " << alien_result_pos_beam.transpose() << std::endl;
                                Eigen::Vector3d direct(0,0,1);
                                // visualizer.vis_orientation(alien_result_pos_beam,direct,circle_orientation_vis);
                                double dis2alien = (alien_result_pos_beam - alien_gate).norm();

                                if (dis2alien < min_dis2alien_circle)
                                {
                                    alien_result_pos = alien_result_pos_beam;
                                    min_dis2alien_circle = dis2alien;
                                }
                            }
                        }
                    }
                } //end for

                if( (alien_result_pos - alien_gate).norm() < config.a_result_tolerance)
                {
                    geometry_msgs::PoseStamped circle_pos_msg;
                    circle_pos_msg.pose.position.x = alien_result_pos(0);
                    circle_pos_msg.pose.position.y = alien_result_pos(1);
                    circle_pos_msg.pose.position.z = alien_result_pos(2);
                    circle_pos_pub.publish(circle_pos_msg);

                    Eigen::Vector3d direct(0,0,1);
                    visualizer.vis_orientation(alien_result_pos,direct,camera_point_vis);
                    std::cout << "[circle odom debug rgb alien circle position] :   " << alien_result_pos.transpose() << std::endl;
                    std::cout << "[circle odom debug current odom]  :   " << current_odom.transpose() << std::endl;
                    std::cout << "[circle odom debug distance 2 alien circle]   :   " << (current_odom - alien_result_pos).norm() << std::endl;
                    ROS_ERROR_STREAM("RGB ALIEN RGB ALIEN RGB ALIEN");
                }
            }//end distance if
        } //end if have_rgb_msg

        ROS_WARN_STREAM("[total time consume] :  " <<(ros::Time::now() - t_start).toSec());
    }



    void Odom_Cbk(const nav_msgs::OdometryConstPtr &odom_msg)
    {  
        have_odom_msg = true;
        odom_vec.push_back(*odom_msg);
        if(odom_vec.size() > config.max_odom_vec_size) odom_vec.erase(odom_vec.begin());
        current_odom(0) = odom_msg->pose.pose.position.x;
        current_odom(1) = odom_msg->pose.pose.position.y;
        current_odom(2) = odom_msg->pose.pose.position.z;
    }



    void update_circle_prior_position_by_realtive_position(const Eigen::Vector3d &detected_circle_center)
    {
        if (!config.enable_relative) return;

        Eigen::Vector3d corresponding_circle;
        double min_dis = 1e10;
        int corresponding_circle_index = 0;
        for (size_t k = 0; k < gate_list.size(); k++)
        {
            double distance = (gate_list[k] - detected_circle_center).norm();
            if (distance < min_dis)
            {
                corresponding_circle = gate_list[k];
                min_dis = distance;
                corresponding_circle_index = k;
            }
        }

        if (min_dis < config.corresponding_min_dis && (current_odom - corresponding_circle).norm() > config.update_min_dis)
        {   
            Eigen::Vector3d tmp = gate_list_[corresponding_circle_index] - detected_circle_center; 
            drift = tmp;
            geometry_msgs::Pose loop_msg;
            loop_msg.position.x = tmp(0);
            loop_msg.position.y = tmp(1);
            loop_msg.position.z = tmp(2);
            detect_loop_pub.publish(loop_msg);
            std::cout << "[circle odom debug drift] :   " << tmp.transpose() << std::endl;
            
            if (corresponding_circle_index == 4)
            {
                detect_loop_pub_for_dyo.publish(loop_msg);
                ROS_WARN_STREAM("[detect_loop_pub_for_dyo] :  " << drift.transpose());
            }


            double dif = (gate_list[corresponding_circle_index] - detected_circle_center).norm();

            // geometry_msgs::PoseStamped debug_msg;
            // debug_msg.pose.position.x = dif;
            // debug_pub.publish(debug_msg);

            gate_list[corresponding_circle_index] = detected_circle_center;
            for (size_t index = corresponding_circle_index; index < gate_list.size(); index++)
            {
                if (index == gate_list.size() - 1)
                {
                    transition_gate = gate_list[index] + transition_circle_relative_2_circle_7_;
                    rotation_gate = gate_list[index] + rotation_circle_relative_2_circle_7_;
                    alien_gate = rotation_gate + alien_circle_relative_2_rotation_circle_;
                }
                else
                {
                    gate_list[index+1] = gate_list[index] + circle_relative_position[index];
                }
            }
            return;
        }
        
        if ((detected_circle_center - rotation_gate).norm() < 3 && (current_odom - corresponding_circle).norm() > config.update_min_dis)
        {
            Eigen::Vector3d tmp = rotation_gate_ - detected_circle_center; 
            drift = tmp;
            std::cout << "[circle odom debug drift] :   " << tmp.transpose() << std::endl;

            geometry_msgs::Pose loop_msg;
            loop_msg.position.x = tmp(0);
            loop_msg.position.y = tmp(1);
            loop_msg.position.z = tmp(2);
            detect_loop_pub.publish(loop_msg);

            std::cout << "[circle odom debug before update alien] " << alien_gate.transpose() << std::endl;
            rotation_gate = detected_circle_center;
            alien_gate = rotation_gate + alien_circle_relative_2_rotation_circle_;
            std::cout << "[circle odom debug after update alien] " << alien_gate.transpose() << std::endl;
            std::cout << "[circle odom debug [alien] circle updated!!!]" << std::endl;
        }
    }


    Eigen::Matrix3d ComputeCovarianceMatrix(const std::vector<Eigen::Vector3d>& points)
    {
        Eigen::Matrix3d covariance ;
        covariance.setZero() ;
        Eigen::Vector3d avg(0.0,0.0,0.0);
        std::vector<Eigen::Vector3d> tempPoints = points ; 
        const int POINTNUMS = tempPoints.size();
        for(int i = 0 ; i < POINTNUMS ; ++i)
        {
            avg +=  tempPoints[i];
        }
        avg /= static_cast<double>(POINTNUMS);
        
        for(int i = 0 ; i < POINTNUMS ; ++i){tempPoints[i] -= avg;}

        for(int i = 0 ; i < 3 ; ++i)
        {
            for(int j = i ; j < 3 ; ++j)
            {
                for(int k = 0 ; k < POINTNUMS ; ++k)
                {
                    covariance(i,j) += tempPoints[k](i)*tempPoints[k](j);
                }
                covariance(i,j)/=POINTNUMS;
                covariance(j,i) = covariance(i,j);
            }
        }
        return covariance;
    }



    /*
     * Distinguish circle and digit by clustering
     * 1.cal center of both circle and digit
     * 2.compute covMatrix of circle to obtain base vector
     * 3.rotation center is located on the line connecting circlr center and digit center
    */
    bool Determine_the_center_of_the_rotation_by_clustered_pointcloud(const pcl::PointCloud<pcl::PointXYZ> &input_cloud,
                                                                    Eigen::Vector3d &center,
                                                                    Eigen::Vector3d &e1)
    {
        // clustering
        const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr = input_cloud.makeShared();
        pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
        kdtree->setInputCloud(input_cloud_ptr);
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> clustering;
        clustering.setClusterTolerance(config.r2_cluster_tolerance);
        clustering.setMinClusterSize(config.r2_cluster_min_size);
        clustering.setMaxClusterSize(5000);
        clustering.setSearchMethod(kdtree);
        clustering.setInputCloud(input_cloud_ptr);
        std::vector<pcl::PointIndices> clusters;
        clustering.extract(clusters);

        if(clusters.size() < 2)
        {
            ROS_ERROR_STREAM("[clusters size] :" << clusters.size() << "  This means circle and digit Not separated");
            return false;
        }

        for (size_t i = 0 ; i < clusters.size() ; i++ )
        {
            std::cout << "[circle odom debug]   clusters[" << i << "] : " << clusters[i].indices.size() << std::endl;
        }


        //cal rotation circle center
        Eigen::Vector3d circle_center(0,0,0);
        std::vector<Eigen::Vector3d> circle_points;
        for (auto index_iter = clusters[0].indices.begin() ; index_iter != clusters[0].indices.end() ; index_iter++)
        {
            Eigen::Vector3d tmp;
            tmp(0) = input_cloud_ptr->points[*index_iter].x;
            tmp(1) = input_cloud_ptr->points[*index_iter].y;
            tmp(2) = input_cloud_ptr->points[*index_iter].z;
            circle_points.push_back(tmp);
            circle_center += tmp;
        }

        circle_center = circle_center / clusters[0].indices.size();
        Determine_circle_center_by_pointcloud(circle_points,circle_center);

        std::cout << "[circle odom debug circle_center] : " << circle_center.transpose() << std::endl;

        Eigen::Matrix3d covMatrix = ComputeCovarianceMatrix(circle_points);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(covMatrix);
        Eigen::Matrix<double,3,1> evalue = eigen_solver.eigenvalues(); 
        Eigen::Matrix<double,3,3> evector = eigen_solver.eigenvectors();
        
        if(evector(0,0) > 0)  evector.col(0) = - evector.col(0);

        evector.col(0)(2) = 0;
        evector.col(0).normalize();
        e1 = evector.col(0).cross(Eigen::Vector3d(0,0,1));
        if (e1(0) > 0)   e1 = -e1;

        e1(2) = 0;
        e1.normalize();
        //cal digit center
        Eigen::Vector3d digit_center(0,0,0);
        std::vector<Eigen::Vector3d> digit_points;
        for (auto index_iter = clusters[1].indices.begin() ; index_iter != clusters[1].indices.end() ; index_iter++)
        {
            Eigen::Vector3d tmp;
            tmp(0) = input_cloud_ptr->points[*index_iter].x;
            tmp(1) = input_cloud_ptr->points[*index_iter].y;
            tmp(2) = input_cloud_ptr->points[*index_iter].z;
            digit_center += tmp;
            digit_points.push_back(tmp);
        }
        digit_center = digit_center / clusters[1].indices.size();
        std::cout << "[circle odom debug digit_center] : " << digit_center.transpose() << std::endl;

        //cal rotation center
        Eigen::Vector3d direc = digit_center - circle_center;
        direc.normalize();
        center = (circle_center + direc * 2);
        std::cout << "[circle odom debug estimated rotation origin]  : " << center.transpose() << std::endl;
        std::cout << "[circle odom debug e1]" << e1.transpose() << std::endl;

        visualizer.vis_orientation(center,e1,circle_orientation_vis);

        if(center(2) < 0.5 - drift(2))
        {
            visualizer.vis_pointcloud(circle_points,vis_R_circle_pub);
            visualizer.vis_pointcloud(digit_points,digit_vis_pub);
            return false;
        }

        //vis digit cluster
        pcl::PointCloud<pcl::PointXYZ> digit_cluster;
        for (auto k = clusters[1].indices.begin() ; k != clusters[1].indices.end() ; k++)
        {
            pcl::PointXYZ pt;
            pt.x = input_cloud_ptr->points[*k].x;
            pt.y = input_cloud_ptr->points[*k].y;
            pt.z = input_cloud_ptr->points[*k].z;
            digit_cluster.push_back(pt);

        }
        pcl::PointXYZ pt;
        pt.x = center(0);
        pt.y = center(1);
        pt.z = center(2);
        digit_cluster.push_back(pt);
        visualizer.vis_pointcloud(digit_cluster,digit_vis_pub);
        return true;
    }

        /*1.clustering all input points
        * 2.get each cluster content and compute covMatrix ,eigenvalue
        * 3.filtering by eigenvalue and min distance 2 it's cluster center
        */
    bool  getCircleclusterPos(const pcl::PointCloud<pcl::PointXYZ> &input_cloud,std::vector<Eigen::Vector3d> &result_cluster,
                            Eigen::Vector3d &cluster_origin,const double RATIO1_L, const double RATIO1_U,const double RATIO2 ,
                            const double min_accept_radius,const double min_leaf_size,const double cluster_tolerance,
                            const double min_cluster_size,const double max_cluster_size)
    {
        if(input_cloud.empty()) return false;


        const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = input_cloud.makeShared();

        //voxel filter
        pcl::PointCloud<pcl::PointXYZ> cloud_after_filter;
        pcl::VoxelGrid<pcl::PointXYZ> voxel_sampler;
        voxel_sampler.setLeafSize(min_leaf_size, min_leaf_size, min_leaf_size);
        voxel_sampler.setInputCloud(cloud);
        voxel_sampler.filter(cloud_after_filter);

        const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_ptr = cloud_after_filter.makeShared();

        // clustering
        pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
        kdtree->setInputCloud(cloud_filtered_ptr);
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> clustering;
        clustering.setClusterTolerance(cluster_tolerance);
        clustering.setMinClusterSize(min_cluster_size);
        clustering.setMaxClusterSize(max_cluster_size);
        clustering.setSearchMethod(kdtree);
        clustering.setInputCloud(cloud_filtered_ptr);
        std::vector<pcl::PointIndices> clusters;
        clustering.extract(clusters);

        std::vector<std::vector<Eigen::Vector3d>> potential_result_vec;
        bool flag_potential = false;        
        /*
        * get each cluster content
        * compute covMatrix ,eigenvalue
        * filtering by eigenvalue
        */
        for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
        {
            std::vector<Eigen::Vector3d> cluster;
            for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
            {
                Eigen::Vector3d pt(cloud_filtered_ptr->points[*point].x,cloud_filtered_ptr->points[*point].y,cloud_filtered_ptr->points[*point].z);
                cluster.push_back(pt);
            }

            Eigen::Matrix3d covMatrix = ComputeCovarianceMatrix(cluster);
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(covMatrix);
            Eigen::Matrix<double,3,1> evalue = eigen_solver.eigenvalues(); //升序排列
            Eigen::Matrix<double,3,3> evector = eigen_solver.eigenvectors();
            double ratio1 = evalue(1) / (evalue(2) +0.001);
            double ratio2 = evalue(1) / (evalue(0) +0.0001);

            std::cout << "[circle odom debug evalue] : " << evalue.transpose() << std::endl;
            std::cout << "[circle odom debug ratio1 & ratio2] : " << ratio1 << " , " << ratio2 << std::endl;

            if(ratio1 > RATIO1_L && ratio1 < RATIO1_U && ratio2 > RATIO2 )
            {
                potential_result_vec.push_back(cluster);
                flag_potential = true;
            }
        }

        // wxx
        std::vector<Eigen::Vector4d> vis_points;
        for(int i = 0; i < potential_result_vec.size(); i++)
        {
            for(auto pt:potential_result_vec[i])
            {
                Eigen::Vector4d temp_point;
                temp_point[0] = pt[0];
                temp_point[1] = pt[1];
                temp_point[2] = pt[2];
                temp_point[3] = i*10 + 1;

                vis_points.push_back(temp_point);
            }
        }
        visualizer.vis_pointcloud_intensity(vis_points,clusters_pt_vis);


        if(!flag_potential)  return false;

        //filtering clusters by distance 2 it's origin bigger than a specific threshhold
        std::vector<int> candicates;
        std::vector<Eigen::Vector3d> cluster_origin_vec;
        pcl::PointCloud<pcl::PointXYZ> pt_for_vis_debug;
        for(size_t i = 0 ; i < potential_result_vec.size() ;i++)
        {
            std::vector<Eigen::Vector3d> potential_result = potential_result_vec[i];
            Eigen::Vector3d cluster_origin(0,0,0);
            for(size_t j = 0 ;j < potential_result.size();j++)
            {
                Eigen::Vector3d potential_point = potential_result[j];
                pcl::PointXYZ ptt;
                ptt.x = potential_point(0);
                ptt.y = potential_point(1);
                ptt.z = potential_point(2);
                pt_for_vis_debug.points.push_back(ptt);
                cluster_origin = cluster_origin + potential_point;
            }
            cluster_origin = cluster_origin / potential_result.size();
            cluster_origin_vec.push_back(cluster_origin);

            bool potential_cluster_flag = true;
            for(size_t n = 0 ;n < potential_result.size();n++)
            {
                Eigen::Vector3d tmp = potential_result[n];
                double dis2origin = (tmp - cluster_origin).norm(); 
                if(dis2origin < min_accept_radius)  
                {
                    potential_cluster_flag = false;
                    break;
                }
            }
            if(potential_cluster_flag)  candicates.push_back(i);
        }

        if(candicates.empty()) return false;

        for (int i = 0;i < gate_list.size();i++)
        {
            std::cout << "[circle odom debug] gate_list[" << i << "] = " << gate_list[i].transpose() << std::endl;
        }

        for (int i = 0; i < cluster_origin_vec.size(); i++)
        {
            std::cout << "[circle odom debug] cluster_origin_vec[" << i << "] = " << cluster_origin_vec[i].transpose() << std::endl;
        }

        for (int i = 0; i < candicates.size(); i++)
        {
            std::cout << "[circle odom debug] candicates[" << i << "] = " << candicates[i] << std::endl;
        }
        //choose the closest one
        double min_distance = 1e10;
        int final_index;
        for (size_t k = 0;k < candicates.size();k++)
        {
            final_index = candicates[k];

            for (size_t m = 0; m < gate_list.size(); m++)
            {
                double tmp_dis = (cluster_origin_vec[final_index] - gate_list[m]).norm();
                std::cout << "[circle odom debug] cluster_origin_vec[" << final_index << "] to gate[" << m << "] = " << tmp_dis << std::endl;
                if (min_distance > tmp_dis)
                {
                    min_distance = tmp_dis;
                    result_cluster = potential_result_vec[final_index];
                    cluster_origin = cluster_origin_vec[final_index];
                }
            }
        }
        if (min_distance < 2) return true;
        

        min_distance = 1e10;
        for (size_t k = 0;k < candicates.size();k++)
        {
            final_index = candicates[k];
            double tmp_dis = (cluster_origin_vec[final_index] - transition_gate).norm();
            if (min_distance > tmp_dis)
            {
                min_distance = tmp_dis;
                result_cluster = potential_result_vec[final_index];
                cluster_origin = cluster_origin_vec[final_index];
            }
        }
        if (min_distance < 6) return true;

        min_distance = 1e10;
        for (size_t k = 0;k < candicates.size();k++)
        {
            final_index = candicates[k];
            double tmp_dis = (cluster_origin_vec[final_index] - rotation_gate).norm();
            if (min_distance > tmp_dis)
            {
                min_distance = tmp_dis;
                result_cluster = potential_result_vec[final_index];
                cluster_origin = cluster_origin_vec[final_index];
            }
        }
        if (min_distance < 6) return true;

        min_distance = 1e10;
        for (size_t k = 0;k < candicates.size();k++)
        {
            final_index = candicates[k];
            double tmp_dis = (cluster_origin_vec[final_index] - alien_gate).norm();
            if (min_distance > tmp_dis)
            {
                min_distance = tmp_dis;
                result_cluster = potential_result_vec[final_index];
                cluster_origin = cluster_origin_vec[final_index];
            }
        }
        if (min_distance < 3) return true;

        return false;
    }


    void dealWithRotation(const std::vector<Eigen::Vector3d> &rotation_circle_pos_vec,const std::vector<ros::Time> &time_stamp_vec)
    {
        if(first_rotation)
        {
            t_fisrt_rotation = time_stamp_vec.back();
            first_rotation = false;
        }
        
        std::cout << std::setprecision(19) << "[the gap between current time and last rotation circle time ]" <<ros::Time::now().toSec()-  time_stamp_vec.back().toSec() << std::endl;
        std::cout << std::setprecision(19) << "[dealWithRotation]     " << time_stamp_vec.back().toSec() 
                                                                    << std::setprecision(7) << std::endl;
        Eigen::Vector3d circle_pos = rotation_circle_pos_vec.back();
        Eigen::Vector3d V = circle_pos - rotation_origin ;
        Eigen::Vector2d proj2plane;
        e1.normalize();
        proj2plane(0) = e1.dot(V);
        proj2plane(1) = V(2);
        proj2plane.normalize();

        theta0 = std::atan2(proj2plane(1), proj2plane(0))  -  w * (time_stamp_vec.back().toSec()-t_fisrt_rotation.toSec()) ;

        int cycle = theta0 /2 /M_PI;
        theta0 = theta0 - cycle * 2 *M_PI;
    }



    bool dealWithTransition(std::vector<std::pair<Eigen::Vector3d,ros::Time>> &T_pos_stamp_pair_vec,double &vel,double &left_endpoint_y)
    {
        static int last_vel_direction = 0;
        static std::vector<double> vel_vec;
        static double absolute_mean_vel = 0.0;
        static int continue_count = 0;
        static double last_vel;
        static bool reverse_flag = false;
        double  curr_pos_y = T_pos_stamp_pair_vec.back().first(1);

        static double max_y = -1.0e10;
        static double min_y = 1.0e10;
        max_y = curr_pos_y > max_y ? curr_pos_y : max_y;
        min_y = curr_pos_y < min_y ? curr_pos_y : min_y;

        std::vector<std::pair<double, int>> dist;
        double length = 0.0;
        double time_length = 0.0;
        for (int i = 5; i < T_pos_stamp_pair_vec.size() - 1; i++)
        {
            double tmp_time = (T_pos_stamp_pair_vec[i + 1].second - T_pos_stamp_pair_vec[i].second).toSec();
            if ( tmp_time >= config.time_tolerance) 
             continue;

            time_length += tmp_time;
            double tmp_dist = T_pos_stamp_pair_vec[i + 1].first(1) - T_pos_stamp_pair_vec[i].first(1);   
            length += std::abs(tmp_dist);
        }
        vel = length / time_length;

        // vel direction
        // length = 0.0;
        // for (int i = T_pos_stamp_pair_vec.size()-1, k = 0; k < direction_count ;k++)
        // {
        //     double tmp_dist = T_pos_stamp_pair_vec[i-k].first(1) - T_pos_stamp_pair_vec[i-k-1].first(1);  
        //     length += std::abs(tmp_dist);
        //     int tmp_direction = tmp_dist > 0 ? 1 : -1;
        //     if (tmp_direction == 1) count++;
        //     else count--;
        // }
        // int current_index = T_pos_stamp_pair_vec.size()-1;
        // double first_end_dist = T_pos_stamp_pair_vec[current_index].first(1) - T_pos_stamp_pair_vec[current_index - direction_count].first(1);  
        // if (first_end_dist > ( length*0.5 ))
        // {
        //     current_vel_direction = last_vel_direction;
        // }
        // else
        // {
        //     // current_vel_direction = count > 0 ? 1:-1;
        //     current_vel_direction = -last_vel_direction;
        // }

        // change direction or not
        double local_max_y = -1.0e10;
        double local_min_y = 1.0e10;
        int change_count = 0;
        int direction_count = 6;
        int count = 0;
        bool change_direction = false;
        for (int i = 0; i < T_pos_stamp_pair_vec.size() - direction_count; i++)
        {
            double temp_curr_pos_y = T_pos_stamp_pair_vec[i].first(1);
            // local_max_y = temp_curr_pos_y > local_max_y ? temp_curr_pos_y : local_max_y;
            // local_min_y = temp_curr_pos_y < local_min_y ? temp_curr_pos_y : local_min_y;

            if (temp_curr_pos_y > local_max_y)
            {
                local_max_y = temp_curr_pos_y;
                count++;
            }

            if (temp_curr_pos_y < local_min_y)
            {
                local_min_y = temp_curr_pos_y;
                count--;
            }
        }
        for (int i = T_pos_stamp_pair_vec.size()-direction_count; i < T_pos_stamp_pair_vec.size() ;i++)
        {
            double temp_curr_pos_y = T_pos_stamp_pair_vec[i].first(1);
            if( temp_curr_pos_y > local_min_y && temp_curr_pos_y < local_max_y  )
                change_count++;

            local_max_y = temp_curr_pos_y > local_max_y ? temp_curr_pos_y : local_max_y;
            local_min_y = temp_curr_pos_y < local_min_y ? temp_curr_pos_y : local_min_y;
            // local_min_y = temp_curr_pos_y < local_min_y ? temp_curr_pos_y : local_min_y;
        }
        int current_vel_direction = last_vel_direction;
        if ( change_count >= ( direction_count - 2 ) )
        {
            change_direction = true;
            int temp_direction = count > 0 ? 1:-1;
            current_vel_direction = -temp_direction;
            last_vel_direction = current_vel_direction;
        }

        geometry_msgs::PoseStamped debug_msg;
        debug_msg.pose.position.x = vel;
        debug_msg.pose.position.y = change_count;
        debug_pub.publish(debug_msg);


        bool flag = false;

        if (reverse_flag)
        {
            vel_vec.push_back(vel);
            absolute_mean_vel = absolute_mean_vel + 1.0 / vel_vec.size() * (std::abs(vel) - absolute_mean_vel);
        }

        for (size_t k = 0;k < T_pos_stamp_pair_vec.size() ;k++)
        {
            std::cout << "[circle odom debug] T_pos_stamp_pair_vec[" << k << "] : " << T_pos_stamp_pair_vec[k].first(1) << std::endl;
        }

        std::cout << "[circle odom debug    diff_t]  :   " << ( T_pos_stamp_pair_vec.back().second - T_pos_stamp_pair_vec.begin()->second).toSec() << std::endl;
        std::cout << "[circle odom debug    p_k+1]  :   " << T_pos_stamp_pair_vec.back().first(1) << std::endl;
        std::cout << "[circle odom debug    p_k]  :   " << T_pos_stamp_pair_vec.begin()->first(1) << std::endl;
        std::cout << "[circle odom debug    diff_p]  :   " << (T_pos_stamp_pair_vec.back().first(1) - T_pos_stamp_pair_vec.begin()->first(1) ) << std::endl;
        std::cout << "[circle odom debug    vel_vec size]   :   " << vel_vec.size() << std::endl;
        std::cout << "[circle odom debug    absolute_mean_vel]   :   " << absolute_mean_vel << std::endl;
        std::cout << "[circle odom debug    before vel]   :   " << vel << std::endl;
        // vel = current_vel_direction * absolute_mean_vel;
        vel = current_vel_direction * vel;
        
        std::cout << "[circle odom debug    after vel]   :   " << vel << std::endl;

        // if (reverse_flag)
        {
            if ( std::abs(last_vel - std::abs(vel)) < config.vel_tolerance)
            {
                continue_count++;
            }
            else continue_count = 0;
        }

        //vel reverse occur, then we believe vel will not reverse in the next short time
        // if ((last_vel_direction * current_vel_direction) == -1)
        if ( change_direction )
        {
            if (!reverse_flag)
            {       
                // vel_vec.clear();
                // absolute_mean_vel = std::abs(vel);
                vel_vec.push_back(vel);
                last_vel = std::abs(vel);
                reverse_flag = true;
            }

            flag = true;
        }

        last_vel = std::abs(vel);
        //In truth region
        if (std::abs(curr_pos_y - transition_gate(1)) < config.truth_region)
        {
            flag = true;
        }

        if ((curr_pos_y - transition_gate(1)) < 0 && current_vel_direction > 0)
        {
            flag = true;
        }
        if ((curr_pos_y - transition_gate(1)) > 0 && current_vel_direction < 0)
        {
            flag = true;
        }

        left_endpoint_y = current_vel_direction > 0 ? min_y + 6.54 : max_y;
        flag = (continue_count > config.max_continue_count) && flag;

        last_vel_direction = current_vel_direction;
        return flag;
    }


    bool check_eigenvalue_constraints(const Eigen::Vector3d& eigenvalues) 
    {
        int pos(0), neg(0);
        if(eigenvalues(0) > 0) 
            { ++pos;} 
        else if(eigenvalues(0) < 0) 
            { ++neg; }
        else 
        { std::cerr << "Eigenvalue equal to zero" << std::endl; }

        if(eigenvalues(1) > 0) 
            { ++pos;} 
        else if(eigenvalues(1) < 0) 
            { ++neg; }
        else 
            { std::cerr << "Eigenvalue equal to zero" << std::endl; }

        if(eigenvalues(2) > 0) 
            { ++pos;} 
        else if(eigenvalues(2) < 0) 
            { ++neg; }
        else 
            { std::cerr << "Eigenvalue equal to zero" << std::endl; }
        
        if(pos == 2 && neg == 1) 
            { return true; }
        return false;
    }

    
    Eigen::Matrix3d params2EllipseMat(const Eigen::Vector2d &center,const Eigen::Vector2d &semi_axis, 
                                    const double &angle,std::vector<cv::Point>&ellipse_candidate)
    {
        double B2 = semi_axis(1) * semi_axis(1) /4;
        double A2 = semi_axis(0) * semi_axis(0) /4;
        Eigen::Matrix3d ellipseMat = Eigen::Matrix3d::Zero();

        ellipseMat(0,0) = 1.0/A2;
        ellipseMat(1,1) = 1.0/B2;
        ellipseMat(2,2) = -1.0;

        double cosa = std::cos(angle*M_PI/180.0);
        double sina = std::sin(angle*M_PI/180.0);
        Eigen::Matrix3d rotation;
        rotation << cosa,-sina,  center(0),
                    sina, cosa,  center(1),
                    0,    0,     1;
        auto H = rotation.inverse();

        return H.transpose() * ellipseMat * H;
    }


    bool get_circle_pose(const Eigen::Matrix3d &pMat,const Eigen::Matrix3d &Ellipse
                            ,Eigen::Vector3d &pos1, Eigen::Vector3d &normal1
                            ,Eigen::Vector3d &pos2, Eigen::Vector3d &normal2)
    {
        Eigen::Matrix3d Ellipse_ = pMat.transpose() * Ellipse * pMat;

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(Ellipse_);
        Eigen::Vector3d eigenvalues = eigen_solver.eigenvalues();
        Eigen::Matrix3d eigenvector = eigen_solver.eigenvectors();
        if(!check_eigenvalue_constraints(eigenvalues)) 
        {
            eigen_solver = Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(-Ellipse_);
            eigenvalues = eigen_solver.eigenvalues();
            eigenvector = eigen_solver.eigenvectors();
            if(!check_eigenvalue_constraints(eigenvalues)) 
            {
                std::cout << "eigenvalues = " << eigenvalues.transpose() << std::endl;
                std::cerr << "Eigenvales does not macth with the ellipse constraints"
                        << std::endl;
                return false;
            }
        }
        // std::cout << "eigenvalues = " << eigenvalues.transpose() << std::endl;
        double lambda1 = eigenvalues(2);
        double lambda2 = eigenvalues(1);
        double lambda3 = eigenvalues(0);
        
        Eigen::Matrix3d H;
        H.col(0) = eigenvector.col(2);
        H.col(1) = eigenvector.col(1);
        H.col(2) = eigenvector.col(0);

        double cosqs = (lambda2 - lambda3) / (lambda1 - lambda3);
        double cosq = std::sqrt(cosqs);
        double sinq = std::sqrt(1.0-cosqs);

        // std::cout << "----cosq = " << cosq << "  , sinq = " << sinq << std::endl;
        // std::cout << "---H = " << std::endl << H << std::endl;
        normal1 = H * Eigen::Vector3d(-sinq, 0, cosq);
        normal1 = normal1.normalized();
        normal2 = -normal1;

        double kk = config.a_radius / sqrt(-lambda1*lambda3);
        pos1 = H * Eigen::Vector3d(-lambda3*sinq, 0, lambda1*cosq);
        pos1 *= kk;
        pos2 = -pos1;

        return true;
    }


    bool Determine_circle_center_by_pointcloud(const std::vector<Eigen::Vector3d> &input_cloud,Eigen::Vector3d &circle_center)
    {
        if (input_cloud.empty()) return false;

        std::vector<Eigen::Vector3d> vis_point_vec;
        const int POINTNUMS = input_cloud.size();
        Eigen::Matrix3d covMatrix = ComputeCovarianceMatrix(input_cloud);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(covMatrix);
        Eigen::Matrix<double,3,1> evalue = eigen_solver.eigenvalues();
        Eigen::Matrix<double,3,3> evector = eigen_solver.eigenvectors();

        std::cout << "[circle odom debug evector] : " << std::endl << evector << std::endl;

        evector.col(0).normalize();
        evector.col(1).normalize();
        evector.col(2).normalize();

        Eigen::Vector3d  base1 = evector.col(1);
        Eigen::Vector3d  base2 = evector.col(2);

        double infinity = std::numeric_limits<float>::infinity();
        Eigen::Vector3d minExtents(infinity,infinity,infinity);
        Eigen::Vector3d maxExtents(-infinity,-infinity,-infinity);
        std::vector<Eigen::Vector3d> tempPoints = input_cloud;     
        Eigen::Vector3d left_endpoint;
        Eigen::Vector3d right_endpoint;
        Eigen::Vector3d middle_point;
        bool have_left = false;
        bool have_right = false;
        bool have_middle = false;
        for (int i = 0 ; i < POINTNUMS ; ++i)
        {
            Eigen::Matrix<double,3,1> displacement = tempPoints[i] -  circle_center;
            Eigen::Matrix<double,3,1> displacement_R = evector.transpose() * displacement;

            if (displacement_R(2) < minExtents(2))
            {
                have_left = true;
                minExtents(2) = displacement_R(2);
                left_endpoint = tempPoints[i];
            }
            else if (displacement_R(2) > maxExtents(2))
            {
                have_right = true;
                maxExtents(2) = displacement_R(2);
                right_endpoint = tempPoints[i];
            }

            if (displacement_R(1) < minExtents(1) && std::abs(displacement_R(2)) < 0.15)
            {
                have_middle = true;
                minExtents(1) = displacement_R(1);
                middle_point = tempPoints[i];
            }
            else if (displacement_R(1) > maxExtents(1) && std::abs(displacement_R(2)) < 0.15)
            {
                have_middle = true;
                maxExtents(1) = displacement_R(1);
                middle_point = tempPoints[i];
            }
        }

        visualizer.vis_orientation(circle_center,base1,circle_base1_vis);
        visualizer.vis_orientation(circle_center,base2,circle_base2_vis);
        if(have_left && have_right && have_middle)
        {
            Eigen::Vector3d tmp_left = evector.transpose() * (left_endpoint - circle_center);
            Eigen::Vector3d tmp_right = evector.transpose() * (right_endpoint - circle_center);
            Eigen::Vector3d tmp_middle = evector.transpose() * (middle_point - circle_center);

            Eigen::Vector2d middle1 = (tmp_left.tail<2>() + tmp_middle.tail<2>()) /2.0;
            Eigen::Vector2d normal1 = tmp_left.tail<2>() - tmp_middle.tail<2>();
            normal1 = Eigen::Vector2d(normal1(1),-normal1(0));

            Eigen::Vector2d middle2 = (tmp_right.tail<2>() + tmp_middle.tail<2>()) /2.0;
            Eigen::Vector2d normal2 = tmp_right.tail<2>() - tmp_middle.tail<2>();
            normal2 = Eigen::Vector2d(normal2(1),-normal2(0));

            double t_ = ((middle1(1) - middle2(1)) * normal2(0) - (middle1(0) - middle2(0)) * normal2(1)) / (normal1(0) * normal2(1) - normal1(1) * normal2(0)); 
            Eigen::Vector2d intersection = middle1 + normal1 * t_;

            circle_center = (evector.transpose()).inverse() * Eigen::Vector3d(0,intersection(0),intersection(1)) + circle_center;

            std::cout << "[circle odom debug t_] :   " << t_ << std::endl;
            std::cout << "[dedbug circle center]    :   " << circle_center.transpose() << std::endl;

            vis_point_vec.push_back(circle_center);
            vis_point_vec.push_back(left_endpoint);
            vis_point_vec.push_back(right_endpoint);
            vis_point_vec.push_back(middle_point);

            std::cout <<"[circle odom debug] circle_center : " << circle_center.transpose() << std::endl;
            std::cout <<"[circle odom debug] left_endpoint : " << left_endpoint.transpose() << std::endl;
            std::cout <<"[circle odom debug] right_endpoint : " << right_endpoint.transpose() << std::endl;
            std::cout <<"[circle odom debug] middle_point : " << middle_point.transpose() << std::endl;
            std::cout <<"[circle odom debug] circle_center2middle_point : " << (circle_center - middle_point).norm() << std::endl;
            std::cout <<"[circle odom debug] circle_center2left_endpoint : " << (circle_center - left_endpoint).norm() << std::endl;
            std::cout <<"[circle odom debug] circle_center2right_endpoint : " << (circle_center - right_endpoint).norm() << std::endl;

            visualizer.vis_pointcloud(vis_point_vec,partial_detect_result_vis);

            return true;
        }
        else return false;
    }


    // void vis_rotation_circle(const ros::TimerEvent &event)
    // {
    //     return;
    //     if(first_rotation) return;
    //     double t = ros::Time::now().toSec();
    //     if(first_rotation) return;
    //     // std::cout <<"[e1] : " << e1 << std::endl;
    //     // std::cout <<"[e2] : " << e2 << std::endl;
    //     Eigen::Vector3d circle_ori = rotation_origin + rotation_radius * (e1 * std::cos(w * (t - t_fisrt_rotation.toSec() - time_offset) + theta0) + 
    //                                                                     e2 * std::sin(w * (t - t_fisrt_rotation.toSec() - time_offset) + theta0));
    //     pcl::PointCloud<pcl::PointXYZ> pt_for_vis_debug;
    //     for(double theta = 0.0 ; theta < 2*M_PI ; theta = theta +0.02)
    //     {
    //         Eigen::Vector3d point_on_circle = circle_ori +  e1 * std::cos(theta) + e2 * std::sin(theta);
    //         pcl::PointXYZ pt;
    //         pt.x = point_on_circle(0);
    //         pt.y = point_on_circle(1);
    //         pt.z = point_on_circle(2);
    //         pt_for_vis_debug.push_back(pt);
    //     }
    //     pt_for_vis_debug.push_back(pcl::PointXYZ(circle_ori(0),circle_ori(1),circle_ori(2)));
    //     pt_for_vis_debug.push_back(pcl::PointXYZ(rotation_origin(0),rotation_origin(1),rotation_origin(2)));
    //     sensor_msgs::PointCloud2 circle_pt_vis_msg_;
    //     pcl::toROSMsg(pt_for_vis_debug,circle_pt_vis_msg_);
    //     circle_pt_vis_msg_.header.frame_id = "world";
    //     // rotation_fake_circle_vis.publish(circle_pt_vis_msg_);
    // }
};


#endif








<launch>
    <node pkg = "circle_detector" type = "circle_odom" name = "circle_odom0_00" output = "screen"> 
        <rosparam file="$(find circle_detector)/config/config.yaml" command="load" />
        <param name = "frame_delay"  value = "0.00" />
        <remap from = "/circle_odom" to = "/circle_odom0_00"/>
    </node>

        <node pkg = "circle_detector" type = "circle_odom" name = "circle_odom0_002" output = "screen"> 
        <rosparam file="$(find circle_detector)/config/config.yaml" command="load" />
        <param name = "frame_delay"  value = "0.002" />
        <remap from = "/circle_odom" to = "/circle_odom0_002"/>
    </node>

    <node pkg = "circle_detector" type = "circle_odom" name = "circle_odom0_004" output = "screen"> 
        <rosparam file="$(find circle_detector)/config/config.yaml" command="load" />
        <param name = "frame_delay"  value = "0.004" />
        <remap from = "/circle_odom" to = "/circle_odom0_004"/>
    </node>

    <node pkg = "circle_detector" type = "circle_odom" name = "circle_odom0_006" output = "screen"> 
        <rosparam file="$(find circle_detector)/config/config.yaml" command="load" />
        <param name = "frame_delay"  value = "0.006" />
        <remap from = "/circle_odom" to = "/circle_odom0_006"/>
    </node>


    <node pkg = "circle_detector" type = "circle_odom" name = "circle_odom0_008" output = "screen"> 
        <rosparam file="$(find circle_detector)/config/config.yaml" command="load" />
        <param name = "frame_delay"  value = "0.008" />
        <remap from = "/circle_odom" to = "/circle_odom0_008"/>
    </node>

    <node pkg = "circle_detector" type = "circle_odom" name = "circle_odom0_01" output = "screen"> 
        <rosparam file="$(find circle_detector)/config/config.yaml" command="load" />
        <param name = "frame_delay"  value = "0.01" />
        <remap from = "/circle_odom" to = "/circle_odom0_01"/>
    </node>
</launch>