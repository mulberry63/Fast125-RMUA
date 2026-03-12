#ifndef _CONFIG_
#define _CONFIG_


#include<ros/ros.h>

struct Config
{
    std::string depth_topic;
    std::string odom_topic;

    int max_odom_vec_size;
    int min_odom_vec_size;
    
    double ekf_rate;
    double ekf_Q_p;
    double ekf_Q_v;
    double ekf_R_p;
    double ekf_R_v;

    double fx;
    double fy;
    double cx;
    double cy;
    double cam2body_x;
    double cam2body_y;
    double cam2body_z;
    int skip_pixel;

    double g_cluster_leaf_size;
    double g_cluster_tolerance;
    int g_cluster_min_size;
    int g_cluster_max_size;

    double transition_circle_box_min_x;
    double transition_circle_box_min_y;
    double transition_circle_box_min_z;
    double transition_circle_box_max_x;
    double transition_circle_box_max_y;
    double transition_circle_box_max_z;


    std::vector<double> gate_list_;
    std::vector<double>transition_gate;
    std::vector<double> rotation_gate;
    std::vector<double> alien_gate;
    double frame_delay;

    std::vector<double> rotation_origin;
    std::vector<double> e1;
    std::vector<double> normal;
    double rotation_radius;
    double rotation_circle_min_radius;

    double g_circle_min_search_radius;
    double g_ratio1_l;
    double g_ratio1_u;
    double g_ratio2;
    double g_min_accept_radius;

    double t_ratio1_l;
    double t_ratio1_u;
    double t_ratio2;
    double t_min_accept_radius;

    double r_circle_min_search_radius;
    double r_ratio1_l;
    double r_ratio1_u;
    double r_ratio2;
    double r_min_accept_radius;

    double a_circle_min_search_radius;
    double a_ratio1_l;
    double a_ratio1_u;
    double a_ratio2;
    double a_min_accept_radius;

    void init(const ros::NodeHandle &nh)
    {
        nh.getParam("depth_topic",depth_topic);
        nh.getParam("odom_topic",odom_topic);

        nh.getParam("max_odom_vec_size",max_odom_vec_size);
        nh.getParam("min_odom_vec_size",min_odom_vec_size);

        nh.getParam("ekf_rate",ekf_rate);
        nh.getParam("ekf_Q_p",ekf_Q_p);
        nh.getParam("ekf_Q_v",ekf_Q_v);
        nh.getParam("ekf_R_p",ekf_R_p);
        nh.getParam("ekf_R_v",ekf_R_v);

        nh.getParam("fx",fx);
        nh.getParam("fy",fy);
        nh.getParam("cx",cx);
        nh.getParam("cy",cy);
        nh.getParam("cam2body_x",cam2body_x);
        nh.getParam("cam2body_y",cam2body_y);
        nh.getParam("cam2body_z",cam2body_z);
        nh.getParam("skip_pixel",skip_pixel);
        
        nh.getParam("g_cluster_leaf_size",g_cluster_leaf_size);
        nh.getParam("g_cluster_tolerance",g_cluster_tolerance);
        nh.getParam("g_cluster_min_size",g_cluster_min_size);
        nh.getParam("g_cluster_max_size",g_cluster_max_size);
        
        nh.getParam("transition_circle_box_min_x",transition_circle_box_min_x);
        nh.getParam("transition_circle_box_min_y",transition_circle_box_min_y);
        nh.getParam("transition_circle_box_min_z",transition_circle_box_min_z);
        nh.getParam("transition_circle_box_max_x",transition_circle_box_max_x);
        nh.getParam("transition_circle_box_max_y",transition_circle_box_max_y);
        nh.getParam("transition_circle_box_max_z",transition_circle_box_max_z);

        nh.getParam("gate_list_",gate_list_);
        nh.getParam("transition_gate",transition_gate);
        nh.getParam("rotation_gate",rotation_gate);
        nh.getParam("alien_gate",alien_gate);
        nh.getParam("frame_delay",frame_delay);
        
        nh.getParam("rotation_origin",rotation_origin);
        nh.getParam("e1",e1);
        nh.getParam("normal",normal);
        nh.getParam("rotation_radius",rotation_radius);
        nh.getParam("rotation_circle_min_radius",rotation_circle_min_radius);

        nh.getParam("g_circle_min_search_radius",g_circle_min_search_radius);
        nh.getParam("g_ratio1_l",g_ratio1_l);
        nh.getParam("g_ratio1_u",g_ratio1_u);
        nh.getParam("g_ratio2",g_ratio2);
        nh.getParam("g_min_accept_radius",g_min_accept_radius);

        nh.getParam("t_ratio1_l",t_ratio1_l);
        nh.getParam("t_ratio1_u",t_ratio1_u);
        nh.getParam("t_ratio2",t_ratio2);
        nh.getParam("t_min_accept_radius",t_min_accept_radius);

        nh.getParam("r_circle_min_search_radius",r_circle_min_search_radius);
        nh.getParam("r_ratio1_l",r_ratio1_l);
        nh.getParam("r_ratio1_u",r_ratio1_u);
        nh.getParam("r_ratio2",r_ratio2);
        nh.getParam("r_min_accept_radius",r_min_accept_radius);

        nh.getParam("a_circle_min_search_radius",a_circle_min_search_radius);
        nh.getParam("a_ratio1_l",a_ratio1_l);
        nh.getParam("a_ratio1_u",a_ratio1_u);
        nh.getParam("a_ratio2",a_ratio2);
        nh.getParam("a_min_accept_radius",a_min_accept_radius);
    }
};


#endif