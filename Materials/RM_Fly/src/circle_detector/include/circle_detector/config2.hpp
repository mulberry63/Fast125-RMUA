#ifndef _CONFIG2_
#define _CONFIG2_


#include<ros/ros.h>

struct Config2
{
    std::string depth_topic;
    std::string rgb_topic;
    std::string odom_topic;
    std::string loop_odom_topic;
    std::string detect_loop_topic;

    int max_odom_vec_size;
    int min_odom_vec_size;

    double fx;
    double fy;
    double cx;
    double cy;
    double cam2body_x;
    double cam2body_y;
    double cam2body_z;
    int skip_pixel;
    double min_depth;
    double max_depth;

    double cluster_leaf_size;
    double cluster_tolerance;
    int cluster_min_size;
    int cluster_max_size;
    
    double transition_circle_x_tolerance;
    double transition_circle_y_tolerance;
    double transition_circle_z_tolerance;
    int transition_vec_size;

    bool vis_debug_enable;
    bool enable_relative;
    bool enable_loopclosture;
    std::vector<double> circle_relative_postion;
    std::vector<double> transition_circle_relative_2_circle_7;
    std::vector<double> rotation_circle_relative_2_circle_7;
    std::vector<double> alien_circle_relative_2_rotation_circle;
    std::vector<double> gate_list_;
    std::vector<double>transition_gate;
    std::vector<double> rotation_gate;
    std::vector<double> alien_gate;
    std::vector<double> middle_of_transition_line;
    double vel_ratio_l;
    double vel_ratio_u;
    double truth_region;
    double vel_tolerance;
    double time_tolerance;
    int max_continue_count;
    double variance_tolerance;
    double frame_delay;
    double update_min_dis;
    double corresponding_min_dis;

    std::vector<double> rotation_origin;
    std::vector<double> e1;
    std::vector<double> normal;
    double rotation_radius;
    double rotation_circle_min_radius;

    double circle_min_search_radius;
    double ratio1_l;
    double ratio1_u;
    double ratio2;
    double min_accept_radius;

    double t_ratio1_l;
    double t_ratio1_u;
    double t_ratio2;
    double t_min_accept_radius;

    double r_circle_min_search_radius;
    double r_ratio1_l;
    double r_ratio1_u;
    double r_ratio2;
    double r_min_accept_radius;

    double r1_cluster_leaf_size;
    double r1_cluster_tolerance;
    double r1_cluster_min_size;

    double r2_cluster_tolerance;
    double r2_cluster_min_size;


    double a_circle_min_search_radius;
    double a_ratio1_l;
    double a_ratio1_u;
    double a_ratio2;
    double a_min_accept_radius;
    double a_cluster_leaf_size;
    double a_cluster_tolerance;
    double a_radius;
    double a_result_tolerance;
    void init(const ros::NodeHandle &nh)
    {
        nh.getParam("depth_topic",depth_topic);
        nh.getParam("rgb_topic",rgb_topic);
        nh.getParam("odom_topic",odom_topic);
        nh.getParam("loop_odom_topic",loop_odom_topic);
        nh.getParam("detect_loop_topic",detect_loop_topic);

        nh.getParam("max_odom_vec_size",max_odom_vec_size);
        nh.getParam("min_odom_vec_size",min_odom_vec_size);

        nh.getParam("fx",fx);
        nh.getParam("fy",fy);
        nh.getParam("cx",cx);
        nh.getParam("cy",cy);
        nh.getParam("cam2body_x",cam2body_x);
        nh.getParam("cam2body_y",cam2body_y);
        nh.getParam("cam2body_z",cam2body_z);
        nh.getParam("skip_pixel",skip_pixel);
        nh.getParam("min_depth",min_depth);
        nh.getParam("max_depth",max_depth);
        
        nh.getParam("cluster_leaf_size",cluster_leaf_size);
        nh.getParam("cluster_tolerance",cluster_tolerance);
        nh.getParam("cluster_min_size",cluster_min_size);
        nh.getParam("cluster_max_size",cluster_max_size);
        
        nh.getParam("transition_circle_x_tolerance",transition_circle_x_tolerance);
        nh.getParam("transition_circle_y_tolerance",transition_circle_y_tolerance);
        nh.getParam("transition_circle_z_tolerance",transition_circle_z_tolerance);
        nh.getParam("transition_vec_size",transition_vec_size);

        nh.getParam("vis_debug_enable",vis_debug_enable);
        nh.getParam("enable_loopclosture",enable_loopclosture);
        nh.getParam("enable_relative",enable_relative);
        nh.getParam("circle_relative_postion",circle_relative_postion);
        nh.getParam("transition_circle_relative_2_circle_7",transition_circle_relative_2_circle_7);
        nh.getParam("rotation_circle_relative_2_circle_7",rotation_circle_relative_2_circle_7);
        nh.getParam("alien_circle_relative_2_rotation_circle",alien_circle_relative_2_rotation_circle);
        nh.getParam("gate_list_",gate_list_);
        nh.getParam("transition_gate",transition_gate);
        nh.getParam("rotation_gate",rotation_gate);
        nh.getParam("alien_gate",alien_gate);
        nh.getParam("middle_of_transition_line",middle_of_transition_line);
        nh.getParam("vel_ratio_l",vel_ratio_l);
        nh.getParam("max_continue_count",max_continue_count);
        nh.getParam("truth_region",truth_region);
        nh.getParam("vel_tolerance",vel_tolerance);
        nh.getParam("time_tolerance",time_tolerance);
        nh.getParam("variance_tolerance",variance_tolerance);
        nh.getParam("frame_delay",frame_delay);
        nh.getParam("update_min_dis",update_min_dis);
        nh.getParam("corresponding_min_dis",corresponding_min_dis);
        
        nh.getParam("rotation_origin",rotation_origin);
        nh.getParam("e1",e1);
        nh.getParam("normal",normal);
        nh.getParam("rotation_radius",rotation_radius);
        nh.getParam("rotation_circle_min_radius",rotation_circle_min_radius);

        nh.getParam("circle_min_search_radius",circle_min_search_radius);
        nh.getParam("ratio1_l",ratio1_l);
        nh.getParam("ratio1_u",ratio1_u);
        nh.getParam("ratio2",ratio2);
        nh.getParam("min_accept_radius",min_accept_radius);

        nh.getParam("t_ratio1_l",t_ratio1_l);
        nh.getParam("t_ratio1_u",t_ratio1_u);
        nh.getParam("t_ratio2",t_ratio2);
        nh.getParam("t_min_accept_radius",t_min_accept_radius);

        nh.getParam("r_circle_min_search_radius",r_circle_min_search_radius);
        nh.getParam("r_ratio1_l",r_ratio1_l);
        nh.getParam("r_ratio1_u",r_ratio1_u);
        nh.getParam("r_ratio2",r_ratio2);
        nh.getParam("r_min_accept_radius",r_min_accept_radius);


        nh.getParam("r1_cluster_leaf_size",r1_cluster_leaf_size);
        nh.getParam("r1_cluster_tolerance",r1_cluster_tolerance);
        nh.getParam("r1_cluster_min_size",r1_cluster_min_size);

        nh.getParam("r2_cluster_tolerance",r2_cluster_tolerance);
        nh.getParam("r2_cluster_min_size",r2_cluster_min_size);

        nh.getParam("a_circle_min_search_radius",a_circle_min_search_radius);
        nh.getParam("a_ratio1_l",a_ratio1_l);
        nh.getParam("a_ratio1_u",a_ratio1_u);
        nh.getParam("a_ratio2",a_ratio2);
        nh.getParam("a_min_accept_radius",a_min_accept_radius);
        nh.getParam("a_cluster_leaf_size",a_cluster_leaf_size);
        nh.getParam("a_cluster_tolerance",a_cluster_tolerance);
        nh.getParam("a_radius",a_radius);
        nh.getParam("a_result_tolerance",a_result_tolerance);
    }
};


#endif