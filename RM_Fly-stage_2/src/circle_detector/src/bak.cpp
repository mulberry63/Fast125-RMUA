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
#include <eigen3/Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include<geometry_msgs/PoseStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

ros::Publisher circle_pt_vis;
ros::Publisher estimation_pt_vis;
ros::Publisher circle_orientation_vis;
ros::Publisher camera_point_vis;
ros::Subscriber depth_sub;
ros::Subscriber odom_sub;
ros::Publisher  circle_pos_pub;

std::vector<Eigen::Vector3d> gate_list;
Eigen::Vector3d transition_gate;
Eigen::Vector3d rotation_gate;
Eigen::Vector3d align_gate;

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

void vis_orientation(const Eigen::Vector3d &vertex, const Eigen::Vector3d &direction,int id = 0)
{

    visualization_msgs::Marker ProjectedPointsMaker;
    ProjectedPointsMaker.id = id;
    ProjectedPointsMaker.type = visualization_msgs::Marker::SPHERE_LIST;
    ProjectedPointsMaker.header.stamp = ros::Time::now();
    ProjectedPointsMaker.header.frame_id = "drone_1";
    ProjectedPointsMaker.action = visualization_msgs::Marker::ADD;
    ProjectedPointsMaker.ns = "projectedpoints";
    ProjectedPointsMaker.pose.orientation.w = 1.0;
    ProjectedPointsMaker.color.r = 1.00;
    ProjectedPointsMaker.color.g = 0.00;
    ProjectedPointsMaker.color.b = 0.00;
    ProjectedPointsMaker.color.a = 1.00;
    ProjectedPointsMaker.scale.x = 0.20 ;
    ProjectedPointsMaker.scale.y = 0.20 ;
    ProjectedPointsMaker.scale.z = 0.20 ;

    for(size_t i = 0 ; i < 100;i++)
    {
        geometry_msgs::Point point;
        point.x = (vertex + direction * 0.02 * i)(0);
        point.y = (vertex + direction * 0.02 * i)(1);
        point.z = (vertex + direction * 0.02 * i)(2);
        ProjectedPointsMaker.points.push_back(point);
    }
    circle_orientation_vis.publish(ProjectedPointsMaker);
}

void vis_camera_point(const Eigen::Vector3d &camera_point,double radius = 0.1)
{
        visualization_msgs::Marker sphereMarkers, sphereDeleter;

        sphereMarkers.id = 0;
        sphereMarkers.type = visualization_msgs::Marker::SPHERE_LIST;
        sphereMarkers.header.stamp = ros::Time::now();
        sphereMarkers.header.frame_id = "odom";
        sphereMarkers.pose.orientation.w = 1.00;
        sphereMarkers.action = visualization_msgs::Marker::ADD;
        sphereMarkers.ns = "spheres";
        sphereMarkers.color.r = 1.00;
        sphereMarkers.color.g = 0.00;
        sphereMarkers.color.b = 0.00;
        sphereMarkers.color.a = 1.00;
        sphereMarkers.scale.x = radius * 2.0;
        sphereMarkers.scale.y = radius * 2.0;
        sphereMarkers.scale.z = radius * 2.0;

        sphereDeleter = sphereMarkers;
        sphereDeleter.action = visualization_msgs::Marker::DELETE;

        geometry_msgs::Point point;
        point.x = camera_point(0);
        point.y = camera_point(1);
        point.z = camera_point(2);
        sphereMarkers.points.push_back(point);

        camera_point_vis.publish(sphereDeleter);
        camera_point_vis.publish(sphereMarkers);
}

Eigen::Matrix3d cam_k_, cam_k_inv_;
Eigen::Matrix3d cam2drone_;

 cv_bridge::CvImagePtr cv_ptr;
cv::Mat depth_image;
bool have_depth_msg = false;
void Depth_Img_Cbk(const sensor_msgs::ImageConstPtr &depth_img_msg)
{
        try 
        {
            cv_ptr = cv_bridge::toCvCopy(depth_img_msg, depth_img_msg->encoding.c_str());
        } 
        catch (cv_bridge::Exception& e) 
        {
            ROS_ERROR("cv_bridge execption: %s", e.what());
            return;
        }
        cv_ptr->image.copyTo(depth_image);
        have_depth_msg = true;
        // ROS_WARN("[DEPTH CALL BACK]");
}


bool  getCircleclusterPos(const pcl::PointCloud<pcl::PointXYZ> &input_cloud,
                                                   std::vector<Eigen::Vector3d> &result_cluster,
                                                   Eigen::Vector3d &cluster_origin,
                                                   const double RATIO1_L, const double RATIO1_U,
                                                   const double RATIO2     ,const double min_accept_radius)
{
    if(input_cloud.empty()) return false;

    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = input_cloud.makeShared();

    //voxel filter
    pcl::PointCloud<pcl::PointXYZ> cloud_after_filter;
    pcl::VoxelGrid<pcl::PointXYZ> voxel_sampler;
    voxel_sampler.setLeafSize(0.05f, 0.05f, 0.05f);
    voxel_sampler.setInputCloud(cloud);
    voxel_sampler.filter(cloud_after_filter);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_ptr = cloud_after_filter.makeShared();

    // clustering
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    kdtree->setInputCloud(cloud_filtered_ptr);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> clustering;
    clustering.setClusterTolerance(0.1);
    clustering.setMinClusterSize(100);
    clustering.setMaxClusterSize(5000);
    clustering.setSearchMethod(kdtree);
    clustering.setInputCloud(cloud_filtered_ptr);
    std::vector<pcl::PointIndices> clusters;
    clustering.extract(clusters);


    std::vector<std::vector<Eigen::Vector3d>> potential_result_vec;
    bool flag = false;
    int num = 0;
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
        
        // std::cout << "[evalue] : " << evalue.transpose() << std::endl;
        double ratio1 = evalue(1) / (evalue(2) +0.001);
        double ratio2 = evalue(1) / (evalue(0) +0.0001);
        // std::cout << "[ratio1 & ratio2] : " << ratio1 << " , " << ratio2 << std::endl;
        if(ratio1 > RATIO1_L && ratio1 < RATIO1_U && ratio2 > RATIO2 )
        {
            std::cout << "[eigen vector]  " << std::endl << evector << std::endl;
            std::cout << "[eigen value]  " << std::endl << evalue << std::endl;
            potential_result_vec.push_back(cluster);
            flag = true;
        }
    }
    // ROS_WARN("[potential_result_vec size]   :   %d",potential_result_vec.size());
    if(!flag)  return false;

    //filter clusters by distance 2 it's origin
    std::vector<int> candicates;
    std::vector<Eigen::Vector3d> cluster_origin_vec;
    pcl::PointCloud<pcl::PointXYZ> pt_for_vis_debug;
    for(size_t i = 0 ; i < potential_result_vec.size() ;i++)
    {
        Eigen::Vector3d cluster_origin(0,0,0);
        for(size_t j = 0 ;j < potential_result_vec[i].size();j++)
        {
            pcl::PointXYZ ptt;
            ptt.x = potential_result_vec[i][j](0);
            ptt.y = potential_result_vec[i][j](1);
            ptt.z = potential_result_vec[i][j](2);
            pt_for_vis_debug.points.push_back(ptt);
            cluster_origin = cluster_origin + potential_result_vec[i][j];
        }
        cluster_origin = cluster_origin / potential_result_vec[i].size();
        cluster_origin_vec.push_back(cluster_origin);

        bool potential_cluster_flag = true;
        for(size_t n = 0 ;n < potential_result_vec[i].size();n++)
        {
            double dis2origin = (potential_result_vec[i][n] - cluster_origin).norm(); 
            if(dis2origin < min_accept_radius)  
            {
                potential_cluster_flag = false;
                break;
            }
        }
        if(potential_cluster_flag)  candicates.push_back(i);
    }

    if(candicates.empty()) return false;

    //choose biggest
    double max_candicate_size = 0;
    for(size_t k = 0;k < candicates.size();k++)
    {
        if(max_candicate_size < potential_result_vec[candicates[k]].size())
        {
            max_candicate_size = potential_result_vec[candicates[k]].size();
            result_cluster = potential_result_vec[candicates[k]];
            cluster_origin = cluster_origin_vec[k];
        }
    }
    return true;
}



bool dealWithTransition(const std::vector<Eigen::Vector3d> &transtion_circle_pos_vec,const std::vector<ros::Time> &time_stamp_vec,int &vel_direction)
{
    if(transtion_circle_pos_vec.size() < 4 || time_stamp_vec.size() < 4) return false;

    Eigen::Vector3d first_pos = transtion_circle_pos_vec[transtion_circle_pos_vec.size()-2];
    Eigen::Vector3d second_pos = transtion_circle_pos_vec[transtion_circle_pos_vec.size()-1];

    if(second_pos(1) - first_pos(1) > 0)
    {
        vel_direction = 1;
        return true;
    }
    else if(second_pos(1) - first_pos(1) < 0)
    {
        vel_direction = 2;
        return true;
    }
    return false;
}


//these param should be calibrated
// Eigen::Vector3d rotation_origin(66.00,63.50,2.5);
// Eigen::Vector3d rotation_origin(65.697,  63.40,  2.6);
Eigen::Vector3d rotation_origin(65.9,  63.34,  2.78);
Eigen::Vector3d e1(-0.5,-0.866025,0);
Eigen::Vector3d e2(0,0,1);
Eigen::Vector3d R_normal(-0.866025,0.5,0);
const double rotation_radius = 2.0;   
double theta0;
const double w = 2* M_PI / 8.0;

static double t_fisrt_rotation;
static bool first_rotation = true;
const double time_offset = 0.0;

void vis_rotation_circle(const ros::TimerEvent &event)
{
    if(first_rotation) return;
    double t = ros::Time::now().toSec();
    if(first_rotation) return;
    Eigen::Vector3d circle_ori = rotation_origin + rotation_radius * (e1 * std::cos(w * (t - t_fisrt_rotation - time_offset) + theta0) + e2 * std::sin(w * (t -t_fisrt_rotation - time_offset) + theta0));
    
     pcl::PointCloud<pcl::PointXYZ> pt_for_vis_debug;
    for(double theta = 0.0 ; theta < 2*M_PI ; theta = theta +0.02)
    {
        Eigen::Vector3d point_on_circle = circle_ori +  e1 * std::cos(theta) + e2 * std::sin(theta);
        pcl::PointXYZ pt;
        pt.x = point_on_circle(0);
        pt.y = point_on_circle(1);
        pt.z = point_on_circle(2);
        pt_for_vis_debug.push_back(pt);
    }
    pt_for_vis_debug.push_back(pcl::PointXYZ(circle_ori(0),circle_ori(1),circle_ori(2)));
    pt_for_vis_debug.push_back(pcl::PointXYZ(rotation_origin(0),rotation_origin(1),rotation_origin(2)));
    sensor_msgs::PointCloud2 circle_pt_vis_msg_;
    pcl::toROSMsg(pt_for_vis_debug,circle_pt_vis_msg_);
    circle_pt_vis_msg_.header.frame_id = "world";
    estimation_pt_vis.publish(circle_pt_vis_msg_);
}


bool dealWithRotation(const std::vector<Eigen::Vector3d> &rotation_circle_pos_vec,const std::vector<ros::Time> &time_stamp_vec)
{   
    if(rotation_circle_pos_vec.size() != time_stamp_vec.size())
    {
        ROS_ERROR("!!! This situlation should not happen!!!");
        return false;
    }

    if(first_rotation)
    {
        t_fisrt_rotation = time_stamp_vec.begin()->toSec();
        first_rotation = false;
    }
    geometry_msgs::PoseStamped circle_pos_msg;

    // std::vector<double> theta0_result_vec;
    // for(size_t i = 0 ; i < rotation_circle_pos_vec.size() ;i++)
    // {
    //     Eigen::Vector3d circle_pos = rotation_circle_pos_vec[i];
    //     Eigen::Vector3d V = circle_pos - rotation_origin ;
    //     Eigen::Vector2d proj2plane;
    //     e1.normalize();
    //     proj2plane(0) = e1.dot(V);  //project to target plane
    //     proj2plane(1) = V(2);
    //     proj2plane.normalize();  //shrink to unit circle

    //     double theta = std::atan2(proj2plane(1), proj2plane(0));

    //     double result_theta = theta  -  w * (time_stamp_vec[i].toSec() - t_fisrt_rotation) ;

    //     int cycle = (int)(result_theta /2 /M_PI);
    //     result_theta = result_theta - cycle * 2 * M_PI;
    //     if(result_theta > M_PI) result_theta = result_theta -2*M_PI;
    //     if(result_theta < 0) result_theta = result_theta + M_PI * 2;

    //     theta0_result_vec.push_back(result_theta);
    // }

    // int num = 0;
    // double theta_total = 0.0;
    // for(size_t k = 0; k < theta0_result_vec.size() ; k++)
    // {
    //     theta_total += (k+1) * theta0_result_vec[k];
    //     num = num + k +1 ;
    // }

    // theta0 =  theta_total / num;

    // circle_pos_msg.pose.position.x = theta0;
    // circle_pos_pub.publish(circle_pos_msg);

    Eigen::Vector3d circle_pos = rotation_circle_pos_vec.back();
    Eigen::Vector3d V = circle_pos - rotation_origin ;
    Eigen::Vector2d proj2plane;
    e1.normalize();
    proj2plane(0) = e1.dot(V);
    proj2plane(1) = V(2);
    // circle_pos_msg.pose.orientation.x = proj2plane(0);
    // circle_pos_msg.pose.orientation.y = proj2plane(1);
    proj2plane.normalize();
    std::cout << proj2plane.norm() <<std::endl;
    double theta0_;
    double theta1 = std::asin(proj2plane(1))* 180 / M_PI;
    theta0_ = std::atan2(proj2plane(1), proj2plane(0));
    theta0_ = theta0_  -  w * (time_stamp_vec.back().toSec() - t_fisrt_rotation) ;
    int cycle = (int)(theta0_ /2 /M_PI);
    theta0 = theta0_ - cycle * 2 * M_PI;
    if(theta0 > M_PI) theta0 = theta0 -2*M_PI;
    if(theta0 < 0) theta0 = theta0 + M_PI * 2;
    circle_pos_msg.pose.position.x = theta0;
    circle_pos_msg.pose.position.y = w * time_stamp_vec.back().toSec()  - ((int)(w * time_stamp_vec.back().toSec()/2/M_PI)) * 2 * M_PI;
    circle_pos_msg.pose.position.z = theta0;
    circle_pos_pub.publish(circle_pos_msg);
    // double t = time_stamp_vec.back().toSec();
    // circle_pos = rotation_origin + rotation_radius * (e1 * std::cos(w * (t + 0.0001) + theta0) + e2 * std::sin(w * (t + 0.0001) + theta0));
    // pcl::PointCloud<pcl::PointXYZ> pt_for_vis;
    // for(double theta = 0.0 ; theta < 2*M_PI ; theta = theta +0.02)
    // {
    //     Eigen::Vector3d point_on_circle = circle_pos +  e1 * std::cos(theta) + e2 * std::sin(theta);
    //     pcl::PointXYZ pt;
    //     pt.x = point_on_circle(0);
    //     pt.y = point_on_circle(1);
    //     pt.z = point_on_circle(2);
    //     pt_for_vis.push_back(pt);
    // }
    // sensor_msgs::PointCloud2 circle_pt_vis_msg_;
    // pcl::toROSMsg(pt_for_vis,circle_pt_vis_msg_);
    // circle_pt_vis_msg_.header.frame_id = "world";
    // circle_pt_vis.publish(circle_pt_vis_msg_);

    return true;
}

int image_length_, image_height_;

std::vector<Eigen::Vector3d> rotation_circle_pos_vec;
std::vector<ros::Time> R_time_stamp_vec;
std::vector<Eigen::Vector3d> transtion_circle_pos_vec;
std::vector<ros::Time> T_time_stamp_vec;
const int max_odom_vec_size = 30;
std::vector<nav_msgs::Odometry> odom_vec;
// pcl::PointCloud<pcl::PointXYZ> pt_for_vis_detect;
// std::vector<Eigen::Vector3d> cal_origin;
void Odom_Cbk(const nav_msgs::OdometryConstPtr &odom_msg)
{
    nav_msgs::Odometry temp_odom = *odom_msg;
    odom_vec.push_back(temp_odom);
    if(odom_vec.size() < 10) return;
    if(odom_vec.size() > 30) odom_vec.erase(odom_vec.begin());
    if(!have_depth_msg) return;
    have_depth_msg = false;

    Eigen::Vector3d drone_odom_p_;
    Eigen::Quaterniond drone_odom_q_;

    //odom syn
    nav_msgs::Odometry syn_odom = odom_vec[odom_vec.size() - 6];
    drone_odom_p_(0) = syn_odom.pose.pose.position.x;
    drone_odom_p_(1) = syn_odom.pose.pose.position.y;
    drone_odom_p_(2) = syn_odom.pose.pose.position.z;
    drone_odom_q_.w() = syn_odom.pose.pose.orientation.w;
    drone_odom_q_.x() = syn_odom.pose.pose.orientation.x;
    drone_odom_q_.y() = syn_odom.pose.pose.orientation.y;
    drone_odom_q_.z() = syn_odom.pose.pose.orientation.z;

    cv_ptr->image.copyTo(depth_image);

    Eigen::Matrix3d cam2world = drone_odom_q_ * cam2drone_;
    Eigen::Vector3d cam2world_trans = drone_odom_q_ * Eigen::Vector3d(0.25,0,0);
    Eigen::Vector3d trans = cam2world_trans + drone_odom_p_;
    pcl::PointCloud<pcl::PointXYZ> general_gate_points;
    pcl::PointCloud<pcl::PointXYZ> dynamic_object_points;
    pcl::PointCloud<pcl::PointXYZ> transtion_gate_points;
    pcl::PointCloud<pcl::PointXYZ> rotation_gate_points;
    pcl::PointCloud<pcl::PointXYZ> align_gate_points;
    for(int u = 0; u < image_length_; u = u+1) //x
    {
        for(int v = 0; v < image_height_; v=v+1)//y
        {
            double depth_pixel = depth_image.at<float>(v,u);
             if(depth_pixel  == 0.0) continue;

            Eigen::Vector3d pixel_point = Eigen::Vector3d(u, v, 1);
            Eigen::Vector3d camera_vec = cam_k_inv_ * pixel_point;
            Eigen::Vector3d camera_point = camera_vec * depth_pixel;
            camera_point = cam2world * camera_point + trans;

            //dynamic object1
            if(camera_point(0) >32.0 && camera_point(0) <34 &&   //dynamic object 1   
               camera_point(1) >55 && camera_point(1) < 78.5&&
               camera_point(2) > 0.0&&camera_point(2)< 2.5)
               {    
                    pcl::PointXYZ single_point;
                    single_point.x = camera_point(0); 
                    single_point.y = camera_point(1);
                    single_point.z = camera_point(2);
                    dynamic_object_points.push_back(single_point);
               }

            // transition circle
            if(camera_point(0) >52.0 && camera_point(0) <56.0 && 
               camera_point(1) >60.0 && camera_point(1) < 72.0&&
               camera_point(2) > 0.0&&camera_point(2)< 5.0)
               {
                    pcl::PointXYZ single_point;
                    single_point.x = camera_point(0); 
                    single_point.y = camera_point(1);
                    single_point.z = camera_point(2);
                    transtion_gate_points.push_back(single_point);   
               }

            //rotation circle
            if((camera_point - Eigen::Vector3d(66.0,63.5,3.5)).norm() < 6.0)
               {
                    pcl::PointXYZ single_point;
                    single_point.x = camera_point(0); 
                    single_point.y = camera_point(1);
                    single_point.z = camera_point(2);
                    rotation_gate_points.push_back(single_point); 
                    continue;  
               }

            //align circle
            if((camera_point - align_gate).norm() < 3.0)
               {
                    pcl::PointXYZ single_point;
                    single_point.x = camera_point(0); 
                    single_point.y = camera_point(1);
                    single_point.z = camera_point(2);
                    align_gate_points.push_back(single_point); 
                    continue;  
               }
            
            //general circle
            for(size_t k = 0; k < gate_list.size();k++)
            {
                double dis2ciecle = (camera_point -gate_list[k]).norm();
                if(dis2ciecle < 3) 
                {
                    pcl::PointXYZ single_point;
                    single_point.x = camera_point(0); 
                    single_point.y = camera_point(1);
                    single_point.z = camera_point(2);
                    general_gate_points.push_back(single_point);
                    break;
                }
            }
        }
    } 

    // sensor_msgs::PointCloud2 circle_pt_vis_msg_;
    // pcl::toROSMsg(rotation_gate_points,circle_pt_vis_msg_);
    // circle_pt_vis_msg_.header.frame_id = "world";
    // circle_pt_vis.publish(circle_pt_vis_msg_);

    //general circles
    const double G_RATIO1_L = 0.4;
    const double G_RATIO1_U = 1.6;
    const double G_RATIO2 = 30;
    const double G_min_accept_radius_general = 0.65;
    Eigen::Vector3d G_result_odom;
    std::vector<Eigen::Vector3d> G_result_cluster;
    bool G_have_result = false;
    if(!general_gate_points.empty())
    G_have_result = getCircleclusterPos(general_gate_points,G_result_cluster,G_result_odom,G_RATIO1_L,G_RATIO1_U,G_RATIO2,G_min_accept_radius_general);

    if(G_have_result)
    {
        pcl::PointCloud<pcl::PointXYZ> pt_for_vis;
        Eigen::Vector3d circle_pos(0,0,0);
        for(size_t i = 0 ; i < G_result_cluster.size() ;i++)
        {
            pcl::PointXYZ ptt;
            ptt.x = G_result_cluster[i](0);
            ptt.y = G_result_cluster[i](1);
            ptt.z = G_result_cluster[i](2);
            pt_for_vis.points.push_back(ptt);
        }

        geometry_msgs::PoseStamped circle_pos_msg;
        circle_pos_msg.pose.position.x = G_result_odom(0);
        circle_pos_msg.pose.position.y = G_result_odom(1);
        circle_pos_msg.pose.position.z = G_result_odom(2);
        circle_pos_pub.publish(circle_pos_msg);
        ROS_ERROR("GENERAL  GENERAL GENERAL");
    }

    //transition circle
    const double T_RATIO1_L = 0.4;
    const double T_RATIO1_U = 1.6;
    const double T_RATIO2 = 30;
    const double T_min_accept_radius_general = 0.65;
    Eigen::Vector3d T_result_odom;
    std::vector<Eigen::Vector3d> T_result_cluster;
    bool T_have_result = false;
    if(!transtion_gate_points.empty())
    T_have_result = getCircleclusterPos(transtion_gate_points,T_result_cluster,T_result_odom,G_RATIO1_L,G_RATIO1_U,G_RATIO2,G_min_accept_radius_general);

    if(T_have_result)
    {
        pcl::PointCloud<pcl::PointXYZ> pt_for_vis;
        Eigen::Vector3d circle_pos(0,0,0);
        for(size_t i = 0 ; i < T_result_cluster.size() ;i++)
        {
            pcl::PointXYZ ptt;
            ptt.x = T_result_cluster[i](0);
            ptt.y = T_result_cluster[i](1);
            ptt.z = T_result_cluster[i](2);
            pt_for_vis.points.push_back(ptt);
        }

        geometry_msgs::PoseStamped circle_pos_msg;
        circle_pos_msg.pose.position.x = T_result_odom(0);
        circle_pos_msg.pose.position.y = T_result_odom(1);
        circle_pos_msg.pose.position.z = T_result_odom(2);
        circle_pos_pub.publish(circle_pos_msg);
        ROS_ERROR("TRANSITION  TRANSITION TRANSITION");

        transtion_circle_pos_vec.push_back(T_result_odom);
        T_time_stamp_vec.push_back(syn_odom.header.stamp);
        int vel_direction = 0;
        dealWithTransition(transtion_circle_pos_vec,T_time_stamp_vec,vel_direction);
    }

    //ratation circle     this param should be careful tuned
    const double R_RATIO1_L = 0.75;
    const double R_RATIO1_U = 1.25;
    const double R_RATIO2 = 50;
    const double R_min_accept_radius_general = 0.5;
    Eigen::Vector3d R_result_odom;
    std::vector<Eigen::Vector3d> R_result_cluster;
    bool R_have_result = false;
    if(!rotation_gate_points.empty())
    R_have_result = getCircleclusterPos(rotation_gate_points,R_result_cluster,R_result_odom,R_RATIO1_L,R_RATIO1_U,R_RATIO2,R_min_accept_radius_general);

    if(R_have_result)
    {
        pcl::PointCloud<pcl::PointXYZ> pt_for_vis_detect;
        Eigen::Vector3d circle_pos(0,0,0);
        for(size_t i = 0 ; i < R_result_cluster.size() ;i++)
        {
            pcl::PointXYZ ptt;
            ptt.x = R_result_cluster[i](0);
            ptt.y = R_result_cluster[i](1);
            ptt.z = R_result_cluster[i](2);
            pt_for_vis_detect.points.push_back(ptt);
            // cal_origin.push_back(R_result_cluster[i]);
        }

        // if((ros::Time::now().toSec() - t_fisrt_rotation) > 180)
        // {
        //     Eigen::Vector3d origin(0,0,0);
        //     for(size_t m = 0; m < cal_origin.size();m++)
        //     {
        //         origin += cal_origin[m];
        //     }
        //     origin = origin / cal_origin.size();
        //     std::cout << "[origin]  :   " << origin << std::endl;
        // }
        pt_for_vis_detect.push_back(pcl::PointXYZ(R_result_odom(0),R_result_odom(1),R_result_odom(2)));
        sensor_msgs::PointCloud2 circle_pt_vis_msg_;
        pcl::toROSMsg(pt_for_vis_detect,circle_pt_vis_msg_);
        circle_pt_vis_msg_.header.frame_id = "world";
        circle_pt_vis.publish(circle_pt_vis_msg_);

        // geometry_msgs::PoseStamped circle_pos_msg;
        // circle_pos_msg.pose.position.x = R_result_odom(0);
        // circle_pos_msg.pose.position.y = R_result_odom(1);  
        // circle_pos_msg.pose.position.z = R_result_odom(2);
        // circle_pos_pub.publish(circle_pos_msg);
        // ROS_ERROR("ROTATION  ROTATION  ROTATION");
        rotation_circle_pos_vec.push_back(R_result_odom);
        R_time_stamp_vec.push_back(syn_odom.header.stamp);

        if(rotation_circle_pos_vec.size() > 10) rotation_circle_pos_vec.erase(rotation_circle_pos_vec.begin ());
        if(R_time_stamp_vec.size() > 10) R_time_stamp_vec.erase(R_time_stamp_vec.begin ());
        std::vector<Eigen::Vector3d> test;
        test.push_back(R_result_odom);
        std::vector<ros::Time> t_vec;
        t_vec.push_back(syn_odom.header.stamp);
        dealWithRotation(test,t_vec);
        static int m = 0;

        // dealWithRotation(rotation_circle_pos_vec,R_time_stamp_vec);
        // ROS_WARN("HAVE ROTATION RESULT IN  %d",m);
        // m++;
    }

    //align circle
    const double A_RATIO1_L = 0.4;
    const double A_RATIO1_U = 1.6;
    const double A_RATIO2 = 30;
    const double A_min_accept_radius_general = 0.01;
    Eigen::Vector3d A_result_odom;
    std::vector<Eigen::Vector3d> A_result_cluster;
    bool A_have_result = false;
    if(!transtion_gate_points.empty())
    R_have_result = getCircleclusterPos(transtion_gate_points,A_result_cluster,A_result_odom,A_RATIO1_L,A_RATIO1_U,A_RATIO2,A_min_accept_radius_general);

    if(A_have_result)
    {
        pcl::PointCloud<pcl::PointXYZ> pt_for_vis;
        Eigen::Vector3d circle_pos(0,0,0);
        for(size_t i = 0 ; i < A_result_cluster.size() ;i++)
        {
            pcl::PointXYZ ptt;
            ptt.x = A_result_cluster[i](0);
            ptt.y = A_result_cluster[i](1);
            ptt.z = A_result_cluster[i](2);
            pt_for_vis.points.push_back(ptt);
        }

        geometry_msgs::PoseStamped circle_pos_msg;
        circle_pos_msg.pose.position.x = A_result_odom(0);
        circle_pos_msg.pose.position.y = A_result_odom(1);
        circle_pos_msg.pose.position.z = A_result_odom(2);
        circle_pos_pub.publish(circle_pos_msg);
        ROS_ERROR("PUB  PUB PUB");
    }

    //vis result
    // sensor_msgs::PointCloud2 circle_pt_vis_msg_;
    // pcl::toROSMsg(pt_for_vis,circle_pt_vis_msg_);
    // circle_pt_vis_msg_.header.frame_id = "world";
    // circle_pt_vis.publish(circle_pt_vis_msg_);
    return;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"circle_odom");
    ros::NodeHandle nh;
    circle_pt_vis = nh.advertise<sensor_msgs::PointCloud2>("/circle_pt_vis_topic_d",100);
    estimation_pt_vis = nh.advertise<sensor_msgs::PointCloud2>("/estimation_vis_topic_d",100);
    circle_orientation_vis = nh.advertise<visualization_msgs::Marker>("/visualizer/mesh", 1000);
    camera_point_vis = nh.advertise<visualization_msgs::Marker>("/camera_point",100);

    ros::Timer esti_time = nh.createTimer(ros::Duration(0.001),&vis_rotation_circle);

    depth_sub = nh.subscribe( "/airsim_node/drone_1/front_center/DepthPlanar",1,&Depth_Img_Cbk,ros::TransportHints().tcpNoDelay());
    odom_sub = nh.subscribe("/odom",1,Odom_Cbk,ros::TransportHints().tcpNoDelay());
    circle_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/circle_odom", 1);

    cam_k_ << 159.233784156,                                           0,                                                      160,
            0,                                                         159.233784156,                           120,
            0,                                                         0,                                                       1;

    cam_k_inv_ = cam_k_.inverse();
    cam2drone_ << 0,    0,    1, 
                -1,     0,    0,
                0,    -1,    0;

    image_height_ = 240;

    image_length_ = 320;

    gate_list.push_back(Eigen::Vector3d( 1.75,     6.92,       2.66));
	gate_list.push_back(Eigen::Vector3d( -0.31,     18.91,      3.11));
    gate_list.push_back(Eigen::Vector3d( 1.36,      33.71,      3.15));
    gate_list.push_back(Eigen::Vector3d( 0.63,      56.9,       1.8));
    gate_list.push_back(Eigen::Vector3d( 25.4,      64.2,       2.7));
    gate_list.push_back(Eigen::Vector3d( 35.0,      65.8,       2.2));
    gate_list.push_back(Eigen::Vector3d( 64.0,      60.8,       2.2));
    gate_list.push_back(Eigen::Vector3d( 72.0,      56.8,       2.2));
    gate_list.push_back(Eigen::Vector3d( 72.0,      52.8,       2.2));
    gate_list.push_back(Eigen::Vector3d(72.0,       50.8,       2.2));
    gate_list.push_back(Eigen::Vector3d(43.0,      64.4,        2.7));
    gate_list.push_back(Eigen::Vector3d(64.4,      46.0,        2.7));
    // gate_list.push_back(Eigen::Vector3d(54.31,   64.32,      3.52  ));//transition  
    // gate_list.push_back(Eigen::Vector3d(65.85,   63.29,       2.65));//rotation
    // gate_list.push_back(Eigen::Vector3d(69.55,   42.33,       3.15));//alien

    transition_gate = Eigen::Vector3d(54.31,   64.32,      3.52);
    rotation_gate = Eigen::Vector3d(65.85,  63.29,         2.65);
    align_gate = Eigen::Vector3d(69.55,   42.33,       3.15);

    while(ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}




















































































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
#include <eigen3/Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include<geometry_msgs/PoseStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>



ros::Publisher circle_pt_vis;
ros::Publisher circle_orientation_vis;
ros::Publisher estimation_pt_vis;
ros::Publisher camera_point_vis;
ros::Subscriber depth_sub;
ros::Subscriber odom_sub;
ros::Publisher  circle_pos_pub;

std::vector<Eigen::Vector3d> gate_list;
Eigen::Vector3d transition_gate;
Eigen::Vector3d rotation_gate;
Eigen::Vector3d align_gate;

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

void vis_orientation(const Eigen::Vector3d &vertex, const Eigen::Vector3d &direction,int id = 0)
{

    visualization_msgs::Marker ProjectedPointsMaker;
    ProjectedPointsMaker.id = id;
    ProjectedPointsMaker.type = visualization_msgs::Marker::SPHERE_LIST;
    ProjectedPointsMaker.header.stamp = ros::Time::now();
    ProjectedPointsMaker.header.frame_id = "drone_1";
    ProjectedPointsMaker.action = visualization_msgs::Marker::ADD;
    ProjectedPointsMaker.ns = "projectedpoints";
    ProjectedPointsMaker.pose.orientation.w = 1.0;
    ProjectedPointsMaker.color.r = 1.00;
    ProjectedPointsMaker.color.g = 0.00;
    ProjectedPointsMaker.color.b = 0.00;
    ProjectedPointsMaker.color.a = 1.00;
    ProjectedPointsMaker.scale.x = 0.20 ;
    ProjectedPointsMaker.scale.y = 0.20 ;
    ProjectedPointsMaker.scale.z = 0.20 ;

    for(size_t i = 0 ; i < 100;i++)
    {
        geometry_msgs::Point point;
        point.x = (vertex + direction * 0.02 * i)(0);
        point.y = (vertex + direction * 0.02 * i)(1);
        point.z = (vertex + direction * 0.02 * i)(2);
        ProjectedPointsMaker.points.push_back(point);
    }
    circle_orientation_vis.publish(ProjectedPointsMaker);
}

void vis_camera_point(const Eigen::Vector3d &camera_point,double radius = 0.1)
{
        visualization_msgs::Marker sphereMarkers, sphereDeleter;

        sphereMarkers.id = 0;
        sphereMarkers.type = visualization_msgs::Marker::SPHERE_LIST;
        sphereMarkers.header.stamp = ros::Time::now();
        sphereMarkers.header.frame_id = "odom";
        sphereMarkers.pose.orientation.w = 1.00;
        sphereMarkers.action = visualization_msgs::Marker::ADD;
        sphereMarkers.ns = "spheres";
        sphereMarkers.color.r = 1.00;
        sphereMarkers.color.g = 0.00;
        sphereMarkers.color.b = 0.00;
        sphereMarkers.color.a = 1.00;
        sphereMarkers.scale.x = radius * 2.0;
        sphereMarkers.scale.y = radius * 2.0;
        sphereMarkers.scale.z = radius * 2.0;

        sphereDeleter = sphereMarkers;
        sphereDeleter.action = visualization_msgs::Marker::DELETE;

        geometry_msgs::Point point;
        point.x = camera_point(0);
        point.y = camera_point(1);
        point.z = camera_point(2);
        sphereMarkers.points.push_back(point);

        camera_point_vis.publish(sphereDeleter);
        camera_point_vis.publish(sphereMarkers);
}

Eigen::Matrix3d cam_k_, cam_k_inv_;
Eigen::Matrix3d cam2drone_;

 cv_bridge::CvImagePtr cv_ptr;
cv::Mat depth_image;
bool have_depth_msg = false;
void Depth_Img_Cbk(const sensor_msgs::ImageConstPtr &depth_img_msg)
{
        try 
        {
            cv_ptr = cv_bridge::toCvCopy(depth_img_msg, depth_img_msg->encoding.c_str());
        } 
        catch (cv_bridge::Exception& e) 
        {
            ROS_ERROR("cv_bridge execption: %s", e.what());
            return;
        }
        cv_ptr->image.copyTo(depth_image);
        have_depth_msg = true;
        // ROS_WARN("[DEPTH CALL BACK]");
}

bool  getCircleclusterPos(const pcl::PointCloud<pcl::PointXYZ> &input_cloud,
                                                   std::vector<Eigen::Vector3d> &result_cluster,
                                                   Eigen::Vector3d &cluster_origin,
                                                   const double RATIO1_L, const double RATIO1_U,
                                                   const double RATIO2     ,const double min_accept_radius)
{
    if(input_cloud.empty()) return false;

    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = input_cloud.makeShared();

    //voxel filter
    pcl::PointCloud<pcl::PointXYZ> cloud_after_filter;
    pcl::VoxelGrid<pcl::PointXYZ> voxel_sampler;
    voxel_sampler.setLeafSize(0.05f, 0.05f, 0.05f);
    voxel_sampler.setInputCloud(cloud);
    voxel_sampler.filter(cloud_after_filter);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_ptr = cloud_after_filter.makeShared();

    // clustering
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    kdtree->setInputCloud(cloud_filtered_ptr);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> clustering;
    clustering.setClusterTolerance(0.1);
    clustering.setMinClusterSize(100);
    clustering.setMaxClusterSize(5000);
    clustering.setSearchMethod(kdtree);
    clustering.setInputCloud(cloud_filtered_ptr);
    std::vector<pcl::PointIndices> clusters;
    clustering.extract(clusters);


    // ROS_WARN("[clusters size] : %d",clusters.size());
    std::vector<std::vector<Eigen::Vector3d>> potential_result_vec;
    bool flag = false;
    int num = 0;
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
        
        // std::cout << "[evalue] : " << evalue.transpose() << std::endl;
        double ratio1 = evalue(1) / (evalue(2) +0.001);
        double ratio2 = evalue(1) / (evalue(0) +0.0001);
        // std::cout << "[ratio1 & ratio2] : " << ratio1 << " , " << ratio2 << std::endl;
        if(ratio1 > RATIO1_L && ratio1 < RATIO1_U && ratio2 > RATIO2 )
        {
            std::cout << "[eigen vector]  " << std::endl << evector << std::endl;
            potential_result_vec.push_back(cluster);
            flag = true;
        }
    }
    // ROS_WARN("[potential_result_vec size]   :   %d",potential_result_vec.size());
    if(!flag)  return false;

    //filter clusters by distance 2 it's origin
    std::vector<int> candicates;
    std::vector<Eigen::Vector3d> cluster_origin_vec;
    pcl::PointCloud<pcl::PointXYZ> pt_for_vis_debug;
    for(size_t i = 0 ; i < potential_result_vec.size() ;i++)
    {
        Eigen::Vector3d cluster_origin(0,0,0);
        for(size_t j = 0 ;j < potential_result_vec[i].size();j++)
        {
            pcl::PointXYZ ptt;
            ptt.x = potential_result_vec[i][j](0);
            ptt.y = potential_result_vec[i][j](1);
            ptt.z = potential_result_vec[i][j](2);
            pt_for_vis_debug.points.push_back(ptt);
            cluster_origin = cluster_origin + potential_result_vec[i][j];
        }
        cluster_origin = cluster_origin / potential_result_vec[i].size();
        cluster_origin_vec.push_back(cluster_origin);

        bool potential_cluster_flag = true;
        for(size_t n = 0 ;n < potential_result_vec[i].size();n++)
        {
            double dis2origin = (potential_result_vec[i][n] - cluster_origin).norm(); 
            if(dis2origin < min_accept_radius)  
            {
                potential_cluster_flag = false;
                break;
            }
        }
        if(potential_cluster_flag)  candicates.push_back(i);
    }

    if(candicates.empty()) return false;

    //choose biggest
    double max_candicate_size = 0;
    for(size_t k = 0;k < candicates.size();k++)
    {
        if(max_candicate_size < potential_result_vec[candicates[k]].size())
        {
            max_candicate_size = potential_result_vec[candicates[k]].size();
            result_cluster = potential_result_vec[candicates[k]];
            cluster_origin = cluster_origin_vec[k];
        }
    }
    return true;
}


bool dealWithTransition(const std::vector<Eigen::Vector3d> &transtion_circle_pos_vec,const std::vector<ros::Time> &time_stamp_vec,int &vel_direction)
{
    if(transtion_circle_pos_vec.size() < 4 || time_stamp_vec.size() < 4) return false;

    Eigen::Vector3d first_pos = transtion_circle_pos_vec[transtion_circle_pos_vec.size()-2];
    Eigen::Vector3d second_pos = transtion_circle_pos_vec[transtion_circle_pos_vec.size()-1];

    if(second_pos(1) - first_pos(1) > 0)
    {
        vel_direction = 1;
        return true;
    }
    else if(second_pos(1) - first_pos(1) < 0)
    {
        vel_direction = 2;
        return true;
    }
    return false;
}


//these param should be calibrated
// Eigen::Vector3d rotation_origin(66.00,63.50,2.5);
Eigen::Vector3d rotation_origin(65.9,  63.34,  2.78);
Eigen::Vector3d e1(-0.5,-0.866025,0);
Eigen::Vector3d e2(0,0,1);
Eigen::Vector3d R_normal(-0.866025,0.5,0);
const double rotation_radius = 2.0;   
double theta0;
const double w = 2* M_PI / 8.0;

static double t_fisrt_rotation;
static bool first_rotation = true;
const double time_offset = 0.8;

void vis_rotation_circle(const ros::TimerEvent &event)
{
    if(first_rotation) return;
    double t = ros::Time::now().toSec();
    ROS_ERROR("[vis_rotation_circle]    :   %lf",t);
    if(first_rotation) return;
    Eigen::Vector3d circle_ori = rotation_origin + rotation_radius * (e1 * std::cos(w * (t - t_fisrt_rotation - time_offset) + theta0) + e2 * std::sin(w * (t -t_fisrt_rotation - time_offset) + theta0));
    
     pcl::PointCloud<pcl::PointXYZ> pt_for_vis_debug;
    for(double theta = 0.0 ; theta < 2*M_PI ; theta = theta +0.02)
    {
        Eigen::Vector3d point_on_circle = circle_ori +  e1 * std::cos(theta) + e2 * std::sin(theta);
        pcl::PointXYZ pt;
        pt.x = point_on_circle(0);
        pt.y = point_on_circle(1);
        pt.z = point_on_circle(2);
        pt_for_vis_debug.push_back(pt);
    }
    pt_for_vis_debug.push_back(pcl::PointXYZ(circle_ori(0),circle_ori(1),circle_ori(2)));
    pt_for_vis_debug.push_back(pcl::PointXYZ(rotation_origin(0),rotation_origin(1),rotation_origin(2)));
    sensor_msgs::PointCloud2 circle_pt_vis_msg_;
    pcl::toROSMsg(pt_for_vis_debug,circle_pt_vis_msg_);
    circle_pt_vis_msg_.header.frame_id = "world";
    estimation_pt_vis.publish(circle_pt_vis_msg_);
}


bool dealWithRotation(const std::vector<Eigen::Vector3d> &rotation_circle_pos_vec,const std::vector<ros::Time> &time_stamp_vec)
{
    if(first_rotation)
    {
        t_fisrt_rotation = time_stamp_vec.begin()->toSec();
        first_rotation = false;
    }
    ROS_ERROR("[dealWithRotation]   %lf",time_stamp_vec.back().toSec());
    Eigen::Vector3d circle_pos = rotation_circle_pos_vec.back();
    Eigen::Vector3d V = circle_pos - rotation_origin ;
    Eigen::Vector2d proj2plane;
    e1.normalize();
    proj2plane(0) = e1.dot(V);
    proj2plane(1) = V(2);

    proj2plane.normalize();
    std::cout << proj2plane.norm() <<std::endl;

    theta0 = std::atan2(proj2plane(1), proj2plane(0))  -  w * (time_stamp_vec.back().toSec()-t_fisrt_rotation) ;

    int cycle = theta0 /2 /M_PI;
    theta0 = theta0 - cycle * 2 *M_PI;

    geometry_msgs::PoseStamped circle_pos_msg;
    circle_pos_msg.pose.position.x = theta0;
    circle_pos_pub.publish(circle_pos_msg);
}

int image_length_, image_height_;

std::vector<Eigen::Vector3d> rotation_circle_pos_vec;
std::vector<ros::Time> R_time_stamp_vec;
std::vector<Eigen::Vector3d> transtion_circle_pos_vec;
std::vector<ros::Time> T_time_stamp_vec;
 const int max_odom_vec_size = 30;
 std::vector<nav_msgs::Odometry> odom_vec;
void Odom_Cbk(const nav_msgs::OdometryConstPtr &odom_msg)
{
    nav_msgs::Odometry temp_odom = *odom_msg;
    odom_vec.push_back(temp_odom);
    if(odom_vec.size() < 10) return;
    if(odom_vec.size() > 30) odom_vec.erase(odom_vec.begin());
    if(!have_depth_msg) return;
    have_depth_msg = false;

    Eigen::Vector3d drone_odom_p_;
    Eigen::Quaterniond drone_odom_q_;

    //odom syn
    nav_msgs::Odometry syn_odom = odom_vec[odom_vec.size() - 6];
    drone_odom_p_(0) = syn_odom.pose.pose.position.x;
    drone_odom_p_(1) = syn_odom.pose.pose.position.y;
    drone_odom_p_(2) = syn_odom.pose.pose.position.z;
    drone_odom_q_.w() = syn_odom.pose.pose.orientation.w;
    drone_odom_q_.x() = syn_odom.pose.pose.orientation.x;
    drone_odom_q_.y() = syn_odom.pose.pose.orientation.y;
    drone_odom_q_.z() = syn_odom.pose.pose.orientation.z;

    cv_ptr->image.copyTo(depth_image);

    Eigen::Matrix3d cam2world = drone_odom_q_ * cam2drone_;
    Eigen::Vector3d cam2world_trans = drone_odom_q_ * Eigen::Vector3d(0.25,0,0);
    Eigen::Vector3d trans = cam2world_trans + drone_odom_p_;
    pcl::PointCloud<pcl::PointXYZ> general_gate_points;
    pcl::PointCloud<pcl::PointXYZ> dynamic_object_points;
    pcl::PointCloud<pcl::PointXYZ> transtion_gate_points;
    pcl::PointCloud<pcl::PointXYZ> rotation_gate_points;
    pcl::PointCloud<pcl::PointXYZ> align_gate_points;
    for(int u = 0; u < image_length_; u = u+1) //x
    {
        for(int v = 0; v < image_height_; v=v+1)//y
        {
            double depth_pixel = depth_image.at<float>(v,u);
             if(depth_pixel  == 0.0) continue;

            Eigen::Vector3d pixel_point = Eigen::Vector3d(u, v, 1);
            Eigen::Vector3d camera_vec = cam_k_inv_ * pixel_point;
            Eigen::Vector3d camera_point = camera_vec * depth_pixel;
            camera_point = cam2world * camera_point + trans;

            //dynamic object1
            if(camera_point(0) >32.0 && camera_point(0) <34 &&   //dynamic object 1   
               camera_point(1) >55 && camera_point(1) < 78.5&&
               camera_point(2) > 0.0&&camera_point(2)< 2.5)
               {    
                    pcl::PointXYZ single_point;
                    single_point.x = camera_point(0); 
                    single_point.y = camera_point(1);
                    single_point.z = camera_point(2);
                    dynamic_object_points.push_back(single_point);
               }

            // transition circle
            if(camera_point(0) >52.0 && camera_point(0) <56.0 && 
               camera_point(1) >60.0 && camera_point(1) < 72.0&&
               camera_point(2) > 0.0&&camera_point(2)< 5.0)
               {
                    pcl::PointXYZ single_point;
                    single_point.x = camera_point(0); 
                    single_point.y = camera_point(1);
                    single_point.z = camera_point(2);
                    transtion_gate_points.push_back(single_point);   
               }

            //rotation circle
            if((camera_point - Eigen::Vector3d(66.0,63.5,3.5)).norm() < 6.0)
               {
                    pcl::PointXYZ single_point;
                    single_point.x = camera_point(0); 
                    single_point.y = camera_point(1);
                    single_point.z = camera_point(2);
                    rotation_gate_points.push_back(single_point); 
                    continue;  
               }

            //align circle
            if((camera_point - align_gate).norm() < 3.0)
               {
                    pcl::PointXYZ single_point;
                    single_point.x = camera_point(0); 
                    single_point.y = camera_point(1);
                    single_point.z = camera_point(2);
                    align_gate_points.push_back(single_point); 
                    continue;  
               }
            
            //general circle
            for(size_t k = 0; k < gate_list.size();k++)
            {
                double dis2ciecle = (camera_point -gate_list[k]).norm();
                if(dis2ciecle < 3) 
                {
                    pcl::PointXYZ single_point;
                    single_point.x = camera_point(0); 
                    single_point.y = camera_point(1);
                    single_point.z = camera_point(2);
                    general_gate_points.push_back(single_point);
                    break;
                }
            }
        }
    } 

    // sensor_msgs::PointCloud2 circle_pt_vis_msg_;
    // pcl::toROSMsg(rotation_gate_points,circle_pt_vis_msg_);
    // circle_pt_vis_msg_.header.frame_id = "world";
    // circle_pt_vis.publish(circle_pt_vis_msg_);

    //general circles
    const double G_RATIO1_L = 0.4;
    const double G_RATIO1_U = 1.6;
    const double G_RATIO2 = 30;
    const double G_min_accept_radius_general = 0.65;
    Eigen::Vector3d G_result_odom;
    std::vector<Eigen::Vector3d> G_result_cluster;
    bool G_have_result = false;
    if(!general_gate_points.empty())
    G_have_result = getCircleclusterPos(general_gate_points,G_result_cluster,G_result_odom,G_RATIO1_L,G_RATIO1_U,G_RATIO2,G_min_accept_radius_general);

    if(G_have_result)
    {
        pcl::PointCloud<pcl::PointXYZ> pt_for_vis;
        Eigen::Vector3d circle_pos(0,0,0);
        for(size_t i = 0 ; i < G_result_cluster.size() ;i++)
        {
            pcl::PointXYZ ptt;
            ptt.x = G_result_cluster[i](0);
            ptt.y = G_result_cluster[i](1);
            ptt.z = G_result_cluster[i](2);
            pt_for_vis.points.push_back(ptt);
        }

        geometry_msgs::PoseStamped circle_pos_msg;
        circle_pos_msg.pose.position.x = G_result_odom(0);
        circle_pos_msg.pose.position.y = G_result_odom(1);
        circle_pos_msg.pose.position.z = G_result_odom(2);
        circle_pos_pub.publish(circle_pos_msg);
        ROS_ERROR("GENERAL  GENERAL GENERAL");
    }

    //transition circle
    const double T_RATIO1_L = 0.4;
    const double T_RATIO1_U = 1.6;
    const double T_RATIO2 = 30;
    const double T_min_accept_radius_general = 0.65;
    Eigen::Vector3d T_result_odom;
    std::vector<Eigen::Vector3d> T_result_cluster;
    bool T_have_result = false;
    if(!transtion_gate_points.empty())
    T_have_result = getCircleclusterPos(transtion_gate_points,T_result_cluster,T_result_odom,G_RATIO1_L,G_RATIO1_U,G_RATIO2,G_min_accept_radius_general);

    if(T_have_result)
    {
        pcl::PointCloud<pcl::PointXYZ> pt_for_vis;
        Eigen::Vector3d circle_pos(0,0,0);
        for(size_t i = 0 ; i < T_result_cluster.size() ;i++)
        {
            pcl::PointXYZ ptt;
            ptt.x = T_result_cluster[i](0);
            ptt.y = T_result_cluster[i](1);
            ptt.z = T_result_cluster[i](2);
            pt_for_vis.points.push_back(ptt);
        }

        geometry_msgs::PoseStamped circle_pos_msg;
        circle_pos_msg.pose.position.x = T_result_odom(0);
        circle_pos_msg.pose.position.y = T_result_odom(1);
        circle_pos_msg.pose.position.z = T_result_odom(2);
        circle_pos_pub.publish(circle_pos_msg);
        ROS_ERROR("TRANSITION  TRANSITION TRANSITION");

        transtion_circle_pos_vec.push_back(T_result_odom);
        T_time_stamp_vec.push_back(syn_odom.header.stamp);
        int vel_direction = 0;
        dealWithTransition(transtion_circle_pos_vec,T_time_stamp_vec,vel_direction);
    }

    //ratation circle     this param should be careful tuned
    const double R_RATIO1_L = 0.75;
    const double R_RATIO1_U = 1.25;
    const double R_RATIO2 = 50;
    const double R_min_accept_radius_general = 0.5;
    Eigen::Vector3d R_result_odom;
    std::vector<Eigen::Vector3d> R_result_cluster;
    bool R_have_result = false;
    if(!rotation_gate_points.empty())
    R_have_result = getCircleclusterPos(rotation_gate_points,R_result_cluster,R_result_odom,R_RATIO1_L,R_RATIO1_U,R_RATIO2,R_min_accept_radius_general);

    if(R_have_result)
    {
        pcl::PointCloud<pcl::PointXYZ> pt_for_vis;
        Eigen::Vector3d circle_pos(0,0,0);
        for(size_t i = 0 ; i < R_result_cluster.size() ;i++)
        {
            pcl::PointXYZ ptt;
            ptt.x = R_result_cluster[i](0);
            ptt.y = R_result_cluster[i](1);
            ptt.z = R_result_cluster[i](2);
            pt_for_vis.points.push_back(ptt);
        }

        sensor_msgs::PointCloud2 circle_pt_vis_msg_;
        pcl::toROSMsg(pt_for_vis,circle_pt_vis_msg_);
        circle_pt_vis_msg_.header.frame_id = "world";
        circle_pt_vis.publish(circle_pt_vis_msg_);

        // geometry_msgs::PoseStamped circle_pos_msg;
        // circle_pos_msg.pose.position.x = R_result_odom(0);
        // circle_pos_msg.pose.position.y = R_result_odom(1);  
        // circle_pos_msg.pose.position.z = R_result_odom(2);
        // circle_pos_pub.publish(circle_pos_msg);
        // ROS_ERROR("ROTATION  ROTATION  ROTATION");
        rotation_circle_pos_vec.push_back(R_result_odom);
        R_time_stamp_vec.push_back(syn_odom.header.stamp);


        std::vector<Eigen::Vector3d> test;
        test.push_back(R_result_odom);
        std::vector<ros::Time> t_vec;
        t_vec.push_back(syn_odom.header.stamp);
        dealWithRotation(test,t_vec);
        static int m = 0;
        // dealWithRotation(rotation_circle_pos_vec,R_time_stamp_vec);
        ROS_WARN("HAVE ROTATION RESULT IN  %d",m);
        m++;
    }

    //align circle
    const double A_RATIO1_L = 0.4;
    const double A_RATIO1_U = 1.6;
    const double A_RATIO2 = 30;
    const double A_min_accept_radius_general = 0.01;
    Eigen::Vector3d A_result_odom;
    std::vector<Eigen::Vector3d> A_result_cluster;
    bool A_have_result = false;
    if(!transtion_gate_points.empty())
    R_have_result = getCircleclusterPos(transtion_gate_points,A_result_cluster,A_result_odom,A_RATIO1_L,A_RATIO1_U,A_RATIO2,A_min_accept_radius_general);

    if(A_have_result)
    {
        pcl::PointCloud<pcl::PointXYZ> pt_for_vis;
        Eigen::Vector3d circle_pos(0,0,0);
        for(size_t i = 0 ; i < A_result_cluster.size() ;i++)
        {
            pcl::PointXYZ ptt;
            ptt.x = A_result_cluster[i](0);
            ptt.y = A_result_cluster[i](1);
            ptt.z = A_result_cluster[i](2);
            pt_for_vis.points.push_back(ptt);
        }

        geometry_msgs::PoseStamped circle_pos_msg;
        circle_pos_msg.pose.position.x = A_result_odom(0);
        circle_pos_msg.pose.position.y = A_result_odom(1);
        circle_pos_msg.pose.position.z = A_result_odom(2);
        circle_pos_pub.publish(circle_pos_msg);
        ROS_ERROR("PUB  PUB PUB");
    }

    //vis result
    // sensor_msgs::PointCloud2 circle_pt_vis_msg_;
    // pcl::toROSMsg(pt_for_vis,circle_pt_vis_msg_);
    // circle_pt_vis_msg_.header.frame_id = "world";
    // circle_pt_vis.publish(circle_pt_vis_msg_);
    return;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"circle_odom");
    ros::NodeHandle nh;
    circle_pt_vis = nh.advertise<sensor_msgs::PointCloud2>("/circle_pt_vis_topic_d",100);
    circle_orientation_vis = nh.advertise<visualization_msgs::Marker>("/visualizer/mesh", 1000);
    camera_point_vis = nh.advertise<visualization_msgs::Marker>("/camera_point",100);
    estimation_pt_vis = nh.advertise<sensor_msgs::PointCloud2>("/estimation_vis_topic_d",100);

    ros::Timer esti_time = nh.createTimer(ros::Duration(0.001),&vis_rotation_circle);

    depth_sub = nh.subscribe( "/airsim_node/drone_1/front_center/DepthPlanar",1,&Depth_Img_Cbk,ros::TransportHints().tcpNoDelay());
    odom_sub = nh.subscribe("/odom",1,Odom_Cbk,ros::TransportHints().tcpNoDelay());
    circle_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/circle_odom", 1);

    cam_k_ << 159.233784156,                                           0,                                                      160,
                            0,                                                                      159.233784156,                           120,
                            0,                                                                       0,                                                       1;

    cam_k_inv_ = cam_k_.inverse();
    cam2drone_ << 0,    0,    1, 
                                  -1,     0,    0,
                                   0,    -1,    0;

    image_height_ = 240;

    image_length_ = 320;

    gate_list.push_back(Eigen::Vector3d( 1.75,     6.92,       2.66));
	gate_list.push_back(Eigen::Vector3d( -0.31,     18.91,      3.11));
    gate_list.push_back(Eigen::Vector3d( 1.36,      33.71,      3.15));
    gate_list.push_back(Eigen::Vector3d( 0.63,      56.9,       1.8));
    gate_list.push_back(Eigen::Vector3d( 25.4,      64.2,       2.7));
    gate_list.push_back(Eigen::Vector3d( 35.0,      65.8,       2.2));
    gate_list.push_back(Eigen::Vector3d( 64.0,      60.8,       2.2));
    gate_list.push_back(Eigen::Vector3d( 72.0,      56.8,       2.2));
    gate_list.push_back(Eigen::Vector3d( 72.0,      52.8,       2.2));
    gate_list.push_back(Eigen::Vector3d(72.0,       50.8,       2.2));
    gate_list.push_back(Eigen::Vector3d(43.0,      64.4,        2.7));
    gate_list.push_back(Eigen::Vector3d(64.4,      46.0,        2.7));
    // gate_list.push_back(Eigen::Vector3d(54.31,   64.32,      3.52  ));//transition  
    // gate_list.push_back(Eigen::Vector3d(65.85,   63.29,       2.65));//rotation
    // gate_list.push_back(Eigen::Vector3d(69.55,   42.33,       3.15));//alien

    transition_gate = Eigen::Vector3d(54.31,   64.32,      3.52);
    rotation_gate = Eigen::Vector3d(65.85,  63.29,         2.65);
    align_gate = Eigen::Vector3d(69.55,   42.33,       3.15);

    while(ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}


























































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
#include <eigen3/Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include<geometry_msgs/PoseStamped.h>
#include"circle_detector/detect_circle.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


struct Config
{
    std::string depth_topic;
    std::string odom_topic;
    
    double ekf_rate;
    double ekf_Q_p;
    double ekf_Q_v;
    double ekf_Q_p;
    double ekf_Q_v;

    double fx;
    double fy;
    double cx;
    double cy;
    double cam2body_x;
    double cam2body_y;
    double cam2body_z;
    int skip_pixel;

    double g_cluster_leaf_size;
    int g_cluster_tolerance;
    int g_cluster_min_size;
    int g_cluster_max_siza;

    double transition_circle_box_min_x;
    double transition_circle_box_min_y;
    double transition_circle_box_min_z;
    double transition_circle_box_max_x;
    double transition_circle_box_max_y;
    double transition_circle_box_max_z;


    std::vector<double> circle_list;
    int frame_delay;

    std::vector<double> rotation_origin;
    double rotation_radius;
    double rotation_circle_min_radius;

    double g_circle_min_radius;
    double g_ratio1_l;
    double g_ratio1_u;
    double g_ratio2;
    double g_min_accept_radius;

};


ros::Publisher circle_pt_vis;
ros::Publisher circle_orientation_vis;
ros::Publisher estimation_pt_vis;
ros::Publisher camera_point_vis;
ros::Subscriber depth_sub;
ros::Subscriber odom_sub;
ros::Publisher  circle_pos_pub;

std::vector<Eigen::Vector3d> gate_list;
Eigen::Vector3d transition_gate;
Eigen::Vector3d rotation_gate;
Eigen::Vector3d align_gate;

ros::Publisher circle_detect_pub;


struct Ekf 
{
  double dt;
  Eigen::MatrixXd A, B, C;
  Eigen::MatrixXd Qt, Rt;
  Eigen::MatrixXd Sigma, K;
  Eigen::VectorXd x;

  Ekf(double _dt) : dt(_dt) 
  {
    A.setIdentity(2, 2);
    Sigma.setZero(2, 2);
    B.setZero(2, 1);
    C.setZero(2, 2);

    double t2 = dt * dt / 2;
    B(0, 0) = t2;
    B(1,0) = dt;

    C(0,0) = 1;
    C(1,1) = 1;
    K = C;

    Qt.setIdentity(2, 2);
    Qt(0, 0) = 1;
    Qt(1, 1) = 1;

    Rt.setIdentity(2, 2);
    Rt(0, 0) = 0.5;
    Rt(1, 1) = 0.5;

    x.setZero(2);
  }
  inline void predict() 
  {
    x = A * x;
    Sigma = A * Sigma * A.transpose() + B * Qt * B.transpose();
    return;
  }

  inline void reset(const Eigen::Vector3d &z) 
  {
    x(0) = z(1);
    x(1) = 0.0;
    Sigma.setZero();
  }

  inline bool checkValid(const Eigen::Vector3d& z) const 
  {
    Eigen::MatrixXd K_tmp = Sigma * C.transpose() * (C * Sigma * C.transpose() + Rt).inverse();
    Eigen::VectorXd x_tmp = x + K_tmp * (z - C * x);
    const double vmax = 4;
    if (x(1) > vmax) 
    {
      return false;
    } else 
    {
      return true;
    }
  }

  inline void update(const Eigen::Vector3d& z) 
  {
    Eigen::Vector2d Z(z(0),z(1));
    K = Sigma * C.transpose() * (C * Sigma * C.transpose() + Rt).inverse();
    x = x + K * (Z - C * x);
    Sigma = Sigma - K * C * Sigma;
  }

  inline const double pos() const 
  {
    return x(0);
  }

  inline const double vel() const 
  {
    return x(1);
  }
};

std::shared_ptr<Ekf> ekfPtr_;

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

void vis_orientation(const Eigen::Vector3d &vertex, const Eigen::Vector3d &direction,int id = 0)
{

    visualization_msgs::Marker ProjectedPointsMaker;
    ProjectedPointsMaker.id = id;
    ProjectedPointsMaker.type = visualization_msgs::Marker::SPHERE_LIST;
    ProjectedPointsMaker.header.stamp = ros::Time::now();
    ProjectedPointsMaker.header.frame_id = "drone_1";
    ProjectedPointsMaker.action = visualization_msgs::Marker::ADD;
    ProjectedPointsMaker.ns = "projectedpoints";
    ProjectedPointsMaker.pose.orientation.w = 1.0;
    ProjectedPointsMaker.color.r = 1.00;
    ProjectedPointsMaker.color.g = 0.00;
    ProjectedPointsMaker.color.b = 0.00;
    ProjectedPointsMaker.color.a = 1.00;
    ProjectedPointsMaker.scale.x = 0.20 ;
    ProjectedPointsMaker.scale.y = 0.20 ;
    ProjectedPointsMaker.scale.z = 0.20 ;

    for(size_t i = 0 ; i < 100;i++)
    {
        geometry_msgs::Point point;
        point.x = (vertex + direction * 0.02 * i)(0);
        point.y = (vertex + direction * 0.02 * i)(1);
        point.z = (vertex + direction * 0.02 * i)(2);
        ProjectedPointsMaker.points.push_back(point);
    }
    circle_orientation_vis.publish(ProjectedPointsMaker);
}

void vis_camera_point(const Eigen::Vector3d &camera_point,double radius = 0.1)
{
        visualization_msgs::Marker sphereMarkers, sphereDeleter;

        sphereMarkers.id = 0;
        sphereMarkers.type = visualization_msgs::Marker::SPHERE_LIST;
        sphereMarkers.header.stamp = ros::Time::now();
        sphereMarkers.header.frame_id = "odom";
        sphereMarkers.pose.orientation.w = 1.00;
        sphereMarkers.action = visualization_msgs::Marker::ADD;
        sphereMarkers.ns = "spheres";
        sphereMarkers.color.r = 1.00;
        sphereMarkers.color.g = 0.00;
        sphereMarkers.color.b = 0.00;
        sphereMarkers.color.a = 1.00;
        sphereMarkers.scale.x = radius * 2.0;
        sphereMarkers.scale.y = radius * 2.0;
        sphereMarkers.scale.z = radius * 2.0;

        sphereDeleter = sphereMarkers;
        sphereDeleter.action = visualization_msgs::Marker::DELETE;

        geometry_msgs::Point point;
        point.x = camera_point(0);
        point.y = camera_point(1);
        point.z = camera_point(2);
        sphereMarkers.points.push_back(point);

        camera_point_vis.publish(sphereDeleter);
        camera_point_vis.publish(sphereMarkers);
}

Eigen::Matrix3d cam_k_, cam_k_inv_;
Eigen::Matrix3d cam2drone_;

 cv_bridge::CvImagePtr cv_ptr;
cv::Mat depth_image;
bool have_depth_msg = false;
void Depth_Img_Cbk(const sensor_msgs::ImageConstPtr &depth_img_msg)
{
        try 
        {
            cv_ptr = cv_bridge::toCvCopy(depth_img_msg, depth_img_msg->encoding.c_str());
        } 
        catch (cv_bridge::Exception& e) 
        {
            ROS_ERROR("cv_bridge execption: %s", e.what());
            return;
        }
        cv_ptr->image.copyTo(depth_image);
        have_depth_msg = true;
        // ROS_WARN("[DEPTH CALL BACK]");
}

bool  getCircleclusterPos(const pcl::PointCloud<pcl::PointXYZ> &input_cloud,
                                                   std::vector<Eigen::Vector3d> &result_cluster,
                                                   Eigen::Vector3d &cluster_origin,
                                                   const double RATIO1_L, const double RATIO1_U,
                                                   const double RATIO2     ,const double min_accept_radius)
{
    if(input_cloud.empty()) return false;

    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = input_cloud.makeShared();

    //voxel filter
    pcl::PointCloud<pcl::PointXYZ> cloud_after_filter;
    pcl::VoxelGrid<pcl::PointXYZ> voxel_sampler;
    voxel_sampler.setLeafSize(0.05f, 0.05f, 0.05f);
    voxel_sampler.setInputCloud(cloud);
    voxel_sampler.filter(cloud_after_filter);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_ptr = cloud_after_filter.makeShared();

    // clustering
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    kdtree->setInputCloud(cloud_filtered_ptr);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> clustering;
    clustering.setClusterTolerance(0.1);
    clustering.setMinClusterSize(100);
    clustering.setMaxClusterSize(5000);
    clustering.setSearchMethod(kdtree);
    clustering.setInputCloud(cloud_filtered_ptr);
    std::vector<pcl::PointIndices> clusters;
    clustering.extract(clusters);


    // ROS_WARN("[clusters size] : %d",clusters.size());
    std::vector<std::vector<Eigen::Vector3d>> potential_result_vec;
    bool flag = false;
    int num = 0;
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
        
        // std::cout << "[evalue] : " << evalue.transpose() << std::endl;
        double ratio1 = evalue(1) / (evalue(2) +0.001);
        double ratio2 = evalue(1) / (evalue(0) +0.0001);
        // std::cout << "[ratio1 & ratio2] : " << ratio1 << " , " << ratio2 << std::endl;
        if(ratio1 > RATIO1_L && ratio1 < RATIO1_U && ratio2 > RATIO2 )
        {
            // std::cout << "[eigen vector]  " << std::endl << evector << std::endl;
            potential_result_vec.push_back(cluster);
            flag = true;
        }
    }
    // ROS_WARN("[potential_result_vec size]   :   %d",potential_result_vec.size());
    if(!flag)  return false;

    //filter clusters by distance 2 it's origin
    std::vector<int> candicates;
    std::vector<Eigen::Vector3d> cluster_origin_vec;
    pcl::PointCloud<pcl::PointXYZ> pt_for_vis_debug;
    for(size_t i = 0 ; i < potential_result_vec.size() ;i++)
    {
        Eigen::Vector3d cluster_origin(0,0,0);
        for(size_t j = 0 ;j < potential_result_vec[i].size();j++)
        {
            pcl::PointXYZ ptt;
            ptt.x = potential_result_vec[i][j](0);
            ptt.y = potential_result_vec[i][j](1);
            ptt.z = potential_result_vec[i][j](2);
            pt_for_vis_debug.points.push_back(ptt);
            cluster_origin = cluster_origin + potential_result_vec[i][j];
        }
        cluster_origin = cluster_origin / potential_result_vec[i].size();
        cluster_origin_vec.push_back(cluster_origin);

        bool potential_cluster_flag = true;
        for(size_t n = 0 ;n < potential_result_vec[i].size();n++)
        {
            double dis2origin = (potential_result_vec[i][n] - cluster_origin).norm(); 
            if(dis2origin < min_accept_radius)  
            {
                potential_cluster_flag = false;
                break;
            }
        }
        if(potential_cluster_flag)  candicates.push_back(i);
    }

    if(candicates.empty()) return false;

    //choose biggest
    double max_candicate_size = 0;
    for(size_t k = 0;k < candicates.size();k++)
    {
        if(max_candicate_size < potential_result_vec[candicates[k]].size())
        {
            max_candicate_size = potential_result_vec[candicates[k]].size();
            result_cluster = potential_result_vec[candicates[k]];
            cluster_origin = cluster_origin_vec[k];
        }
    }
    return true;
}


bool dealWithTransition(const std::vector<Eigen::Vector3d> &transtion_circle_pos_vec,const std::vector<ros::Time> &time_stamp_vec,int &vel_direction)
{
    if(transtion_circle_pos_vec.size() < 4 || time_stamp_vec.size() < 4) return false;

    Eigen::Vector3d first_pos = transtion_circle_pos_vec[transtion_circle_pos_vec.size()-2];
    Eigen::Vector3d second_pos = transtion_circle_pos_vec[transtion_circle_pos_vec.size()-1];

    if(second_pos(1) - first_pos(1) > 0)
    {
        vel_direction = 1;
        return true;
    }
    else if(second_pos(1) - first_pos(1) < 0)
    {
        vel_direction = 2;
        return true;
    }
    return false;
}


//these param should be calibrated
// Eigen::Vector3d rotation_origin(66.00,63.50,2.5);
Eigen::Vector3d rotation_origin(65.9,  63.34,  2.5);
Eigen::Vector3d e1(-0.5,-0.866025,0);
Eigen::Vector3d e2(0,0,1);
Eigen::Vector3d R_normal(-0.866025,0.5,0);
const double rotation_radius = 2.0;   
double theta0;
const double w = 2* M_PI / 8.0;

static double t_fisrt_rotation;
static bool first_rotation = true;
const double time_offset = -0.20;

void vis_rotation_circle(const ros::TimerEvent &event)
{
    if(first_rotation) return;
    double t = ros::Time::now().toSec();
    ROS_ERROR("[vis_rotation_circle]    :   %lf",t);
    if(first_rotation) return;
    Eigen::Vector3d circle_ori = rotation_origin + rotation_radius * (e1 * std::cos(w * (t - t_fisrt_rotation - time_offset) + theta0) + e2 * std::sin(w * (t -t_fisrt_rotation - time_offset) + theta0));
    
     pcl::PointCloud<pcl::PointXYZ> pt_for_vis_debug;
    for(double theta = 0.0 ; theta < 2*M_PI ; theta = theta +0.02)
    {
        Eigen::Vector3d point_on_circle = circle_ori +  e1 * std::cos(theta) + e2 * std::sin(theta);
        pcl::PointXYZ pt;
        pt.x = point_on_circle(0);
        pt.y = point_on_circle(1);
        pt.z = point_on_circle(2);
        pt_for_vis_debug.push_back(pt);
    }
    pt_for_vis_debug.push_back(pcl::PointXYZ(circle_ori(0),circle_ori(1),circle_ori(2)));
    pt_for_vis_debug.push_back(pcl::PointXYZ(rotation_origin(0),rotation_origin(1),rotation_origin(2)));
    sensor_msgs::PointCloud2 circle_pt_vis_msg_;
    pcl::toROSMsg(pt_for_vis_debug,circle_pt_vis_msg_);
    circle_pt_vis_msg_.header.frame_id = "world";
    estimation_pt_vis.publish(circle_pt_vis_msg_);


    geometry_msgs::PoseStamped circle_pos_msg;
    circle_pos_msg.pose.position.x = circle_ori(0);
    circle_pos_msg.pose.position.y = circle_ori(1);
    circle_pos_msg.pose.position.z = circle_ori(2);
    circle_pos_pub.publish(circle_pos_msg);
}


bool dealWithRotation(const std::vector<Eigen::Vector3d> &rotation_circle_pos_vec,const std::vector<ros::Time> &time_stamp_vec)
{
    if(first_rotation)
    {
        t_fisrt_rotation = time_stamp_vec.begin()->toSec();
        first_rotation = false;
    }
    ROS_ERROR("[dealWithRotation]   %lf",time_stamp_vec.back().toSec());
    Eigen::Vector3d circle_pos = rotation_circle_pos_vec.back();
    Eigen::Vector3d V = circle_pos - rotation_origin ;
    Eigen::Vector2d proj2plane;
    e1.normalize();
    proj2plane(0) = e1.dot(V);
    proj2plane(1) = V(2);

    proj2plane.normalize();
    std::cout << proj2plane.norm() <<std::endl;

    theta0 = std::atan2(proj2plane(1), proj2plane(0))  -  w * (time_stamp_vec.back().toSec()-t_fisrt_rotation) ;

    int cycle = theta0 /2 /M_PI;
    theta0 = theta0 - cycle * 2 *M_PI;

    geometry_msgs::PoseStamped circle_pos_msg;
    circle_pos_msg.pose.position.x = theta0;
    circle_pos_pub.publish(circle_pos_msg);
}

int image_length_, image_height_;

std::vector<Eigen::Vector3d> rotation_circle_pos_vec;
std::vector<ros::Time> R_time_stamp_vec;
std::vector<Eigen::Vector3d> transtion_circle_pos_vec;
std::vector<ros::Time> T_time_stamp_vec;
 const int max_odom_vec_size = 30;
 std::vector<nav_msgs::Odometry> odom_vec;
void Odom_Cbk(const nav_msgs::OdometryConstPtr &odom_msg)
{
    nav_msgs::Odometry temp_odom = *odom_msg;
    odom_vec.push_back(temp_odom);
    if(odom_vec.size() < 10) return;
    if(odom_vec.size() > 30) odom_vec.erase(odom_vec.begin());
    if(!have_depth_msg) return;
    have_depth_msg = false;

    ros::Time t_start = ros::Time::now();

    Eigen::Vector3d drone_odom_p_;
    Eigen::Quaterniond drone_odom_q_;

    //odom syn
    nav_msgs::Odometry syn_odom = odom_vec[odom_vec.size() - 6];
    drone_odom_p_(0) = syn_odom.pose.pose.position.x;
    drone_odom_p_(1) = syn_odom.pose.pose.position.y;
    drone_odom_p_(2) = syn_odom.pose.pose.position.z;
    drone_odom_q_.w() = syn_odom.pose.pose.orientation.w;
    drone_odom_q_.x() = syn_odom.pose.pose.orientation.x;
    drone_odom_q_.y() = syn_odom.pose.pose.orientation.y;
    drone_odom_q_.z() = syn_odom.pose.pose.orientation.z;

    cv_ptr->image.copyTo(depth_image);

    Eigen::Matrix3d cam2world = drone_odom_q_ * cam2drone_;
    Eigen::Vector3d cam2world_trans = drone_odom_q_ * Eigen::Vector3d(0.25,0,0);
    Eigen::Vector3d trans = cam2world_trans + drone_odom_p_;
    pcl::PointCloud<pcl::PointXYZ> general_gate_points;
    pcl::PointCloud<pcl::PointXYZ> dynamic_object_points;
    pcl::PointCloud<pcl::PointXYZ> transtion_gate_points;
    pcl::PointCloud<pcl::PointXYZ> rotation_gate_points;
    pcl::PointCloud<pcl::PointXYZ> align_gate_points;
    for(int u = 0; u < image_length_; u = u+1) //x
    {
        for(int v = 0; v < image_height_; v=v+1)//y
        {
            double depth_pixel = depth_image.at<float>(v,u);
             if(depth_pixel  == 0.0) continue;

            Eigen::Vector3d pixel_point = Eigen::Vector3d(u, v, 1);
            Eigen::Vector3d camera_vec = cam_k_inv_ * pixel_point;
            Eigen::Vector3d camera_point = camera_vec * depth_pixel;
            camera_point = cam2world * camera_point + trans;

            // transition circle
            if(camera_point(0) >52.0 && camera_point(0) <56.0 && 
               camera_point(1) >60.0 && camera_point(1) < 72.0&&
               camera_point(2) > 0.0&&camera_point(2)< 5.0)
               {
                    pcl::PointXYZ single_point;
                    single_point.x = camera_point(0); 
                    single_point.y = camera_point(1);
                    single_point.z = camera_point(2);
                    transtion_gate_points.push_back(single_point);   
               }

            //rotation circle
            if((camera_point - Eigen::Vector3d(66.0,63.5,3.5)).norm() < 6.0)
               {
                    pcl::PointXYZ single_point;
                    single_point.x = camera_point(0); 
                    single_point.y = camera_point(1);
                    single_point.z = camera_point(2);
                    rotation_gate_points.push_back(single_point); 
                    continue;  
               }

            //align circle
            if((camera_point - align_gate).norm() < 3.0)
               {
                    pcl::PointXYZ single_point;
                    single_point.x = camera_point(0); 
                    single_point.y = camera_point(1);
                    single_point.z = camera_point(2);
                    align_gate_points.push_back(single_point); 
                    continue;  
               }
            
            //general circle
            for(size_t k = 0; k < gate_list.size();k++)
            {
                double dis2ciecle = (camera_point -gate_list[k]).norm();
                if(dis2ciecle < 3) 
                {
                    pcl::PointXYZ single_point;
                    single_point.x = camera_point(0); 
                    single_point.y = camera_point(1);
                    single_point.z = camera_point(2);
                    general_gate_points.push_back(single_point);
                    break;
                }
            }
        }
    } 

    // sensor_msgs::PointCloud2 circle_pt_vis_msg_;
    // pcl::toROSMsg(rotation_gate_points,circle_pt_vis_msg_);
    // circle_pt_vis_msg_.header.frame_id = "world";
    // circle_pt_vis.publish(circle_pt_vis_msg_);

    circle_detector::detect_circle detect_result_msg;
    bool pub_flag = false;

    //general circles
    const double G_RATIO1_L = 0.75;
    const double G_RATIO1_U = 1.25;
    const double G_RATIO2 = 50;
    const double G_min_accept_radius_general = 0.65;
    Eigen::Vector3d G_result_odom;
    std::vector<Eigen::Vector3d> G_result_cluster;
    bool G_have_result = false;
    if(!general_gate_points.empty())
    G_have_result = getCircleclusterPos(general_gate_points,G_result_cluster,G_result_odom,G_RATIO1_L,G_RATIO1_U,G_RATIO2,G_min_accept_radius_general);
    if(G_have_result)
    {
        pcl::PointCloud<pcl::PointXYZ> pt_for_vis;
        Eigen::Vector3d circle_pos(0,0,0);
        for(size_t i = 0 ; i < G_result_cluster.size() ;i++)
        {
            pcl::PointXYZ ptt;
            ptt.x = G_result_cluster[i](0);
            ptt.y = G_result_cluster[i](1);
            ptt.z = G_result_cluster[i](2);
            pt_for_vis.points.push_back(ptt);
        }

        sensor_msgs::PointCloud2 circle_pt_vis_msg_;
        pcl::toROSMsg(pt_for_vis,circle_pt_vis_msg_);
        circle_pt_vis_msg_.header.frame_id = "world";
        circle_pt_vis.publish(circle_pt_vis_msg_);

        geometry_msgs::PoseStamped circle_pos_msg;
        circle_pos_msg.pose.position.x = G_result_odom(0);
        circle_pos_msg.pose.position.y = G_result_odom(1);
        circle_pos_msg.pose.position.z = G_result_odom(2);
        circle_pos_pub.publish(circle_pos_msg);

        // pub_flag = true;
        // detect_result_msg.have_general_circle = true;
        // detect_result_msg.header.stamp = syn_odom.header.stamp;
        // detect_result_msg.general_circle.id = 0;
        // detect_result_msg.general_circle.x = G_result_odom(0);
        // detect_result_msg.general_circle.y = G_result_odom(1);
        // detect_result_msg.general_circle.z = G_result_odom(2);
        ROS_ERROR("GENERAL  GENERAL GENERAL");
    }

    //transition circle
    const double T_RATIO1_L = 0.75;
    const double T_RATIO1_U = 1.25;
    const double T_RATIO2 = 50;
    const double T_min_accept_radius_general = 0.65;
    Eigen::Vector3d T_result_odom;
    std::vector<Eigen::Vector3d> T_result_cluster;
    bool T_have_result = false;
    if(!transtion_gate_points.empty())
    T_have_result = getCircleclusterPos(transtion_gate_points,T_result_cluster,T_result_odom,G_RATIO1_L,G_RATIO1_U,G_RATIO2,G_min_accept_radius_general);
    if(T_have_result)
    {
        pcl::PointCloud<pcl::PointXYZ> pt_for_vis;
        Eigen::Vector3d circle_pos(0,0,0);
        for(size_t i = 0 ; i < T_result_cluster.size() ;i++)
        {
            pcl::PointXYZ ptt;
            ptt.x = T_result_cluster[i](0);
            ptt.y = T_result_cluster[i](1);
            ptt.z = T_result_cluster[i](2);
            pt_for_vis.points.push_back(ptt);
        }

        sensor_msgs::PointCloud2 circle_pt_vis_msg_;
        pcl::toROSMsg(pt_for_vis,circle_pt_vis_msg_);
        circle_pt_vis_msg_.header.frame_id = "world";
        circle_pt_vis.publish(circle_pt_vis_msg_);

        geometry_msgs::PoseStamped circle_pos_msg;
        circle_pos_msg.pose.position.x = T_result_odom(0);
        circle_pos_msg.pose.position.y = T_result_odom(1);
        circle_pos_msg.pose.position.z = T_result_odom(2);
        circle_pos_pub.publish(circle_pos_msg);


        // pub_flag = true;
        // detect_result_msg.have_transition_circle = true;
        // detect_result_msg.header.stamp = syn_odom.header.stamp;
        // detect_result_msg.transition_circle.x = T_result_odom(0);
        // detect_result_msg.transition_circle.y = T_result_odom(1);
        // detect_result_msg.transition_circle.z = T_result_odom(2);

        ROS_ERROR("TRANSITION  TRANSITION TRANSITION");

        // transtion_circle_pos_vec.push_back(T_result_odom);
        // T_time_stamp_vec.push_back(syn_odom.header.stamp);
        // int vel_direction = 0;
        // dealWithTransition(transtion_circle_pos_vec,T_time_stamp_vec,vel_direction);
    }

    //ratation circle     this param should be careful tuned
    const double R_RATIO1_L = 0.75;
    const double R_RATIO1_U = 1.25;
    const double R_RATIO2 = 50;
    const double R_min_accept_radius_general = 0.5;
    Eigen::Vector3d R_result_odom;
    std::vector<Eigen::Vector3d> R_result_cluster;
    bool R_have_result = false;
    if(!rotation_gate_points.empty())
    R_have_result = getCircleclusterPos(rotation_gate_points,R_result_cluster,R_result_odom,R_RATIO1_L,R_RATIO1_U,R_RATIO2,R_min_accept_radius_general);
    if(R_have_result)
    {
        pcl::PointCloud<pcl::PointXYZ> pt_for_vis;
        Eigen::Vector3d circle_pos(0,0,0);
        for(size_t i = 0 ; i < R_result_cluster.size() ;i++)
        {
            pcl::PointXYZ ptt;
            ptt.x = R_result_cluster[i](0);
            ptt.y = R_result_cluster[i](1);
            ptt.z = R_result_cluster[i](2);
            pt_for_vis.points.push_back(ptt);
        }

        sensor_msgs::PointCloud2 circle_pt_vis_msg_;
        pcl::toROSMsg(pt_for_vis,circle_pt_vis_msg_);
        circle_pt_vis_msg_.header.frame_id = "world";
        circle_pt_vis.publish(circle_pt_vis_msg_);


        

        geometry_msgs::PoseStamped circle_pos_msg;
        circle_pos_msg.pose.position.x = R_result_odom(0);
        circle_pos_msg.pose.position.y = R_result_odom(1);  
        circle_pos_msg.pose.position.z = R_result_odom(2);
        // circle_pos_pub.publish(circle_pos_msg);
        ROS_ERROR("ROTATION  ROTATION  ROTATION");


        rotation_circle_pos_vec.push_back(R_result_odom);
        R_time_stamp_vec.push_back(syn_odom.header.stamp);


        // std::vector<Eigen::Vector3d> test;
        // test.push_back(R_result_odom);
        // std::vector<ros::Time> t_vec;
        // t_vec.push_back(syn_odom.header.stamp);
        // dealWithRotation(test,t_vec);
        static int m = 0;
        dealWithRotation(rotation_circle_pos_vec,R_time_stamp_vec);



        pub_flag = true;
        detect_result_msg.have_rotation_circle = true;
        detect_result_msg.header.stamp = syn_odom.header.stamp;
        detect_result_msg.rotation_circle.x = R_result_odom(0);
        detect_result_msg.rotation_circle.y = R_result_odom(1);
        detect_result_msg.rotation_circle.z = R_result_odom(2);
        detect_result_msg.rotation_circle.radius = 2.0;
        detect_result_msg.rotation_circle.theta0 = theta0;
        detect_result_msg.rotation_circle.time_offset = 0.075;

        // ROS_WARN("HAVE ROTATION RESULT IN  %d",m);
        m++;
    }

    //align circle
    const double A_RATIO1_L = 0.4;
    const double A_RATIO1_U = 1.6;
    const double A_RATIO2 = 30;
    const double A_min_accept_radius_general = 0.01;
    Eigen::Vector3d A_result_odom;
    std::vector<Eigen::Vector3d> A_result_cluster;
    bool A_have_result = false;
    if(!transtion_gate_points.empty())
    R_have_result = getCircleclusterPos(transtion_gate_points,A_result_cluster,A_result_odom,A_RATIO1_L,A_RATIO1_U,A_RATIO2,A_min_accept_radius_general);

    if(A_have_result)
    {
        pcl::PointCloud<pcl::PointXYZ> pt_for_vis;
        Eigen::Vector3d circle_pos(0,0,0);
        for(size_t i = 0 ; i < A_result_cluster.size() ;i++)
        {
            pcl::PointXYZ ptt;
            ptt.x = A_result_cluster[i](0);
            ptt.y = A_result_cluster[i](1);
            ptt.z = A_result_cluster[i](2);
            pt_for_vis.points.push_back(ptt);
        }

        geometry_msgs::PoseStamped circle_pos_msg;
        circle_pos_msg.pose.position.x = A_result_odom(0);
        circle_pos_msg.pose.position.y = A_result_odom(1);
        circle_pos_msg.pose.position.z = A_result_odom(2);
        circle_pos_pub.publish(circle_pos_msg);


        pub_flag = true;
        detect_result_msg.have_alien_circle = true;
        detect_result_msg.header.stamp = syn_odom.header.stamp;
        detect_result_msg.alien_circle.x = A_result_odom(0);
        detect_result_msg.alien_circle.y = A_result_odom(1);
        detect_result_msg.alien_circle.z = A_result_odom(2);

        ROS_ERROR("ALIEN  ALIEN ALIEN");
    }

    if(pub_flag) circle_detect_pub.publish(detect_result_msg);

    //vis result
    // sensor_msgs::PointCloud2 circle_pt_vis_msg_;
    // pcl::toROSMsg(pt_for_vis,circle_pt_vis_msg_);
    // circle_pt_vis_msg_.header.frame_id = "world";
    // circle_pt_vis.publish(circle_pt_vis_msg_);
    ROS_WARN("[total time consume] :  %lf",(ros::Time::now() - t_start).toSec());
    return;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"circle_odom");
    ros::NodeHandle nh;
    circle_pt_vis = nh.advertise<sensor_msgs::PointCloud2>("/circle_pt_vis_topic_d",100);
    circle_orientation_vis = nh.advertise<visualization_msgs::Marker>("/visualizer/mesh", 1000);
    camera_point_vis = nh.advertise<visualization_msgs::Marker>("/camera_point",100);
    estimation_pt_vis = nh.advertise<sensor_msgs::PointCloud2>("/estimation_vis_topic_d",100);

    ros::Timer esti_time = nh.createTimer(ros::Duration(0.1),&vis_rotation_circle);

    depth_sub = nh.subscribe( "/airsim_node/drone_1/front_center/DepthPlanar",1,&Depth_Img_Cbk,ros::TransportHints().tcpNoDelay());
    odom_sub = nh.subscribe("/odom",1,Odom_Cbk,ros::TransportHints().tcpNoDelay());
    circle_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/circle_odom", 1);
    circle_detect_pub = nh.advertise<circle_detector::detect_circle>("/circle_detect_result",1);


    ekfPtr_ = std::make_shared<Ekf>(1.0/ekf_rate);
    cam_k_ << 159.233784156,                                           0,                                                      160,
                0,                                                                      159.233784156,                           120,
                0,                                                                       0,                                                       1;

    cam_k_inv_ = cam_k_.inverse();
    cam2drone_ << 0,    0,    1, 
                -1,     0,    0,
                0,    -1,    0;

    image_height_ = 240;

    image_length_ = 320;

    gate_list.push_back(Eigen::Vector3d( 1.75,     6.92,       2.66));//1
	gate_list.push_back(Eigen::Vector3d( -0.31,     18.91,      3.11));//1
    gate_list.push_back(Eigen::Vector3d( 1.36,      33.71,      3.15));//1
    gate_list.push_back(Eigen::Vector3d( 0.63,      56.9,       1.8));//1
    gate_list.push_back(Eigen::Vector3d( 25.4,      64.2,       2.7));//1
    gate_list.push_back(Eigen::Vector3d( 35.0,      65.8,       2.2));//1
    // gate_list.push_back(Eigen::Vector3d( 64.0,      60.8,       2.2));
    // gate_list.push_back(Eigen::Vector3d( 72.0,      56.8,       2.2));
    // gate_list.push_back(Eigen::Vector3d( 72.0,      52.8,       2.2));
    // gate_list.push_back(Eigen::Vector3d(72.0,       50.8,       2.2));
    gate_list.push_back(Eigen::Vector3d(43.0,      64.4,        2.7));//1
    // gate_list.push_back(Eigen::Vector3d(64.4,      46.0,        2.7));
    // gate_list.push_back(Eigen::Vector3d(54.31,   64.32,      3.52  ));//transition  
    // gate_list.push_back(Eigen::Vector3d(65.85,   63.29,       2.65));//rotation
    // gate_list.push_back(Eigen::Vector3d(69.55,   42.33,       3.15));//alien

    transition_gate = Eigen::Vector3d(54.31,   64.32,      3.52); //1
    rotation_gate = Eigen::Vector3d(65.85,  63.29,         2.65); //1
    align_gate = Eigen::Vector3d(69.55,   42.33,       3.15); //1

    
    while(ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}

