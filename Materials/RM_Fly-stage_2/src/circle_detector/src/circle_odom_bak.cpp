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
#include<geometry_msgs/PoseStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include"circle_detector/config.hpp"

Config config;

ros::Publisher circle_pt_vis;
ros::Publisher circle_orientation_vis;
ros::Publisher estimation_pt_vis;
ros::Publisher camera_point_vis;
ros::Subscriber depth_sub;
ros::Subscriber odom_sub;
ros::Publisher  circle_pos_pub;
ros::Publisher rotation_circle_pub;
ros::Publisher ekf_debug;

std::vector<Eigen::Vector3d> gate_list;
Eigen::Vector3d transition_gate;
Eigen::Vector3d rotation_gate;
Eigen::Vector3d alien_gate;

ros::Publisher transition_circle_pub;

geometry_msgs::PoseStamped msg;

ros::Time last_update_stamp;
bool isFirstEkf = true;
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

    A(0,0) = 1.0;
    A(0,1) = dt;
    A(1,1) = 1.0;

    double t2 = dt * dt / 2;
    B(0, 0) = t2;
    B(1,0) = dt;

    C(0,0) = 1;
    C(1,1) = 1;
    K = C;

    Qt.setIdentity(2, 2);
    Qt(0, 0) = config.ekf_Q_p;
    Qt(1, 1) = config.ekf_Q_v;

    Rt.setIdentity(2, 2);
    Rt(0, 0) = config.ekf_R_p;
    Rt(1, 1) = config.ekf_R_v;

    x.setZero(2);
  }
  
  inline void predict() 
  {
    x = A * x;
    Sigma = A * Sigma * A.transpose() + B * Qt * B.transpose();
    std::cout << "[ekf] predict x : " << x.transpose() << std::endl;

    msg.pose.position.x = x(0);
    msg.pose.position.y = x(1);
    ekf_debug.publish(msg);
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
    std::cout << "[ekf] vmax = " << x(1) << std::endl;
    std::cout << "[ekf]     config.ekf_rate " << config.ekf_rate << std::endl;
    if (x(1) > vmax) 
    {
      return false;
    } else 
    {
      return true;
    }
  }

  inline void update(const Eigen::Vector3d& z,const double &vel) 
  {
    Eigen::Vector2d Z(z(1),vel);
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

void Ekf_Timer(const ros::TimerEvent &event)
{
    double update_dt = (ros::Time::now() - last_update_stamp).toSec();
    std::cout << "[ekf] update_dt  " << update_dt << std::endl;
    if(update_dt < 0.2)
    {
        ekfPtr_->predict();
    }
    else
    {
        ROS_WARN("[ekf] long time no update");
        return;
    }

    //publish
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
sensor_msgs::Image depth_msg; 
cv::Mat depth_image;
bool have_depth_msg = false;
void Depth_Img_Cbk(const sensor_msgs::ImageConstPtr &depth_img_msg)
{
    depth_msg = *depth_img_msg;
        // try 
        // {
        //     cv_ptr = cv_bridge::toCvCopy(depth_img_msg, depth_img_msg->encoding.c_str());
        // } 
        // catch (cv_bridge::Exception& e) 
        // {
        //     ROS_ERROR("cv_bridge execption: %s", e.what());
        //     return;
        // }
        // cv_ptr->image.copyTo(depth_image);
        have_depth_msg = true;
        // ROS_WARN("[DEPTH CALL BACK]");
}

bool  getCircleclusterPos(const pcl::PointCloud<pcl::PointXYZ> &input_cloud,
                                                   std::vector<Eigen::Vector3d> &result_cluster,
                                                   Eigen::Vector3d &cluster_origin,
                                                   const double RATIO1_L, const double RATIO1_U,
                                                   const double RATIO2     ,const double min_accept_radius,
                                                   const double min_leaf_size,const double cluster_tolerance)
{
    if(input_cloud.empty()) return false;

    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = input_cloud.makeShared();

    //voxel filter
    pcl::PointCloud<pcl::PointXYZ> cloud_after_filter;
    pcl::VoxelGrid<pcl::PointXYZ> voxel_sampler;
    voxel_sampler.setLeafSize(min_leaf_size, min_leaf_size, min_leaf_size);
    voxel_sampler.setInputCloud(cloud);
    voxel_sampler.filter(cloud_after_filter);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_ptr = cloud_after_filter.makeShared();

    // clustering
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    kdtree->setInputCloud(cloud_filtered_ptr);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> clustering;
    clustering.setClusterTolerance(cluster_tolerance);
    clustering.setMinClusterSize(config.g_cluster_min_size);
    clustering.setMaxClusterSize(config.g_cluster_max_size);
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
        if(ratio1 > RATIO1_L && ratio1 < RATIO1_U && ratio2 > RATIO2 )
        {
            std::cout << "[ratio1 & ratio2] : " << ratio1 << " , " << ratio2 << std::endl;
            // std::cout << "[eigen vector]  " << std::endl << evector << std::endl;
            potential_result_vec.push_back(cluster);
            flag = true;
        }
    }
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


//below params should be calibrated carefully
Eigen::Vector3d rotation_origin(65.9,  63.34,  2.5);
Eigen::Vector3d e1(-0.5,-0.866025,0);
Eigen::Vector3d e2(0,0,1);
Eigen::Vector3d R_normal(-0.866025,0.5,0);
double rotation_radius = 2.0;   
double theta0;
const double w = 2* M_PI / 8.0;

static ros::Time t_fisrt_rotation;
static bool first_rotation = true;
const double time_offset = -0.0;

void vis_rotation_circle(const ros::TimerEvent &event)
{
    if(first_rotation) return;
    double t = ros::Time::now().toSec();
    // std::cout << std::setprecision(19) << "vis_rotation_circle     " << t << std::endl;
    if(first_rotation) return;
    Eigen::Vector3d circle_ori = rotation_origin + rotation_radius * (e1 * std::cos(w * (t - t_fisrt_rotation.toSec() - time_offset) + theta0) + 
                                                                      e2 * std::sin(w * (t - t_fisrt_rotation.toSec() - time_offset) + theta0));
    
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

//  pcl::PointCloud<pcl::PointXYZ> pt_for_vis_debug;
bool dealWithRotation(const std::vector<Eigen::Vector3d> &rotation_circle_pos_vec,const std::vector<ros::Time> &time_stamp_vec)
{
    if(first_rotation)
    {
        t_fisrt_rotation = *time_stamp_vec.begin();
        first_rotation = false;
    }
    
    std::cout << std::setprecision(19) << "[time gap]" <<ros::Time::now().toSec()-  time_stamp_vec.back().toSec() << std::endl;
    std::cout << std::setprecision(19) << "dealWithRotation     " << time_stamp_vec.back().toSec() << std::endl;
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


    Eigen::Vector3d circle_ori = rotation_origin + rotation_radius * (e1 * std::cos(w * (time_stamp_vec.back().toSec() - t_fisrt_rotation.toSec() - time_offset) + theta0) + 
                                                                      e2 * std::sin(w * (time_stamp_vec.back().toSec() - t_fisrt_rotation.toSec() - time_offset) + theta0));
    
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

int image_length_, image_height_;

std::vector<Eigen::Vector3d> rotation_circle_pos_vec;
std::vector<ros::Time> R_time_stamp_vec;
std::vector<Eigen::Vector3d> transtion_circle_pos_vec;
std::vector<ros::Time> T_time_stamp_vec;
 const int max_odom_vec_size = 30;
//  pcl::PointCloud<pcl::PointXYZ> pt_for_vis_R;
 std::vector<nav_msgs::Odometry> odom_vec;
void Odom_Cbk(const nav_msgs::OdometryConstPtr &odom_msg)
{
    nav_msgs::Odometry temp_odom = *odom_msg;
    odom_vec.push_back(temp_odom);
    if(odom_vec.size() < config.min_odom_vec_size) return;
    if(odom_vec.size() > config.max_odom_vec_size) odom_vec.erase(odom_vec.begin());
    if(!have_depth_msg) return;
    have_depth_msg = false;

    ros::Time t_start = ros::Time::now();

    Eigen::Vector3d drone_odom_p_;
    Eigen::Quaterniond drone_odom_q_;

    //odom syn
    // nav_msgs::Odometry syn_odom = odom_vec[odom_vec.size() - config.frame_delay];
    nav_msgs::Odometry syn_odom;

    double min_time_gap = 1e10;
    for(size_t k = odom_vec.size() ; k > 0 ;k--)
    {
        double temp_t = std::abs(odom_vec[k].header.stamp.toSec() - depth_msg.header.stamp.toSec() + config.frame_delay);
        if(temp_t < min_time_gap)  
        {
            min_time_gap = temp_t;
            syn_odom = odom_vec[k-1];
        }
    }



    drone_odom_p_(0) = syn_odom.pose.pose.position.x;
    drone_odom_p_(1) = syn_odom.pose.pose.position.y;
    drone_odom_p_(2) = syn_odom.pose.pose.position.z;
    drone_odom_q_.w() = syn_odom.pose.pose.orientation.w;
    drone_odom_q_.x() = syn_odom.pose.pose.orientation.x;
    drone_odom_q_.y() = syn_odom.pose.pose.orientation.y;
    drone_odom_q_.z() = syn_odom.pose.pose.orientation.z;


    try 
    {
        cv_ptr = cv_bridge::toCvCopy(depth_msg,depth_msg.encoding.c_str());
    } 
    catch (cv_bridge::Exception& e) 
    {
        ROS_ERROR("cv_bridge execption: %s", e.what());
        return;
    }

    cv_ptr->image.copyTo(depth_image);
    
    Eigen::Matrix3d cam2world = drone_odom_q_ * cam2drone_;
    Eigen::Vector3d cam2world_trans = drone_odom_q_ * Eigen::Vector3d(0.25,0,0);
    Eigen::Vector3d trans = cam2world_trans + drone_odom_p_;
    pcl::PointCloud<pcl::PointXYZ> general_gate_points;
    pcl::PointCloud<pcl::PointXYZ> dynamic_object_points;
    pcl::PointCloud<pcl::PointXYZ> transtion_gate_points;
    pcl::PointCloud<pcl::PointXYZ> rotation_gate_points;
    pcl::PointCloud<pcl::PointXYZ> alien_gate_points;
    pcl::PointCloud<pcl::PointXYZ> vis_all_points;
    for(int u = 0; u < image_length_; u = u + config.skip_pixel) //x
    {
        for(int v = 0; v < image_height_; v=v + config.skip_pixel)//y
        {
            double depth_pixel = depth_image.at<float>(v,u);
             if(depth_pixel  == 0.0) continue;

            Eigen::Vector3d pixel_point = Eigen::Vector3d(u, v, 1);
            Eigen::Vector3d camera_vec = cam_k_inv_ * pixel_point;
            Eigen::Vector3d camera_point = camera_vec * depth_pixel;
            camera_point = cam2world * camera_point + trans;

            pcl::PointXYZ temp;
            temp.x = camera_point(0); 
            temp.y = camera_point(1);
            temp.z = camera_point(2);
            vis_all_points.push_back(temp);

            // transition circle
            if(camera_point(0) > config.transition_circle_box_min_x && 
               camera_point(0) < config.transition_circle_box_max_x && 
               camera_point(1) > config.transition_circle_box_min_y && 
               camera_point(1) < config.transition_circle_box_max_y &&
               camera_point(2) > config.transition_circle_box_min_z &&
               camera_point(2) < config.transition_circle_box_max_z)
            {
                pcl::PointXYZ single_point;
                single_point.x = camera_point(0); 
                single_point.y = camera_point(1);
                single_point.z = camera_point(2);
                transtion_gate_points.push_back(single_point);   
            }

            //rotation circle
            if((camera_point - rotation_gate).norm() < config.r_circle_min_search_radius /* && camera_point(2) > -0.8*/ )
            {
                pcl::PointXYZ single_point;
                single_point.x = camera_point(0); 
                single_point.y = camera_point(1);
                single_point.z = camera_point(2);
                rotation_gate_points.push_back(single_point); 
                continue;  
            }

            //alien circle
            if((camera_point - alien_gate).norm() < config.a_circle_min_search_radius)
            {
                pcl::PointXYZ single_point;
                single_point.x = camera_point(0); 
                single_point.y = camera_point(1);
                single_point.z = camera_point(2);
                alien_gate_points.push_back(single_point); 
                continue;  
            }
            
            //general circle
            for(size_t k = 0; k < gate_list.size();k++)
            {
                double dis2ciecle = (camera_point -gate_list[k]).norm();
                if(dis2ciecle < config.g_circle_min_search_radius) 
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
   
    bool pub_flag = false;

    //general circles
    Eigen::Vector3d G_result_odom;
    std::vector<Eigen::Vector3d> G_result_cluster;
    bool G_have_result = false;
    if(!general_gate_points.empty())
    G_have_result = getCircleclusterPos(general_gate_points,
                                        G_result_cluster,
                                        G_result_odom,
                                        config.g_ratio1_l,
                                        config.g_ratio1_u,
                                        config.g_ratio2,
                                        config.g_min_accept_radius,
                                        config.g_cluster_leaf_size,
                                        config.g_cluster_tolerance);
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

        std::cout << "[general] pos   " << G_result_odom.transpose() <<std::endl;
        ROS_ERROR("GENERAL  GENERAL GENERAL");
    }

    //transition circle
    Eigen::Vector3d T_result_odom;
    std::vector<Eigen::Vector3d> T_result_cluster;
    bool T_have_result = false;
    if(!transtion_gate_points.empty())
    T_have_result = getCircleclusterPos(transtion_gate_points,
                                        T_result_cluster,
                                        T_result_odom,
                                        config.t_ratio1_l,
                                        config.t_ratio1_u,
                                        config.t_ratio2,
                                        config.t_min_accept_radius,
                                        config.g_cluster_leaf_size,
                                        config.g_cluster_tolerance);
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

        //vis
        sensor_msgs::PointCloud2 circle_pt_vis_msg_;
        pcl::toROSMsg(pt_for_vis,circle_pt_vis_msg_);
        circle_pt_vis_msg_.header.frame_id = "world";
        circle_pt_vis.publish(circle_pt_vis_msg_);

        static std::vector<std::pair<Eigen::Vector3d,ros::Time> > T_pos_stamp_pair_vec;
        T_pos_stamp_pair_vec.push_back(std::make_pair(T_result_odom,syn_odom.header.stamp));


        double vel = 0.0;
        if(T_pos_stamp_pair_vec.size() > 5)
        {
            vel =  (T_pos_stamp_pair_vec.back().first(1) - T_pos_stamp_pair_vec.begin()->first(1) ) / ( T_pos_stamp_pair_vec.back().second - T_pos_stamp_pair_vec.begin()->second).toSec();
            T_pos_stamp_pair_vec.erase(T_pos_stamp_pair_vec.begin());
            msg.pose.position.z = vel;
            std::cout << "[ekf] vel " << vel << std::endl;
        }

        std::cout << "[ekf] transition circle pos   " << T_result_odom.transpose() << std::endl;
        double update_dt = (ros::Time::now() - last_update_stamp).toSec();
        if(update_dt > 0.2)
        {
            ekfPtr_->reset(T_result_odom);
            ROS_WARN("!!!!!ekf reset!!!!!");
        }
        else if(ekfPtr_->checkValid(T_result_odom))
        {
            ekfPtr_->update(T_result_odom,vel);
            double pos_y = ekfPtr_->pos();
            double vel_y = ekfPtr_->vel(); 

            geometry_msgs::PoseStamped circle_pos_msg;
            circle_pos_msg.header.stamp = syn_odom.header.stamp;
            circle_pos_msg.pose.position.x = T_result_odom(0);
            circle_pos_msg.pose.position.y = pos_y;
            circle_pos_msg.pose.position.z = T_result_odom(2);
            circle_pos_msg.pose.orientation.y = vel_y;

            transition_circle_pub.publish(circle_pos_msg);

            circle_pos_msg.pose.position.x = T_result_odom(0);
            circle_pos_msg.pose.position.y = T_result_odom(1);
            circle_pos_msg.pose.position.z = T_result_odom(2);
            circle_pos_pub.publish(circle_pos_msg);
        }
        else
        {
            ROS_ERROR("!!!!!ekf update invalid!!!!!");
        }
        last_update_stamp = ros::Time::now();

        ROS_ERROR("TRANSITION  TRANSITION TRANSITION");

        // transtion_circle_pos_vec.push_back(T_result_odom);
        // T_time_stamp_vec.push_back(syn_odom.header.stamp);
        // int vel_direction = 0;
        // dealWithTransition(transtion_circle_pos_vec,T_time_stamp_vec,vel_direction);
    }

    //ratation circle     this param should be careful tuned
    Eigen::Vector3d R_result_odom;
    std::vector<Eigen::Vector3d> R_result_cluster;
    bool R_have_result = false;
    if(!rotation_gate_points.empty())
    R_have_result = getCircleclusterPos(rotation_gate_points,
                                        R_result_cluster,
                                        R_result_odom,
                                        config.r_ratio1_l,
                                        config.r_ratio1_u,
                                        config.r_ratio2,
                                        config.r_min_accept_radius,
                                        config.g_cluster_leaf_size,
                                        config.g_cluster_tolerance);
    if(R_have_result)
    {
        // static pcl::PointCloud<pcl::PointXYZ> pt_for_vis_R;
        
        // Eigen::Vector3d circle_pos(0,0,0);
        // for(size_t i = 0 ; i < R_result_cluster.size() ;i++)
        // {
        //     pcl::PointXYZ ptt;
        //     ptt.x = R_result_cluster[i](0);
        //     ptt.y = R_result_cluster[i](1);
        //     ptt.z = R_result_cluster[i](2);
        //     pt_for_vis_R.points.push_back(ptt);
        // }

        // //vis
        // sensor_msgs::PointCloud2 circle_pt_vis_msg_;
        // pcl::toROSMsg(pt_for_vis_R,circle_pt_vis_msg_);
        // circle_pt_vis_msg_.header.frame_id = "world";
        // circle_pt_vis.publish(circle_pt_vis_msg_);


    
        ROS_ERROR("ROTATION  ROTATION  ROTATION");


        rotation_circle_pos_vec.push_back(R_result_odom);
        R_time_stamp_vec.push_back(syn_odom.header.stamp);


        std::vector<Eigen::Vector3d> test;
        test.push_back(R_result_odom);
        std::vector<ros::Time> t_vec;
        t_vec.push_back(syn_odom.header.stamp);
        dealWithRotation(test,t_vec);
        // dealWithRotation(rotation_circle_pos_vec,R_time_stamp_vec);

        geometry_msgs::PoseStamped rotation_circle_msg;
        rotation_circle_msg.pose.position.x = R_result_odom(0);
        rotation_circle_msg.pose.position.y = R_result_odom(1);
        rotation_circle_msg.pose.position.z = R_result_odom(2);
        rotation_circle_msg.header.stamp = t_fisrt_rotation;
        rotation_circle_msg.pose.orientation.w = theta0;
        rotation_circle_pub.publish(rotation_circle_msg);


        geometry_msgs::PoseStamped circle_pos_msg;
        circle_pos_msg.pose.position.x = R_result_odom(0);
        circle_pos_msg.pose.position.y = R_result_odom(1);
        circle_pos_msg.pose.position.z = R_result_odom(2);
        circle_pos_pub.publish(circle_pos_msg);

        if(R_result_odom(2) < -0.8)
        {
            std::cout << "[alien bug]  "  << R_result_odom.transpose() << std::endl;
            
            pcl::PointCloud<pcl::PointXYZ> pt_for_vis_R;
            
            Eigen::Vector3d circle_pos(0,0,0);
            for(size_t i = 0 ; i < R_result_cluster.size() ;i++)
            {
                pcl::PointXYZ ptt;
                ptt.x = R_result_cluster[i](0);
                ptt.y = R_result_cluster[i](1);
                ptt.z = R_result_cluster[i](2);
                pt_for_vis_R.points.push_back(ptt);
            }

            //vis
            sensor_msgs::PointCloud2 circle_pt_vis_msg_;
            pcl::toROSMsg(pt_for_vis_R,circle_pt_vis_msg_);
            circle_pt_vis_msg_.header.frame_id = "world";
            circle_pt_vis.publish(circle_pt_vis_msg_);
            cv::waitKey(0);
        } 

        std::cout << "[rotation] pos  " << R_result_odom.transpose() << std::endl;
    }

    //align circle
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
                                        0.02,
                                        config.g_cluster_tolerance);

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

        std::cout << "[alien] pos   " << A_result_odom.transpose() <<std::endl;
        sensor_msgs::PointCloud2 circle_pt_vis_msg_;
        pcl::toROSMsg(pt_for_vis,circle_pt_vis_msg_);
        circle_pt_vis_msg_.header.frame_id = "world";
        circle_pt_vis.publish(circle_pt_vis_msg_);

        ROS_ERROR("ALIEN  ALIEN ALIEN");
    }

    ROS_WARN("[total time consume] :  %lf",(ros::Time::now() - t_start).toSec());
    return;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"circle_odom");
    ros::NodeHandle nh("~");

    config.init(nh);

    circle_pt_vis = nh.advertise<sensor_msgs::PointCloud2>("/circle_pt_vis_topic_d",100);
    circle_orientation_vis = nh.advertise<visualization_msgs::Marker>("/visualizer/mesh", 1000);

    camera_point_vis = nh.advertise<visualization_msgs::Marker>("/camera_point",100);
    estimation_pt_vis = nh.advertise<sensor_msgs::PointCloud2>("/estimation_vis_topic_d",100);

    ros::Timer esti_timer = nh.createTimer(ros::Duration(0.01),&vis_rotation_circle);

    last_update_stamp =ros::Time::now();
    ros::Timer ekf_timer = nh.createTimer(ros::Duration(1.0/config.ekf_rate),&Ekf_Timer);
    
    depth_sub = nh.subscribe(config.depth_topic,1,&Depth_Img_Cbk,ros::TransportHints().tcpNoDelay());
    odom_sub = nh.subscribe(config.odom_topic,1,Odom_Cbk,ros::TransportHints().tcpNoDelay());
    

    ekf_debug = nh.advertise<geometry_msgs::PoseStamped>("/ekf_debug", 1);
    circle_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/circle_odom", 1);
    rotation_circle_pub = nh.advertise<geometry_msgs::PoseStamped>("/rotation_circle",1);
    transition_circle_pub = nh.advertise<geometry_msgs::PoseStamped>("/transition_circle",1);

    ekfPtr_ = std::make_shared<Ekf>(1.0/config.ekf_rate);

    cam_k_ << config.fx,      0,          config.cx,
                0,            config.fy,  config.cy,
                0,            0,          1;

    cam_k_inv_ = cam_k_.inverse();
    cam2drone_ << 0,    0,    1, 
                 -1,     0,    0,
                 0,    -1,    0;

    image_height_ = 240;

    image_length_ = 320;

    rotation_origin << config.rotation_origin[0] ,config.rotation_origin[1] ,config.rotation_origin[2];
    e1 << config.e1[0], config.e1[1], config.e1[2];
    R_normal << config.normal[0] ,config.normal[1] ,config.normal[2];
    rotation_radius = config.rotation_radius;

    for(size_t i = 0 ;i < config.gate_list_.size() / 3 ;i++)
    {
        Eigen::Vector3d temp_point;
        temp_point << config.gate_list_[3 * i], config.gate_list_[3 * i + 1], config.gate_list_[3 * i + 2];
        gate_list.push_back(temp_point);
    }

    transition_gate << config.transition_gate[0] ,config.transition_gate[1] ,config.transition_gate[2];
    rotation_gate << config.rotation_gate[0] ,config.rotation_gate[1] ,config.rotation_gate[2];
    alien_gate << config.alien_gate[0] ,config.alien_gate[1] ,config.alien_gate[2];

    while(ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}
