/*
    MIT License

    Copyright (c) 2021 Hongkai Ye (kyle_yeh@163.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/
#pragma once

#include <ros/ros.h>

#include "config.hpp"
#include "minco/minco_time_varied.hpp"
#include "minco/minco_time_uniform.hpp"
#include "visualization/visualization.hpp"
#include "misc/stage_2_moving_obs.hpp"
#include "minco/flatness.hpp"
// #define DEBUG

namespace traj_opt {

class GlbTrajReplanOpt {
   public:
    ros::NodeHandle nh_;
    Config config_;

    bool fixed_end_;
    int waypoints_num_, waypoints_dim_;
    int N_, dim_t_, dim_p_;
    Eigen::MatrixXd waypoints_;
    Eigen::VectorXd fixed_;
    int gate_constrain_index_;

    Eigen::Vector3d gate_pos_;

    // collision avoiding and dynamics paramters
    double squared_vel_limit_, squared_acc_limit_, squared_jrk_limit_, thrAccMinSqr, thrAccMaxSqr;

    // Minimum Jerk Optimizer
    minco::varied_T::MinJerkOpt jerkOpt_;
    // waypoints
    Eigen::VectorXd p_;
    // duration of each piece of the trajectory
    Eigen::VectorXd t_;

    // visualizer
    std::shared_ptr<visualization::Visualization> visPtr_;

    Eigen::VectorXd x_;
    Eigen::MatrixXd ini_state_, end_state_;

    // for penaly cost storage
    double vel_pnt_, acc_pnt_, jrk_pnt_, thrmin_pnt_, thrmax_pnt_;

    // for stage 2
    std::shared_ptr<Circle12> circle_12_ptr_;
    std::shared_ptr<Circle13> circle_13_ptr_;
    std::shared_ptr<MovingObs0> moving_obs_0_ptr_;
    std::shared_ptr<DynObs> moving_obs_ptr_;
    int circle_12_index_, circle_13_index_;
    ros::Time timestamp_;
    double obs_radius_sqr_;
    double pos_pnt_;
    double des_EndROll;

    Eigen::Vector3d circle_12_direction_ = Eigen::Vector3d( 1.0,  0.0,  0.0);
    // for stage 2 se3
    minco::varied_T::MinJerkOpt landing_jerkOpt_;
    int landN_, land_dim_p;
    Eigen::MatrixXd landWaypoints;
    Eigen::MatrixXd landFinStates;
    flatness::FlatnessMap flatmap;

   public:
    GlbTrajReplanOpt(ros::NodeHandle& nh, Config& conf);
    ~GlbTrajReplanOpt() {}

    bool generate_traj(const Eigen::MatrixXd& iniState,
                       const Eigen::MatrixXd& finState,
                       const Eigen::MatrixXd& waypoints,
                       const Eigen::VectorXd& Ts,
                       const Eigen::VectorXd& fixed,
                       const Eigen::Vector3d& gate_pos,
                       const int &gate_constrain_index,
                       Trajectory& traj);

    void setVisualizer(const std::shared_ptr<visualization::Visualization>& visPtr) {
        visPtr_ = visPtr;
    };

    void setCircle12(const std::shared_ptr<Circle12>& circle_12_Ptr) {
        circle_12_ptr_ = circle_12_Ptr;
    };

    void setCircle13(const std::shared_ptr<Circle13>& circle_13_Ptr) {
        circle_13_ptr_ = circle_13_Ptr;
    };

    void setMovingObs0(const std::shared_ptr<MovingObs0>& moving_obs_0_Ptr) {
        moving_obs_0_ptr_ = moving_obs_0_Ptr;
    };

    void setMovingObs(const std::shared_ptr<DynObs>& moving_obs_Ptr) {
        moving_obs_ptr_ = moving_obs_Ptr;
    };

    void setTimestamp(const ros::Time& time_stamp) {
        timestamp_ = time_stamp;
    }

    void setVel(const double& vel)
    {
        config_.max_vel = vel;
        squared_vel_limit_ = vel * vel;
    }

    void setRhoT(const double& rhoT)
    {
        config_.rhoT = rhoT;
    }

    bool generate_traj_r2(const Eigen::MatrixXd& iniState,
                          const Eigen::MatrixXd& finState,
                          const Eigen::MatrixXd& waypoints,
                          const Eigen::VectorXd& Ts,
                          const Eigen::VectorXd& fixed,
                          const Eigen::Vector3d& gate_pos,
                          const int &gate_constrain_index,
                          Trajectory& traj);
    
    int optimize_r2(const double& delta = 1e-5);

    static double objectiveFunc_r2(void *ptrObj, const Eigen::VectorXd &x, Eigen::VectorXd& grad);

    double calTimeIntPenalty_r2();

    bool DynObsCostGrad(const double& t,
                        const Eigen::Vector3d& pos,
                        const Eigen::Vector3d& vel,
                        Eigen::Vector3d& grad_tmp,
                        double& gradt,
                        double& grad_prev_t,
                        double& cost_tmp);

    static void addLayerPGrad_r2(const Eigen::Ref<const Eigen::MatrixXd>& gradInPs,
                                 Eigen::Ref<Eigen::MatrixXd> grad,
                                 const Eigen::VectorXd& fixed,
                                 const int &gate_constrain_index,
                                 const int &circle_12_index,
                                 const int &circle_13_index,
                                 const bool &see_circle_12,
                                 const bool &see_circle_13);

    bool generate_traj_se3(const Eigen::MatrixXd& iniState,
                           const Eigen::MatrixXd& finState,
                           const Eigen::MatrixXd& waypoints,
                           const Eigen::VectorXd& Ts,
                           Trajectory& traj,
                           const Eigen::Vector3d& hoverPoint);

    int optimize_se3(const double& delta = 1e-5);

    static double objectiveFunc_se3(void *ptrObj, const Eigen::VectorXd &x, Eigen::VectorXd& grad);

    static void addLayerTGrad_se3(const Eigen::Ref<const Eigen::VectorXd>& t,
                                  const Eigen::Ref<const Eigen::VectorXd>& gradT,
                                  Eigen::Ref<Eigen::VectorXd> gradt);

    static void addLayerPGrad_se3(const Eigen::Ref<const Eigen::MatrixXd>& gradInPs,
                              Eigen::Ref<Eigen::MatrixXd> grad);

    bool generate_traj_r2_dakeai(const Eigen::MatrixXd& iniState,
                                 const Eigen::MatrixXd& finState,
                                 const Eigen::MatrixXd& waypoints,
                                 const Eigen::VectorXd& Ts,
                                 const Eigen::VectorXd& fixed,
                                 const Eigen::Vector3d& gate_pos,
                                 const int &gate_constrain_index,
                                 Trajectory& traj);

   private:
    void setBoundConds(const Eigen::MatrixXd& iniState, const Eigen::MatrixXd& finState);
    void roundingState(Eigen::MatrixXd& state);
    int optimize(const double& delta = 1e-5);

    double calTimeIntPenalty();
    double calSe3EndPenalty();
    double calLandTimeIntPenalty();
    double calSe3BeginPenalty();
    bool highDerivativeCostGrad_vel(const Eigen::Vector3d &derivative,
                                    Eigen::Vector3d &grad,
                                    double &cost);
    bool highDerivativeCostGrad_acc(const Eigen::Vector3d &derivative,
                                    Eigen::Vector3d &grad,
                                    double &cost);
    bool highDerivativeCostGrad_jrk(const Eigen::Vector3d &derivative,
                                    Eigen::Vector3d &grad,
                                    double &cost);
    static double objectiveFunc(void *ptrObj, const Eigen::VectorXd &x, Eigen::VectorXd& grad);

    static int earlyExit(void* ptrObj,
                         const double* x,
                         const double* grad,
                         const double fx,
                         const double xnorm,
                         const double gnorm,
                         const double step,
                         int n,
                         int k,
                         int ls);

    static void forwardT(const Eigen::Ref<const Eigen::VectorXd>& t, Eigen::Ref<Eigen::VectorXd> vecT);
    static void backwardT(const Eigen::Ref<const Eigen::VectorXd>& vecT, Eigen::Ref<Eigen::VectorXd> t);
    static void addLayerTGrad(const Eigen::Ref<const Eigen::VectorXd>& t,
                              const Eigen::Ref<const Eigen::VectorXd>& gradT,
                              Eigen::Ref<Eigen::VectorXd> gradt,
                              const Eigen::VectorXd& fixed,
                              const int &gate_constrain_index);
    static void addLayerPGrad(const Eigen::Ref<const Eigen::MatrixXd>& gradInPs,
                              Eigen::Ref<Eigen::MatrixXd> grad,
                              const Eigen::VectorXd& fixed,
                              const int &gate_constrain_index);

    // this is a smoothed C2 exact penalty
    // inputs x >0, output exact penalty and penalty derivates
    static void positiveSmoothedL1(const double& x,
                                   const double& seps,
                                   double& f,
                                   double& df);
    static double d_quasi_exp(const double& x);
    static double expC2(double t);
    static double logC2(double T);
};

}  // namespace traj_opt