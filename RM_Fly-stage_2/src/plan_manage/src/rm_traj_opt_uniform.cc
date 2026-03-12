#include <traj_opt/rm_traj_opt_uniform.h>

#include <traj_opt/chalk.hpp>
#include <traj_opt/lbfgs_raw.hpp>

namespace traj_opt {
RmTrajOptUniform::RmTrajOptUniform(ros::NodeHandle &nh, Config &conf)
    : nh_(nh), config_(conf), gdT_(2), propagated_gdT_(2) {
  squared_vel_limit_ = config_.max_vel * config_.max_vel;
  squared_acc_limit_ = config_.max_acc * config_.max_acc;
  squared_jrk_limit_ = config_.max_jrk * config_.max_jrk;
}

bool RmTrajOptUniform::generate_traj(const Eigen::MatrixXd &iniState,
                                     const Eigen::MatrixXd &midState,
                                     const Eigen::MatrixXd &finState,
                                     const Eigen::MatrixXd &waypoints_1,
                                     const Eigen::MatrixXd &waypoints_2,
                                     const double &dt_1,
                                     const double &dt_2,
                                     const Eigen::Vector3d &gate_pos,
                                     Trajectory &traj) {
    std::cout << "---------------------------------------------------------------" << std::endl;
    std::cout << "[before] iniState: \n" << iniState.transpose() << std::endl;
    std::cout << "[before] midState: \n" << midState.transpose() << std::endl;
    std::cout << "[before] finState: \n" << finState.transpose() << std::endl;
    std::cout << "[before] waypoints_1: \n" << waypoints_1.transpose() << std::endl;
    std::cout << "[before] waypoints_2: \n" << waypoints_2.transpose() << std::endl;
    std::cout << "[before] dt_1: " << dt_1 << std::endl;
    std::cout << "[before] dt_2: " << dt_2 << std::endl;
    std::cout << "[before] gate_pos: \n" << gate_pos.transpose() << std::endl;

    ini_state_ = iniState;
    mid_state_ = midState;
    end_state_ = finState;

    gate_pos_ = gate_pos;

    waypoints_1_ = waypoints_1;
    waypoints_2_ = waypoints_2;

    waypoints_dim_ = waypoints_1.rows();
    dim_p_1_ = waypoints_dim_ * waypoints_1.cols();
    dim_p_2_ = waypoints_dim_ * waypoints_2.cols();

    N_1_ = waypoints_1.cols() + 1;
    N_2_ = waypoints_2.cols() + 1;

    dim_t_ = 2;
    state_order_ = midState.cols();
    dim_mid_state_ = waypoints_dim_ * state_order_;

    Eigen::VectorXd total_T;
    total_T.resize(dim_t_);
    total_T << dt_1 * N_1_, dt_2 * N_2_;

    roundingState(ini_state_);
    roundingState(mid_state_);
    roundingState(end_state_);
    setBoundConds(ini_state_, mid_state_, end_state_);
    x_.resize(dim_p_1_ + dim_p_2_ + dim_t_ + dim_mid_state_);
    Eigen::Map<Eigen::MatrixXd> P1(x_.data(), waypoints_dim_, N_1_ - 1);
    Eigen::Map<Eigen::MatrixXd> P2(x_.data()+ dim_p_1_, waypoints_dim_, N_2_ - 1);
    Eigen::Map<Eigen::VectorXd> total_t(x_.data() + dim_p_1_ + dim_p_2_, dim_t_);
    Eigen::Map<Eigen::MatrixXd> mid_state(x_ .data()+ dim_p_1_ + dim_p_2_ + dim_t_, waypoints_dim_, state_order_);

    P1 = waypoints_1;
    P2 = waypoints_2;
    backwardT(total_T, total_t);
    mid_state = midState;

    cost_function_cb_times_ = 0;

    jerkOpt_1_.generate(P1, total_T[0]);
    jerkOpt_2_.generate(P2, total_T[1]);

    Trajectory vis_traj;
    vis_traj = jerkOpt_1_.getTraj();
    vis_traj.append(jerkOpt_2_.getTraj());

    double t = 0;
    double dt = 0.02;
    double t_scale = 3.0;
    double duration = vis_traj.getTotalDuration();

    std::vector<Eigen::Vector3d> vn_list, vx_list, vy_list, vz_list;
    std::vector<Eigen::Vector3d> an_list, ax_list, ay_list, az_list;
    std::vector<Eigen::Vector3d> jn_list, jx_list, jy_list, jz_list;

    for( t = 0; t < duration; t += dt)
    {
        auto v = vis_traj.getVel(t);
        auto a = vis_traj.getAcc(t);
        auto j = vis_traj.getJerk(t);

        vn_list.push_back(Eigen::Vector3d(t * t_scale, v.norm(), 0));
        vx_list.push_back(Eigen::Vector3d(t * t_scale, v[0], 0));
        vy_list.push_back(Eigen::Vector3d(t * t_scale, v[1], 0));
        vz_list.push_back(Eigen::Vector3d(t * t_scale, v[2], 0));
        an_list.push_back(Eigen::Vector3d(t * t_scale, a.norm(), 0));
        ax_list.push_back(Eigen::Vector3d(t * t_scale, a[0], 0));
        ay_list.push_back(Eigen::Vector3d(t * t_scale, a[1], 0));
        az_list.push_back(Eigen::Vector3d(t * t_scale, a[2], 0));
        jn_list.push_back(Eigen::Vector3d(t * t_scale, j.norm(), 0));
        jx_list.push_back(Eigen::Vector3d(t * t_scale, j[0], 0));
        jy_list.push_back(Eigen::Vector3d(t * t_scale, j[1], 0));
        jz_list.push_back(Eigen::Vector3d(t * t_scale, j[2], 0));
    }

    visPtr_->visualize_path(vn_list, "vnorm_init");
    visPtr_->visualize_path(vx_list, "vx_init");
    visPtr_->visualize_path(vy_list, "vy_init");
    visPtr_->visualize_path(vz_list, "vz_init");
    visPtr_->visualize_path(an_list, "anorm_init");
    visPtr_->visualize_path(ax_list, "ax_init");
    visPtr_->visualize_path(ay_list, "ay_init");
    visPtr_->visualize_path(az_list, "az_init");
    visPtr_->visualize_path(jn_list, "jnorm_init");
    visPtr_->visualize_path(jx_list, "jx_init");
    visPtr_->visualize_path(jy_list, "jy_init");
    visPtr_->visualize_path(jz_list, "jz_init");

    int opt_ret = optimize();

    forwardT(total_t, total_T);

    std::cout << "[after] iniState: \n" << iniState.transpose() << std::endl;
    std::cout << "[after] midState: \n" << mid_state.transpose() << std::endl;
    std::cout << "[after] finState: \n" << finState.transpose() << std::endl;
    std::cout << "[after] waypoints_1: \n" << P1.transpose() << std::endl;
    std::cout << "[after] waypoints_2: \n" << P2.transpose() << std::endl;
    std::cout << "[after] dt_1: " << total_T[0]/N_1_ << std::endl;
    std::cout << "[after] dt_2: " << total_T[1]/N_2_ << std::endl;
    std::cout << "[after] gate_pos: \n" << gate_pos_.transpose() << std::endl;
    std::cout << "---------------------------------------------------------------" << std::endl;

    if (opt_ret < 0)
    {
        return false;
    }

    jerkOpt_1_.resetTailCon(mid_state);
    jerkOpt_2_.resetHeadCon(mid_state);
    jerkOpt_1_.generate(P1, total_T[0]);
    jerkOpt_2_.generate(P2, total_T[1]);

    traj = jerkOpt_1_.getTraj();
    traj.append(jerkOpt_2_.getTraj());

    return true;
}

int RmTrajOptUniform::optimize(const double &delta) {
  // Setup for L-BFGS solver
  lbfgs::lbfgs_parameter_t lbfgs_params;
  lbfgs_params.mem_size = 16;
  lbfgs_params.past = 3;
  lbfgs_params.g_epsilon = 0.0;
  lbfgs_params.min_step = 1e-32;
  lbfgs_params.delta = 1.0e-4;
  double minObjective;
  auto ret = lbfgs::lbfgs_optimize(
      x_, minObjective, RmTrajOptUniform::objectiveFunc, nullptr,
       nullptr, this, lbfgs_params);//RmTrajOptUniform::Process
    //   nullptr, this, &lbfgs_params);

  std::cout << "\033[32m"
            << "ret: " << ret << ", minObjective: " << minObjective << "\033[0m"
            << std::endl;
  return ret;
}

inline void RmTrajOptUniform::roundingState(Eigen::MatrixXd &state) {
    double tempNorm = state.col(1).norm();
    if (tempNorm > config_.max_vel)
    {
        state.col(1) *= (config_.max_vel - 0.001) / tempNorm;
        state.col(2) *= 0.0;
    }

    tempNorm = state.col(2).norm();
    if (tempNorm > config_.max_acc)
    {
        state.col(2) *= (config_.max_acc - 0.001) / tempNorm;
    }
}

inline void RmTrajOptUniform::setBoundConds(const Eigen::MatrixXd &iniState,
                                            const Eigen::MatrixXd &midState,
                                            const Eigen::MatrixXd &finState)
{
    jerkOpt_1_.reset(iniState, midState, N_1_);
    jerkOpt_2_.reset(midState, finState, N_2_);
}

double RmTrajOptUniform::calGatePenalty(const Eigen::MatrixXd& state, Eigen::Ref<Eigen::MatrixXd> grad)
{
    double cost = 0.0;
    double violation = (state.col(0) - gate_pos_).squaredNorm();
    // if (violation > 0.0)
    // {
        cost = config_.pnlGate * violation;
        grad.col(0) += config_.pnlGate * 2.0 * (state.col(0) - gate_pos_);
    // }
    // std::cout << "gate_pnt : " << cost << std::endl;
    return cost;
}

double RmTrajOptUniform::calTimeIntPenalty(const int& N,
                                           Eigen::MatrixX3d& gdC,
                                           double& gdT,
                                           double total_T,
                                           const Eigen::MatrixX3d& coeffs) {
  Eigen::Vector3d pos, vel, acc, jrk, snp;
  Eigen::Vector3d grad_tmp;
  double cost_tmp;
  static Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4;
  double s1, s2, s3, s4, s5;
  double step_vaj, omega, alpha;
  Eigen::Matrix<double, 6, 3> grad_Jv_to_c, grad_Ja_to_c, grad_Jj_to_c;
  Eigen::Matrix<double, 6, 3> integral_grad_Jv_to_c, integral_grad_Ja_to_c,
      integral_grad_Jj_to_c;
  double grad_Jv_to_T, grad_Ja_to_T, grad_Jj_to_T;
  double integral_grad_Jv_to_T, integral_grad_Ja_to_T, integral_grad_Jj_to_T;
  double integral_cost_v, integral_cost_a, integral_cost_j;

  vel_pnt_ = 0.0;
  acc_pnt_ = 0.0;
  jrk_pnt_ = 0.0;
  double accumulated_dur(0.0);
  for (int i = 0; i < N; ++i) {
    const auto &c = coeffs.block<6, 3>(i * 6, 0);
    integral_grad_Jv_to_c.setZero();
    integral_grad_Ja_to_c.setZero();
    integral_grad_Jj_to_c.setZero();
    integral_grad_Jv_to_T = 0.0;
    integral_grad_Ja_to_T = 0.0;
    integral_grad_Jj_to_T = 0.0;
    integral_cost_v = 0.0;
    integral_cost_a = 0.0;
    integral_cost_j = 0.0;

    step_vaj = (total_T / N) / config_.K;
    s1 = 0.0;
    for (int j = 0; j <= config_.K; ++j, s1 += step_vaj) {
      s2 = s1 * s1;
      s3 = s2 * s1;
      s4 = s2 * s2;
      s5 = s4 * s1;
      beta0[0] = 1.0;
      beta0[1] = s1;
      beta0[2] = s2;
      beta0[3] = s3;
      beta0[4] = s4;
      beta0[5] = s5;
      beta1[0] = 0.0;
      beta1[1] = 1.0;
      beta1[2] = 2.0 * s1;
      beta1[3] = 3.0 * s2;
      beta1[4] = 4.0 * s3;
      beta1[5] = 5.0 * s4;
      beta2[0] = 0.0;
      beta2[1] = 0.0;
      beta2[2] = 2.0;
      beta2[3] = 6.0 * s1;
      beta2[4] = 12.0 * s2;
      beta2[5] = 20.0 * s3;
      beta3[0] = 0.0;
      beta3[1] = 0.0;
      beta3[2] = 0.0;
      beta3[3] = 6.0;
      beta3[4] = 24.0 * s1;
      beta3[5] = 60.0 * s2;
      beta4[0] = 0.0;
      beta4[1] = 0.0;
      beta4[2] = 0.0;
      beta4[3] = 0.0;
      beta4[4] = 24.0;
      beta4[5] = 120.0 * s1;
      pos = c.transpose() * beta0;
      vel = c.transpose() * beta1;
      acc = c.transpose() * beta2;
      jrk = c.transpose() * beta3;
      snp = c.transpose() * beta4;
      alpha = (double)j / config_.K;
      omega = (j == 0 || j == config_.K) ? 0.5 : 1.0;
      if (highDerivativeCostGrad_vel(vel, grad_tmp, cost_tmp)) {
        grad_Jv_to_c = beta1 * grad_tmp.transpose();
        grad_Jv_to_T = alpha * grad_tmp.dot(acc);
        integral_grad_Jv_to_c += omega * grad_Jv_to_c;
        integral_grad_Jv_to_T +=
            omega * (cost_tmp / config_.K + step_vaj * grad_Jv_to_T);
        integral_cost_v += omega * cost_tmp;
      }
      if (highDerivativeCostGrad_acc(acc, grad_tmp, cost_tmp)) {
        grad_Ja_to_c = beta2 * grad_tmp.transpose();
        grad_Ja_to_T = alpha * grad_tmp.dot(jrk);
        integral_grad_Ja_to_c += omega * grad_Ja_to_c;
        integral_grad_Ja_to_T +=
            omega * (cost_tmp / config_.K + step_vaj * grad_Ja_to_T);
        integral_cost_a += omega * cost_tmp;
      }
      if (highDerivativeCostGrad_jrk(jrk, grad_tmp, cost_tmp)) {
        grad_Jj_to_c = beta3 * grad_tmp.transpose();
        grad_Jj_to_T = alpha * grad_tmp.dot(snp);
        integral_grad_Jj_to_c += omega * grad_Jj_to_c;
        integral_grad_Jj_to_T +=
            omega * (cost_tmp / config_.K + step_vaj * grad_Jj_to_T);
        integral_cost_j += omega * cost_tmp;
      }
    }
    vel_pnt_ += step_vaj * integral_cost_v;
    acc_pnt_ += step_vaj * integral_cost_a;
    jrk_pnt_ += step_vaj * integral_cost_j;

    gdC.block<6, 3>(i * 6, 0) +=
        step_vaj * (integral_grad_Jv_to_c * config_.pnlV +
                    integral_grad_Ja_to_c * config_.pnlA +
                    integral_grad_Jj_to_c * config_.pnlJ);
    gdT += (integral_grad_Jv_to_T * config_.pnlV +
           integral_grad_Ja_to_T * config_.pnlA +
           integral_grad_Jj_to_T * config_.pnlJ) / N;

    accumulated_dur += total_T / N;
  }
  vel_pnt_ *= config_.pnlV;
  acc_pnt_ *= config_.pnlA;
  jrk_pnt_ *= config_.pnlJ;

    // std::cout << "vel_pnt : " << vel_pnt_ << "\n"
    //           << "acc_pnt : " << acc_pnt_ << "\n"
    //           << "jrk_pnt : " << jrk_pnt_ << std::endl;

  return (vel_pnt_ + acc_pnt_ + jrk_pnt_);
}

/*
* For penalty of higher derivatives like vel\acc\jrk\...
f = max(v^2 - v_max^2, 0)
*/
bool RmTrajOptUniform::highDerivativeCostGrad_vel(const Eigen::Vector3d &derivative,
                                                  Eigen::Vector3d &grad,
                                                  double &cost) {
    bool ret(false);
    cost = 0.0;
    grad.setZero();
    double sq_v_diff = derivative.squaredNorm() - squared_vel_limit_;
    if (sq_v_diff > DBL_EPSILON)
    {
        double violaPena(sq_v_diff), violaPenaGrad(1.0);
        positiveSmoothedL1(sq_v_diff, config_.smoothEps, violaPena, violaPenaGrad);
        grad = 2.0 * derivative * violaPenaGrad;
        cost += violaPena;
        ret = true;
    }

    return ret;
}

bool RmTrajOptUniform::highDerivativeCostGrad_acc(const Eigen::Vector3d &derivative,
                                                  Eigen::Vector3d &grad,
                                                  double &cost) {
    bool ret(false);
    cost = 0.0;
    grad.setZero();
    // acceleration
    double sq_a_diff = derivative.squaredNorm() - squared_acc_limit_;
    if (sq_a_diff > DBL_EPSILON)
    {
        double violaPena(sq_a_diff), violaPenaGrad(1.0);
        positiveSmoothedL1(sq_a_diff, config_.smoothEps, violaPena, violaPenaGrad);
        grad = 2.0 * derivative * violaPenaGrad;
        cost += violaPena;
        ret = true;
    }

    return ret;
}

bool RmTrajOptUniform::highDerivativeCostGrad_jrk(const Eigen::Vector3d &derivative,
                                                  Eigen::Vector3d &grad,
                                                  double &cost) {
    bool ret(false);
    cost = 0.0;
    grad.setZero();
    // jerk
    double sq_j_diff = derivative.squaredNorm() - squared_jrk_limit_;
    if (sq_j_diff > DBL_EPSILON)
    {
        double violaPena(sq_j_diff), violaPenaGrad(1.0);
        positiveSmoothedL1(sq_j_diff, config_.smoothEps, violaPena, violaPenaGrad);
        grad = 2.0 * derivative * violaPenaGrad;
        cost += violaPena;
        ret = true;
    }

    return ret;
}

// SECTION object function
double RmTrajOptUniform::objectiveFunc(void *ptrObj, const Eigen::VectorXd &x, Eigen::VectorXd& grad) {
    RmTrajOptUniform *obj = reinterpret_cast<RmTrajOptUniform *>(ptrObj);

    Eigen::Map<const Eigen::MatrixXd> P1(x.data(), obj->waypoints_dim_, obj->N_1_ - 1);
    Eigen::Map<const Eigen::MatrixXd> P2(x.data() + obj->dim_p_1_, obj->waypoints_dim_, obj->N_2_ - 1);
    Eigen::Map<const Eigen::VectorXd> total_t(x.data() + obj->dim_p_1_ + obj->dim_p_2_, obj->dim_t_);
    Eigen::Map<const Eigen::MatrixXd> mid_state(x.data() + obj->dim_p_1_ + obj->dim_p_2_ + obj->dim_t_, obj->waypoints_dim_, obj->state_order_);
    Eigen::Map<Eigen::MatrixXd> grad_p_1(grad.data(), obj->waypoints_dim_, obj->N_1_ - 1);
    Eigen::Map<Eigen::MatrixXd> grad_p_2(grad.data() + obj->dim_p_1_, obj->waypoints_dim_, obj->N_2_ - 1);
    Eigen::Map<Eigen::VectorXd> grad_total_t(grad.data() + obj->dim_p_1_ + obj->dim_p_2_, obj->dim_t_);
    Eigen::Map<Eigen::MatrixXd> grad_mid_state(grad.data() + obj->dim_p_1_ + obj->dim_p_2_ + obj->dim_t_, obj->waypoints_dim_, obj->state_order_);

    Eigen::VectorXd total_T;
    total_T.resize(obj->dim_t_);
    forwardT(total_t, total_T);

    obj->jerkOpt_1_.resetTailCon(mid_state);
    obj->jerkOpt_2_.resetHeadCon(mid_state);

    obj->jerkOpt_1_.generate(P1, total_T[0]);
    obj->jerkOpt_2_.generate(P2, total_T[1]);

    double cost = 0.0;
    cost += obj->jerkOpt_1_.getEnergy();
    cost += obj->jerkOpt_2_.getEnergy();

    // std::cout << "--------------------------------------------------" << std::endl;
    // std::cout << "energy_cost : " << cost << std::endl;

    obj->jerkOpt_1_.getEnergyPartialGradByCoeffs(obj->gdC_1_);
    obj->jerkOpt_2_.getEnergyPartialGradByCoeffs(obj->gdC_2_);
    obj->jerkOpt_1_.getEnergyPartialGradByTotalTime(obj->gdT_[0]);
    obj->jerkOpt_2_.getEnergyPartialGradByTotalTime(obj->gdT_[1]);

    double time_itl_cost_1 = obj->calTimeIntPenalty(obj->N_1_, obj->gdC_1_, obj->gdT_[0], total_T[0], obj->jerkOpt_1_.getCoeffs());
    double time_itl_cost_2 = obj->calTimeIntPenalty(obj->N_2_, obj->gdC_2_, obj->gdT_[1], total_T[1], obj->jerkOpt_2_.getCoeffs());

    cost += time_itl_cost_1;
    cost += time_itl_cost_2;

    obj->jerkOpt_1_.propogateGrad(obj->gdC_1_, obj->gdT_[0], obj->gdP_1_, obj->propagated_gdT_[0]);
    obj->jerkOpt_2_.propogateGrad(obj->gdC_2_, obj->gdT_[1], obj->gdP_2_, obj->propagated_gdT_[1]);

    Eigen::MatrixXd tmp_mid_state_grad;
    tmp_mid_state_grad.resize(obj->waypoints_dim_, obj->state_order_);

    grad_mid_state.setZero();

    obj->jerkOpt_1_.getPartailGradsByEndState(tmp_mid_state_grad);
    grad_mid_state += tmp_mid_state_grad.transpose();
    obj->jerkOpt_2_.getPartailGradsByStartState(tmp_mid_state_grad);
    grad_mid_state += tmp_mid_state_grad.transpose();

    cost += obj->calGatePenalty(mid_state, grad_mid_state);

    // double violation = (mid_state.col(0) - obj->gate_pos_).squaredNorm() - 0.04;
    // if (violation > 0.0)
    // {
    //     cost += obj->config_.pnlGate * violation;
    //     grad_mid_state.col(0) += obj->config_.pnlGate * 2.0 * (mid_state.col(0) - obj->gate_pos_);
    // }

    // std::cout << grad_mid_state.transpose() << std::endl;

    cost += obj->config_.rhoT * total_T.sum();
    obj->propagated_gdT_.array() += obj->config_.rhoT;

    addLayerTGrad(total_t, obj->propagated_gdT_, grad_total_t);
    addLayerPGrad(obj->gdP_1_, grad_p_1);
    addLayerPGrad(obj->gdP_2_, grad_p_2);

    // grad_total_t.setZero();
    // grad_mid_state.col(0).setZero();

    // std::cout << "--- i :" << obj->cost_function_cb_times_ << "----------------------------------------" << std::endl;
    // std::cout << grad_p_1.transpose() << std::endl;
    // std::cout << grad_p_2.transpose() << std::endl;
    // obj->cost_function_cb_times_++;

    return cost;
}

inline int RmTrajOptUniform::Process(void *ptrObj, const double *x,
                                  const double *grad, const double fx,
                                  const double xnorm, const double gnorm,
                                  const double step, int n, int k, int ls) {
    RmTrajOptUniform *obj = reinterpret_cast<RmTrajOptUniform *>(ptrObj);

    if (obj->config_.debug == false)
    {
        return 0;
    }

    Eigen::Map<const Eigen::VectorXd> tmp_grad(grad, obj->dim_p_1_ + obj->dim_p_2_ + obj->dim_t_ + obj->dim_mid_state_);
    std::cout << "grad_norm = " << tmp_grad.norm() << std::endl;

    Eigen::Map<const Eigen::MatrixXd> P1(x, obj->waypoints_dim_, obj->N_1_ - 1);
    Eigen::Map<const Eigen::MatrixXd> P2(x + obj->dim_p_1_, obj->waypoints_dim_, obj->N_2_ - 1);
    Eigen::Map<const Eigen::VectorXd> total_t(x + obj->dim_p_1_ + obj->dim_p_2_, obj->dim_t_);
    Eigen::Map<const Eigen::MatrixXd> mid_state(x + obj->dim_p_1_ + obj->dim_p_2_ + obj->dim_t_, obj->waypoints_dim_, obj->state_order_);

    Eigen::VectorXd total_T;
    total_T.resize(obj->dim_t_);
    forwardT(total_t, total_T);

    obj->jerkOpt_1_.resetTailCon(mid_state);
    obj->jerkOpt_2_.resetHeadCon(mid_state);

    obj->jerkOpt_1_.generate(P1, total_T[0]);
    obj->jerkOpt_2_.generate(P2, total_T[1]);

    Trajectory traj;
    traj = obj->jerkOpt_1_.getTraj();
    traj.append(obj->jerkOpt_2_.getTraj());

    obj->visPtr_->visualize_traj(traj, "debug_traj");

    double t = 0;
    double dt = 0.01;
    double t_scale = 3.0;
    double duration = traj.getTotalDuration();

    std::vector<Eigen::Vector3d> vn_list, vx_list, vy_list, vz_list;
    std::vector<Eigen::Vector3d> an_list, ax_list, ay_list, az_list;
    std::vector<Eigen::Vector3d> jn_list, jx_list, jy_list, jz_list;

    for( t = 0; t < duration; t += dt)
    {
        auto v = traj.getVel(t);
        auto a = traj.getAcc(t);
        auto j = traj.getJerk(t);

        vn_list.push_back(Eigen::Vector3d(t * t_scale, v.norm(), 0));
        vx_list.push_back(Eigen::Vector3d(t * t_scale, v[0], 0));
        vy_list.push_back(Eigen::Vector3d(t * t_scale, v[1], 0));
        vz_list.push_back(Eigen::Vector3d(t * t_scale, v[2], 0));
        an_list.push_back(Eigen::Vector3d(t * t_scale, a.norm(), 0));
        ax_list.push_back(Eigen::Vector3d(t * t_scale, a[0], 0));
        ay_list.push_back(Eigen::Vector3d(t * t_scale, a[1], 0));
        az_list.push_back(Eigen::Vector3d(t * t_scale, a[2], 0));
        jn_list.push_back(Eigen::Vector3d(t * t_scale, j.norm(), 0));
        jx_list.push_back(Eigen::Vector3d(t * t_scale, j[0], 0));
        jy_list.push_back(Eigen::Vector3d(t * t_scale, j[1], 0));
        jz_list.push_back(Eigen::Vector3d(t * t_scale, j[2], 0));
    }

    obj->visPtr_->visualize_path(vn_list, "vnorm");
    obj->visPtr_->visualize_path(vx_list, "vx");
    obj->visPtr_->visualize_path(vy_list, "vy");
    obj->visPtr_->visualize_path(vz_list, "vz");
    obj->visPtr_->visualize_path(an_list, "anorm");
    obj->visPtr_->visualize_path(ax_list, "ax");
    obj->visPtr_->visualize_path(ay_list, "ay");
    obj->visPtr_->visualize_path(az_list, "az");
    obj->visPtr_->visualize_path(jn_list, "jnorm");
    obj->visPtr_->visualize_path(jx_list, "jx");
    obj->visPtr_->visualize_path(jy_list, "jy");
    obj->visPtr_->visualize_path(jz_list, "jz");

    ros::Duration(0.01).sleep();

    return 0;
}

inline void RmTrajOptUniform::forwardT(const Eigen::Ref<const Eigen::VectorXd> &t,
                                  Eigen::Ref<Eigen::VectorXd> vecT) {
  int M = t.size();
  for (int i = 0; i < M; ++i) {
    vecT(i) = expC2(t(i));
  }
  return;
}

inline void RmTrajOptUniform::backwardT(
    const Eigen::Ref<const Eigen::VectorXd> &vecT,
    Eigen::Ref<Eigen::VectorXd> t) {
  int M = t.size();
  for (int i = 0; i < M; ++i) {
    t(i) = logC2(vecT(i));
  }
  return;
}

inline void RmTrajOptUniform::addLayerTGrad(
    const Eigen::Ref<const Eigen::VectorXd> &t,
    const Eigen::Ref<const Eigen::VectorXd> &gradT,
    Eigen::Ref<Eigen::VectorXd> gradt) {
  int dim_t = t.size();
  for (int i = 0; i < dim_t; ++i) {
    gradt[i] = gradT[i] * d_quasi_exp(t[i]);
  }
}

inline void RmTrajOptUniform::addLayerPGrad(
    const Eigen::Ref<const Eigen::MatrixXd> &gradInPs,
    Eigen::Ref<Eigen::MatrixXd> grad) {
    grad = gradInPs;
    return;
}

// this is a smoothed C2 exact penalty
// inputs x >0, output exact penalty and penalty derivates
inline void RmTrajOptUniform::positiveSmoothedL1(const double &x, const double &seps,
                                            double &f, double &df) {
  static const double pe = seps;
  static const double half = 0.5 * pe;
  static const double f3c = 1.0 / (pe * pe);
  static const double f4c = -0.5 * f3c / pe;
  static const double d2c = 3.0 * f3c;
  static const double d3c = 4.0 * f4c;

  if (x < pe) {
    f = (f4c * x + f3c) * x * x * x;
    df = (d3c * x + d2c) * x * x;
  } else {
    f = x - half;
    df = 1.0;
  }

  return;
}
inline double RmTrajOptUniform::d_quasi_exp(const double &x) {
  if (x > 0.0) {
    return x + 1.0;
  } else {
    double denSqrt = (0.5 * x - 1.0) * x + 1.0;
    return (1.0 - x) / (denSqrt * denSqrt);
  }
  // return 2.0 * x;
}
double RmTrajOptUniform::expC2(double t) {
  return t > 0.0 ? ((0.5 * t + 1.0) * t + 1.0)
                 : 1.0 / ((0.5 * t - 1.0) * t + 1.0);
  // return 0.5 + t * t;
}
double RmTrajOptUniform::logC2(double T) {
  return T > 1.0 ? (sqrt(2.0 * T - 1.0) - 1.0) : (1.0 - sqrt(2.0 / T - 1.0));
  // return sqrt(std::max(T - 0.5, 0.0));
}

}  // namespace traj_opt
