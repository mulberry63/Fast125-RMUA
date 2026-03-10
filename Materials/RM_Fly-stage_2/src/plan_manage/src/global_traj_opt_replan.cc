#include <traj_opt/global_traj_opt_replan.h>

#include <traj_opt/chalk.hpp>
#include <traj_opt/lbfgs_raw.hpp>

namespace traj_opt {
GlbTrajReplanOpt::GlbTrajReplanOpt(ros::NodeHandle &nh, Config &conf)
    : nh_(nh), config_(conf), landFinStates(3, 3) {
    squared_vel_limit_ = config_.max_vel * config_.max_vel;
    squared_acc_limit_ = config_.max_acc * config_.max_acc;
    squared_jrk_limit_ = config_.max_jrk * config_.max_jrk;
    thrAccMinSqr = config_.min_thracc * config_.min_thracc;
    thrAccMaxSqr = config_.max_thracc * config_.max_thracc;
    obs_radius_sqr_ = config_.obs_radius * config_.obs_radius;
    des_EndROll = config_.desEndROll;
    flatmap.reset(1.0,9.81,0.0,0.0,0.0,0.0001);
}

bool GlbTrajReplanOpt::generate_traj(const Eigen::MatrixXd &iniState,
                                const Eigen::MatrixXd &finState,
                                const Eigen::MatrixXd &waypoints,
                                const Eigen::VectorXd &Ts,
                                const Eigen::VectorXd &fixed,
                                const Eigen::Vector3d &gate_pos,
                                const int &gate_constrain_index, Trajectory &traj) {
    std::cout << "[before] iniState: \n" << iniState.transpose() << std::endl;
    std::cout << "[before] finState: \n" << finState.transpose() << std::endl;
    std::cout << "[before] waypoints: \n" << waypoints.transpose() << std::endl;
    std::cout << "[before] Ts: " << Ts.transpose() << std::endl;
    std::cout << "[before] fixed: " << fixed.transpose() << std::endl;
    std::cout << "[before] gate_pos: \n" << gate_pos.transpose() << std::endl;
    std::cout << "[before] gate_constrain_index: \n" << gate_constrain_index << std::endl;

    ini_state_ = iniState;
    end_state_ = finState;
    fixed_ = fixed;
    gate_pos_ = gate_pos;
    gate_constrain_index_ = gate_constrain_index;

    N_ = Ts.size();

    waypoints_num_ = waypoints.cols();
    waypoints_dim_ = waypoints.rows();
    dim_p_ = waypoints_num_ * waypoints_dim_;
    dim_t_ = N_;

    x_.resize(dim_p_ + dim_t_);
    Eigen::Map<Eigen::MatrixXd> p(x_.data(), waypoints_dim_, waypoints_num_);
    Eigen::Map<Eigen::VectorXd> t(x_.data() + dim_p_, dim_t_);

    p = waypoints;
    backwardT(Ts, t);

    roundingState(ini_state_);
    roundingState(end_state_);
    setBoundConds(ini_state_, end_state_);

    int opt_ret = optimize();

    Eigen::VectorXd Ts_after(N_);
    forwardT(t, Ts_after);

    std::cout << "[after] iniState: \n" << iniState.transpose() << std::endl;
    std::cout << "[after] finState: \n" << finState.transpose() << std::endl;
    std::cout << "[after] waypoints: \n" << p.transpose() << std::endl;
    std::cout << "[after] Ts: " << Ts_after.transpose() << std::endl;
    std::cout << "[after] fixed: " << fixed.transpose() << std::endl;
    std::cout << "[after] gate_pos: \n" << gate_pos_.transpose() << std::endl;
    std::cout << "[after] gate_constrain_index: \n" << gate_constrain_index << std::endl;
    std::cout << "---------------------------------------------------------------" << std::endl;

    jerkOpt_.generate(p, Ts_after);
    traj = jerkOpt_.getTraj();
    return true;
}

int GlbTrajReplanOpt::optimize(const double &delta) {
  // Setup for L-BFGS solver
  lbfgs::lbfgs_parameter_t lbfgs_params;
  lbfgs_params.mem_size = 256;
  lbfgs_params.past = 10;
  lbfgs_params.g_epsilon = 0.0;
  lbfgs_params.min_step = 1e-32;
  lbfgs_params.delta = 1.0e-4;
  double minObjective;
  auto ret = lbfgs::lbfgs_optimize(
      x_, minObjective, GlbTrajReplanOpt::objectiveFunc, nullptr,
    //    &GlbTrajReplanOpt::earlyExit, this, &lbfgs_params);
      nullptr, this, lbfgs_params);

  std::cout << "\033[32m"
            << "ret: " << ret << ", minObjective: " << minObjective << "\033[0m"
            << std::endl;
  return ret;
}

inline void GlbTrajReplanOpt::roundingState(Eigen::MatrixXd &state) {
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

inline void GlbTrajReplanOpt::setBoundConds(const Eigen::MatrixXd &iniState,
                                       const Eigen::MatrixXd &finState) {
  jerkOpt_.reset(iniState, finState, N_);
}

double GlbTrajReplanOpt::calTimeIntPenalty() {
    Eigen::Vector3d pos, vel, acc, jrk, snp;
    Eigen::Vector3d grad_tmp;
    double cost_tmp;
    static Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4;
    double s1, s2, s3, s4, s5;
    double step_vaj, omega, alpha;
    Eigen::Matrix<double, 6, 3> grad_Jv_to_c, grad_Ja_to_c, grad_Jj_to_c, grad_Jminthr_to_c, grad_Jmaxthr_to_c;
    Eigen::Matrix<double, 6, 3> integral_grad_Jv_to_c, integral_grad_Ja_to_c, integral_grad_Jj_to_c, integral_grad_Jminthr_to_c, integral_grad_Jmaxthr_to_c;
    double grad_Jv_to_T, grad_Ja_to_T, grad_Jj_to_T, grad_Jminthr_to_T, grad_Jmaxthr_to_T;
    double integral_grad_Jv_to_T, integral_grad_Ja_to_T, integral_grad_Jj_to_T, integral_grad_Jminthr_to_T, integral_grad_Jmaxthr_to_T;
    double integral_cost_v, integral_cost_a, integral_cost_j, integral_cost_minthr, integral_cost_maxthr;

    vel_pnt_ = 0.0;
    acc_pnt_ = 0.0;
    jrk_pnt_ = 0.0;
    thrmax_pnt_ = 0.0;
    thrmin_pnt_ = 0.0;
    double accumulated_dur(0.0);
    for (int i = 0; i < N_; ++i) {
        const auto &c = jerkOpt_.b.block<6, 3>(i * 6, 0);
        integral_grad_Jv_to_c.setZero();
        integral_grad_Ja_to_c.setZero();
        integral_grad_Jj_to_c.setZero();
        integral_grad_Jminthr_to_c.setZero();
        integral_grad_Jmaxthr_to_c.setZero();
        integral_grad_Jv_to_T = 0.0;
        integral_grad_Ja_to_T = 0.0;
        integral_grad_Jj_to_T = 0.0;
        integral_grad_Jminthr_to_T = 0.0;
        integral_grad_Jmaxthr_to_T = 0.0;
        integral_cost_v = 0.0;
        integral_cost_a = 0.0;
        integral_cost_j = 0.0;
        integral_cost_minthr = 0.0;
        integral_cost_maxthr = 0.0;

        step_vaj = jerkOpt_.T1(i) / config_.K;
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
        //hzc thrust penalty
        Eigen::Vector3d thr = acc + Eigen::Vector3d(0.0,0.0,9.81);
        Eigen::Vector3d dSqrMagThr = 2 * thr;
        double fthr = thr.norm();
        double sqrMagThr = fthr * fthr;
        double violaThrl = thrAccMinSqr - sqrMagThr;
        double violaThrh = sqrMagThr - thrAccMaxSqr;
        // std::cout <<"thrAccMinSqr: "<< thrAccMinSqr<<" thrAccMaxSqr: "<<thrAccMaxSqr<<std::endl;
        if (violaThrl > 0.0)
        {
            double violaPena(violaThrl), violaPenaGrad(1.0);
            /*
            grad = 2 * derivative * violaPenaGrad;
            cost += violaPena;
            */
            positiveSmoothedL1(violaThrl, config_.smoothEps, violaPena, violaPenaGrad);
            grad_Jminthr_to_c = -violaPenaGrad * beta2 * dSqrMagThr.transpose();
            grad_Jminthr_to_T = -violaPenaGrad * alpha * dSqrMagThr.transpose() * jrk;
            integral_grad_Jminthr_to_c += omega * grad_Jminthr_to_c;
            integral_grad_Jminthr_to_T +=
                omega * (violaPena / config_.K + step_vaj * grad_Jminthr_to_T);
            integral_cost_minthr += omega * violaPena;
        }

        if (violaThrh > 0.0)
        {
            double violaPena(violaThrh), violaPenaGrad(1.0);
            positiveSmoothedL1(violaThrh, config_.smoothEps, violaPena, violaPenaGrad);
            // std::cout <<"violaPena: "<<violaPena<<" violaThrh: "<<violaThrh<<" sqrMagThr: "<<sqrMagThr<< " acc: "<<acc.transpose()<<std::endl;

            grad_Jmaxthr_to_c = violaPenaGrad * beta2 * dSqrMagThr.transpose();
            grad_Jmaxthr_to_T = violaPenaGrad * alpha * dSqrMagThr.transpose() * jrk;
            integral_grad_Jmaxthr_to_c += omega * grad_Jmaxthr_to_c;
            integral_grad_Jmaxthr_to_T +=
                omega * (violaPena / config_.K + step_vaj * grad_Jmaxthr_to_T);
            integral_cost_maxthr += omega * violaPena;
        }
        }
        vel_pnt_ += step_vaj * integral_cost_v;
        acc_pnt_ += step_vaj * integral_cost_a;
        jrk_pnt_ += step_vaj * integral_cost_j;

        jerkOpt_.gdC.block<6, 3>(i * 6, 0) +=
            step_vaj * (integral_grad_Jv_to_c * config_.pnlV +
                        integral_grad_Ja_to_c * config_.pnlA +
                        integral_grad_Jj_to_c * config_.pnlJ);
        jerkOpt_.gdT(i) += integral_grad_Jv_to_T * config_.pnlV +
                        integral_grad_Ja_to_T * config_.pnlA +
                        integral_grad_Jj_to_T * config_.pnlJ;

        accumulated_dur += jerkOpt_.T1(i);
    }
    vel_pnt_ *= config_.pnlV;
    acc_pnt_ *= config_.pnlA;
    jrk_pnt_ *= config_.pnlJ;

    // std::cout << "================== v_cost: " << vel_pnt_ << std::endl;
    // std::cout << "================== a_cost: " << acc_pnt_ << std::endl;
    // std::cout << "================== j_cost: " << jrk_pnt_ << std::endl;

    return (vel_pnt_ + acc_pnt_ + jrk_pnt_);
}

/*
* For penalty of higher derivatives like vel\acc\jrk\...
f = max(v^2 - v_max^2, 0)
*/
bool GlbTrajReplanOpt::highDerivativeCostGrad_vel(const Eigen::Vector3d &derivative,
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

bool GlbTrajReplanOpt::highDerivativeCostGrad_acc(const Eigen::Vector3d &derivative,
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

bool GlbTrajReplanOpt::highDerivativeCostGrad_jrk(const Eigen::Vector3d &derivative,
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
double GlbTrajReplanOpt::objectiveFunc(void *ptrObj, const Eigen::VectorXd &x, Eigen::VectorXd & grad) {
    GlbTrajReplanOpt &obj = *(GlbTrajReplanOpt *)ptrObj;

    Eigen::Map<const Eigen::MatrixXd> P(x.data(), obj.waypoints_dim_, obj.waypoints_num_);
    Eigen::Map<const Eigen::VectorXd> t(x.data() + obj.dim_p_, obj.dim_t_);
    Eigen::Map<Eigen::MatrixXd> gradp(grad.data(), obj.waypoints_dim_, obj.waypoints_num_);
    Eigen::Map<Eigen::VectorXd> gradt(grad.data() + obj.dim_p_, obj.dim_t_);

    Eigen::VectorXd T(obj.N_);
    forwardT(t, T);

    obj.jerkOpt_.generate(P, T);
    double cost = obj.jerkOpt_.getTrajJerkCost();

    obj.jerkOpt_.calGrads_CT();
    double time_itl_cost = obj.calTimeIntPenalty();
    cost += time_itl_cost;

    obj.jerkOpt_.calGrads_PT();

    double violation = (P.col(obj.gate_constrain_index_) - obj.gate_pos_).squaredNorm();
    if (violation > 0.0)
    {
        cost += obj.config_.pnlGate * violation;
        obj.jerkOpt_.gdP.col(obj.gate_constrain_index_) += obj.config_.pnlGate * 2.0 * (P.col(obj.gate_constrain_index_) - obj.gate_pos_);
    }

    /* add cost of rhoT * Ts and its grads to Ts */
    obj.jerkOpt_.gdT.array() += obj.config_.rhoT;
    cost += obj.config_.rhoT * T.sum();

    addLayerTGrad(t, obj.jerkOpt_.gdT, gradt, obj.fixed_, obj.gate_constrain_index_);
    addLayerPGrad(obj.jerkOpt_.gdP, gradp, obj.fixed_, obj.gate_constrain_index_);

    return cost;
}

inline int GlbTrajReplanOpt::earlyExit(void *ptrObj, const double *x,
                                  const double *grad, const double fx,
                                  const double xnorm, const double gnorm,
                                  const double step, int n, int k, int ls) {
  // return k > 1e3;
  GlbTrajReplanOpt &obj = *(GlbTrajReplanOpt *)ptrObj;
  if (obj.vel_pnt_ <= 0.01 && obj.acc_pnt_ <= 0.01 && obj.jrk_pnt_ <= 0.01) {
    return 1;
  };
}

inline void GlbTrajReplanOpt::forwardT(const Eigen::Ref<const Eigen::VectorXd> &t,
                                  Eigen::Ref<Eigen::VectorXd> vecT) {
  int M = t.size();
  for (int i = 0; i < M; ++i) {
    vecT(i) = expC2(t(i));
  }
  return;
}

inline void GlbTrajReplanOpt::backwardT(
    const Eigen::Ref<const Eigen::VectorXd> &vecT,
    Eigen::Ref<Eigen::VectorXd> t) {
  int M = t.size();
  for (int i = 0; i < M; ++i) {
    t(i) = logC2(vecT(i));
  }
  return;
}

inline void GlbTrajReplanOpt::addLayerTGrad(
    const Eigen::Ref<const Eigen::VectorXd> &t,
    const Eigen::Ref<const Eigen::VectorXd> &gradT,
    Eigen::Ref<Eigen::VectorXd> gradt,
    const Eigen::VectorXd& fixed,
    const int &gate_constrain_index) {
    int dim_t = t.size();
    for (int i = 0; i < dim_t; ++i) {
        gradt[i] = gradT[i] * d_quasi_exp(t[i]);
    }

    int i, j;
    for (i = 0, j = 0; i < dim_t - 1; i++)
    {
        if (fixed[i] == 1 || i == gate_constrain_index)
        {
            gradt.segment(j, i - j + 1) = Eigen::VectorXd::Constant(i - j + 1, gradt.segment(j, i - j + 1).mean());
            j = i + 1;
        }
    }
    gradt.segment(j, dim_t - j) = Eigen::VectorXd::Constant(dim_t - j, gradt.segment(j, dim_t - j).mean());
}

inline void GlbTrajReplanOpt::addLayerPGrad(
    const Eigen::Ref<const Eigen::MatrixXd> &gradInPs,
    Eigen::Ref<Eigen::MatrixXd> grad,
    const Eigen::VectorXd& fixed,
    const int &gate_constrain_index) {
    for ( int i = 0; i < grad.cols(); i++)
    {
        if (fixed[i] == 1 && i != gate_constrain_index)
        {
            grad.col(i).setZero();
        }
        else
        {
            grad.col(i) = gradInPs.col(i);
        }
    }
    return;
}

// this is a smoothed C2 exact penalty
// inputs x >0, output exact penalty and penalty derivates
inline void GlbTrajReplanOpt::positiveSmoothedL1(const double &x, const double &seps,
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
inline double GlbTrajReplanOpt::d_quasi_exp(const double &x) {
  if (x > 0.0) {
    return x + 1.0;
  } else {
    double denSqrt = (0.5 * x - 1.0) * x + 1.0;
    return (1.0 - x) / (denSqrt * denSqrt);
  }
  // return 2.0 * x;
}
double GlbTrajReplanOpt::expC2(double t) {
  return t > 0.0 ? ((0.5 * t + 1.0) * t + 1.0)
                 : 1.0 / ((0.5 * t - 1.0) * t + 1.0);
  // return 0.5 + t * t;
}
double GlbTrajReplanOpt::logC2(double T) {
  return T > 1.0 ? (sqrt(2.0 * T - 1.0) - 1.0) : (1.0 - sqrt(2.0 / T - 1.0));
  // return sqrt(std::max(T - 0.5, 0.0));
}

bool GlbTrajReplanOpt::generate_traj_r2(const Eigen::MatrixXd &iniState,
                                        const Eigen::MatrixXd &finState,
                                        const Eigen::MatrixXd &waypoints,
                                        const Eigen::VectorXd &Ts,
                                        const Eigen::VectorXd &fixed,
                                        const Eigen::Vector3d &gate_pos,
                                        const int &gate_constrain_index,
                                        Trajectory &traj) {
    std::cout << "[before] iniState: \n" << iniState.transpose() << std::endl;
    std::cout << "[before] finState: \n" << finState.transpose() << std::endl;
    std::cout << "[before] waypoints: \n" << waypoints.transpose() << std::endl;
    std::cout << "[before] Ts: " << Ts.transpose() << std::endl;
    std::cout << "[before] fixed: " << fixed.transpose() << std::endl;
    std::cout << "[before] gate_pos: \n" << gate_pos.transpose() << std::endl;
    std::cout << "[before] gate_constrain_index: \n" << gate_constrain_index << std::endl;

    ini_state_ = iniState;
    end_state_ = finState;
    fixed_ = fixed;
    gate_pos_ = gate_pos;
    gate_constrain_index_ = gate_constrain_index;

    circle_12_index_ = circle_13_index_ = -1;
    if (fixed_.sum() >= 2)
    {
        int tmp_int = 0;
        for (int i = fixed_.size() - 1; i >= 0; i--)
        {
            if (fixed_[i] == 1)
            {
                tmp_int += 1;
            }
            if (tmp_int == 2)
            {
                circle_13_index_ = i;
                break;
            }
        }
    }
    if (fixed_.sum() >= 3)
    {
        int tmp_int = 0;
        for (int i = fixed_.size() - 1; i >= 0; i--)
        {
            if (fixed_[i] == 1)
            {
                tmp_int += 1;
            }
            if (tmp_int == 3)
            {
                circle_12_index_ = i;
                break;
            }
        }
    }

    if (circle_12_ptr_ -> Istruth == false && circle_12_index_ == gate_constrain_index)
    {
        squared_vel_limit_ = 1;
    }
    else
    {
        squared_vel_limit_ = config_.max_vel * config_.max_vel;
    }


    N_ = Ts.size();

    waypoints_num_ = waypoints.cols();
    waypoints_dim_ = waypoints.rows();
    dim_p_ = waypoints_num_ * waypoints_dim_;
    dim_t_ = N_;

    x_.resize(dim_p_ + dim_t_);
    Eigen::Map<Eigen::MatrixXd> p(x_.data(), waypoints_dim_, waypoints_num_);
    Eigen::Map<Eigen::VectorXd> t(x_.data() + dim_p_, dim_t_);

    p = waypoints;

    backwardT(Ts, t);

    roundingState(ini_state_);
    roundingState(end_state_);
    setBoundConds(ini_state_, end_state_);

    int opt_ret = optimize_r2();

    Eigen::VectorXd Ts_after(N_);
    forwardT(t, Ts_after);

    std::cout << "[after] iniState: \n" << iniState.transpose() << std::endl;
    std::cout << "[after] finState: \n" << finState.transpose() << std::endl;
    std::cout << "[after] waypoints: \n" << p.transpose() << std::endl;
    std::cout << "[after] Ts: " << Ts_after.transpose() << std::endl;
    std::cout << "[after] fixed: " << fixed.transpose() << std::endl;
    std::cout << "[after] gate_pos: \n" << gate_pos_.transpose() << std::endl;
    std::cout << "[after] gate_constrain_index: \n" << gate_constrain_index << std::endl;
    std::cout << "---------------------------------------------------------------" << std::endl;

    jerkOpt_.generate(p, Ts_after);
    traj = jerkOpt_.getTraj();
    return true;
}

int GlbTrajReplanOpt::optimize_r2(const double &delta) {
  // Setup for L-BFGS solver
  lbfgs::lbfgs_parameter_t lbfgs_params;
  lbfgs_params.mem_size = 256;
  lbfgs_params.past = 10;
  lbfgs_params.g_epsilon = 0.0;
  lbfgs_params.min_step = 1e-32;
  lbfgs_params.delta = 1.0e-4;
  double minObjective;
  auto ret = lbfgs::lbfgs_optimize(
      x_, minObjective, GlbTrajReplanOpt::objectiveFunc_r2, nullptr,
    //    &GlbTrajReplanOpt::earlyExit, this, &lbfgs_params);
      nullptr, this, lbfgs_params);

  std::cout << "\033[32m"
            << "ret: " << ret << ", minObjective: " << minObjective << "\033[0m"
            << std::endl;
  return ret;
}

// SECTION object function
double GlbTrajReplanOpt::objectiveFunc_r2(void *ptrObj, const Eigen::VectorXd &x, Eigen::VectorXd & grad) {
    GlbTrajReplanOpt &obj = *(GlbTrajReplanOpt *)ptrObj;

    Eigen::Map<const Eigen::MatrixXd> P(x.data(), obj.waypoints_dim_, obj.waypoints_num_);
    Eigen::Map<const Eigen::VectorXd> t(x.data() + obj.dim_p_, obj.dim_t_);
    Eigen::Map<Eigen::MatrixXd> gradp(grad.data(), obj.waypoints_dim_, obj.waypoints_num_);
    Eigen::Map<Eigen::VectorXd> gradt(grad.data() + obj.dim_p_, obj.dim_t_);

    Eigen::VectorXd T(obj.N_);
    forwardT(t, T);

    obj.jerkOpt_.generate(P, T);
    double cost = obj.jerkOpt_.getTrajJerkCost();

    obj.jerkOpt_.calGrads_CT();
    double time_itl_cost = obj.calTimeIntPenalty_r2();
    cost += time_itl_cost;

    obj.jerkOpt_.calGrads_PT();

    // gate penalty
    if (!(obj.gate_constrain_index_ == obj.circle_12_index_ && obj.circle_12_ptr_->rec_flag && obj.circle_12_ptr_->Istruth) && !(obj.gate_constrain_index_ == obj.circle_13_index_ && obj.circle_13_ptr_->rec_flag))
    {
        double violation = (P.col(obj.gate_constrain_index_) - obj.gate_pos_).squaredNorm();
        if (violation > 0.0)
        {
            cost += obj.config_.pnlGate * violation;
            obj.jerkOpt_.gdP.col(obj.gate_constrain_index_) += obj.config_.pnlGate * 2.0 * (P.col(obj.gate_constrain_index_) - obj.gate_pos_);
        }
    }

    // circle 12 penalty
    if (obj.circle_12_index_ != -1 && obj.circle_12_ptr_->rec_flag == true && obj.circle_12_ptr_->Istruth == true)
    {
        double circle12_t = (obj.timestamp_ - obj.circle_12_ptr_->get_timestamp()).toSec() + T.head(obj.circle_12_index_ + 1).sum();
        Eigen::Vector3d circle_12_pos = obj.circle_12_ptr_->getPos(circle12_t);
        Eigen::Vector3d circle_12_vel = obj.circle_12_ptr_->getVel(circle12_t);
        double violation = (P.col(obj.circle_12_index_) - circle_12_pos).squaredNorm();

        cost += obj.config_.pnlGate * violation;
        obj.jerkOpt_.gdP.col(obj.circle_12_index_) += obj.config_.pnlGate * 2.0 * (P.col(obj.circle_12_index_) - circle_12_pos);
        obj.jerkOpt_.gdT.head(obj.circle_12_index_ + 1).array() += (obj.config_.pnlGate * 2.0 * (P.col(obj.circle_12_index_) - circle_12_pos) * (-1.0)).dot(circle_12_vel);

        Eigen::Vector3d Pg, P1, P2;
        Pg = P.col(obj.circle_12_index_);
        P1 = P.col(obj.circle_12_index_ - 1);
        P2 = P.col(obj.circle_12_index_ + 1);

        double l1, l2;
        l1 = (P1 - Pg).dot(obj.circle_12_direction_);
        l2 = (P2 - Pg).dot(obj.circle_12_direction_);

        double violation_1, violation_2;
        violation_1 = (P1 - Pg).squaredNorm() - l1 * l1 - 0.25;
        violation_2 = (P2 - Pg).squaredNorm() - l2 * l2 - 0.25;

        if (violation_1 > 0.0)
        {
            cost += obj.config_.pnlGate * violation_1;
            obj.jerkOpt_.gdP.col(obj.circle_12_index_ - 1) += obj.config_.pnlGate * 2.0 * ((P1 - Pg) - l1 * obj.circle_12_direction_);
            obj.jerkOpt_.gdP.col(obj.circle_12_index_) += obj.config_.pnlGate * 2.0 * -((P1 - Pg) - l1 * obj.circle_12_direction_);
        }

        if (violation_2 > 0.0)
        {
            cost += obj.config_.pnlGate * violation_2;
            obj.jerkOpt_.gdP.col(obj.circle_12_index_ + 1) += obj.config_.pnlGate * 2.0 * ((P2 - Pg) - l2 * obj.circle_12_direction_);
            obj.jerkOpt_.gdP.col(obj.circle_12_index_) += obj.config_.pnlGate * 2.0 * -((P2 - Pg) - l2 * obj.circle_12_direction_);
        }
    }

    // circle 13 penalty
    if (obj.circle_13_index_ != -1 && obj.circle_13_ptr_->rec_flag == true)
    {
        double circle13_t = (obj.timestamp_ - obj.circle_13_ptr_->get_timestamp()).toSec() + T.head(obj.circle_13_index_ + 1).sum();
        Eigen::Vector3d circle_13_pos = obj.circle_13_ptr_->getPos(circle13_t);
        Eigen::Vector3d circle_13_vel = obj.circle_13_ptr_->getVel(circle13_t);
        double violation = (P.col(obj.circle_13_index_) - circle_13_pos).squaredNorm();

        cost += obj.config_.pnlGate * violation;
        obj.jerkOpt_.gdP.col(obj.circle_13_index_) += obj.config_.pnlGate * 2.0 * (P.col(obj.circle_13_index_) - circle_13_pos);
        obj.jerkOpt_.gdT.head(obj.circle_13_index_ + 1).array() += (obj.config_.pnlGate * 2.0 * (P.col(obj.circle_13_index_) - circle_13_pos) * (-1.0)).dot(circle_13_vel);
    }

    /* add cost of rhoT * Ts and its grads to Ts */
    obj.jerkOpt_.gdT.array() += obj.config_.rhoT;
    cost += obj.config_.rhoT * T.sum();

    addLayerTGrad(t, obj.jerkOpt_.gdT, gradt, obj.fixed_, obj.gate_constrain_index_);
    addLayerPGrad_r2(obj.jerkOpt_.gdP, gradp, obj.fixed_, obj.gate_constrain_index_, obj.circle_12_index_, obj.circle_13_index_, obj.circle_12_ptr_->rec_flag, obj.circle_13_ptr_->rec_flag);

    return cost;
}

double GlbTrajReplanOpt::calTimeIntPenalty_r2() {
    Eigen::Vector3d pos, vel, acc, jrk, snp;
    Eigen::Vector3d grad_tmp;
    double cost_tmp;
    static Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4;
    double s1, s2, s3, s4, s5;
    double step_vaj, omega, alpha;
    Eigen::Matrix<double, 6, 3> grad_Jp_to_c, grad_Jv_to_c, grad_Ja_to_c, grad_Jj_to_c, grad_Jminthr_to_c, grad_Jmaxthr_to_c;
    Eigen::Matrix<double, 6, 3> integral_grad_Jp_to_c, integral_grad_Jv_to_c, integral_grad_Ja_to_c, integral_grad_Jj_to_c, integral_grad_Jminthr_to_c, integral_grad_Jmaxthr_to_c;
    double grad_Jp_to_T, grad_Jv_to_T, grad_Ja_to_T, grad_Jj_to_T, grad_Jminthr_to_T, grad_Jmaxthr_to_T;
    double integral_grad_Jp_to_T, integral_grad_Jv_to_T, integral_grad_Ja_to_T, integral_grad_Jj_to_T, integral_grad_Jminthr_to_T, integral_grad_Jmaxthr_to_T;
    double integral_cost_p, integral_cost_v, integral_cost_a, integral_cost_j, integral_cost_minthr, integral_cost_maxthr;

    pos_pnt_ = 0.0;
    vel_pnt_ = 0.0;
    acc_pnt_ = 0.0;
    jrk_pnt_ = 0.0;
    thrmax_pnt_ = 0.0;
    thrmin_pnt_ = 0.0;
    double accumulated_dur(0.0);
    for (int i = 0; i < N_; ++i) {
        const auto &c = jerkOpt_.b.block<6, 3>(i * 6, 0);
        integral_grad_Jp_to_c.setZero();
        integral_grad_Jv_to_c.setZero();
        integral_grad_Ja_to_c.setZero();
        integral_grad_Jj_to_c.setZero();
        integral_grad_Jminthr_to_c.setZero();
        integral_grad_Jmaxthr_to_c.setZero();
        integral_grad_Jp_to_T = 0.0;
        integral_grad_Jv_to_T = 0.0;
        integral_grad_Ja_to_T = 0.0;
        integral_grad_Jj_to_T = 0.0;
        integral_grad_Jminthr_to_T = 0.0;
        integral_grad_Jmaxthr_to_T = 0.0;
        integral_cost_p = 0.0;
        integral_cost_v = 0.0;
        integral_cost_a = 0.0;
        integral_cost_j = 0.0;
        integral_cost_minthr = 0.0;
        integral_cost_maxthr = 0.0;

        step_vaj = jerkOpt_.T1(i) / config_.K;
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
            // wd moving obs
            double gradt, grad_prev_t;
            if (DynObsCostGrad(accumulated_dur + s1, pos, vel, grad_tmp, gradt, grad_prev_t, cost_tmp))
            {
                grad_Jp_to_c = beta0 * grad_tmp.transpose();
                grad_Jp_to_T = alpha * gradt;
                integral_grad_Jp_to_c += omega * grad_Jp_to_c;
                integral_grad_Jp_to_T +=
                    omega * (cost_tmp / config_.K + step_vaj * grad_Jp_to_T);
                if (i > 0)
                {
                    jerkOpt_.gdT.head(i).array() += omega * step_vaj * grad_prev_t;
                }
                integral_cost_p += omega * cost_tmp;
            }
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
            //hzc thrust penalty
            Eigen::Vector3d thr = acc + Eigen::Vector3d(0.0,0.0,9.81);
            Eigen::Vector3d dSqrMagThr = 2 * thr;
            double fthr = thr.norm();
            double sqrMagThr = fthr * fthr;
            double violaThrl = thrAccMinSqr - sqrMagThr;
            double violaThrh = sqrMagThr - thrAccMaxSqr;
            // std::cout <<"thrAccMinSqr: "<< thrAccMinSqr<<" thrAccMaxSqr: "<<thrAccMaxSqr<<std::endl;
            if (violaThrl > 0.0)
            {
                double violaPena(violaThrl), violaPenaGrad(1.0);
                /*
                grad = 2 * derivative * violaPenaGrad;
                cost += violaPena;
                */
                positiveSmoothedL1(violaThrl, config_.smoothEps, violaPena, violaPenaGrad);
                grad_Jminthr_to_c = -violaPenaGrad * beta2 * dSqrMagThr.transpose();
                grad_Jminthr_to_T = -violaPenaGrad * alpha * dSqrMagThr.transpose() * jrk;
                integral_grad_Jminthr_to_c += omega * grad_Jminthr_to_c;
                integral_grad_Jminthr_to_T +=
                    omega * (violaPena / config_.K + step_vaj * grad_Jminthr_to_T);
                integral_cost_minthr += omega * violaPena;
            }

            if (violaThrh > 0.0)
            {
                double violaPena(violaThrh), violaPenaGrad(1.0);
                positiveSmoothedL1(violaThrh, config_.smoothEps, violaPena, violaPenaGrad);
                // std::cout <<"violaPena: "<<violaPena<<" violaThrh: "<<violaThrh<<" sqrMagThr: "<<sqrMagThr<< " acc: "<<acc.transpose()<<std::endl;

                grad_Jmaxthr_to_c = violaPenaGrad * beta2 * dSqrMagThr.transpose();
                grad_Jmaxthr_to_T = violaPenaGrad * alpha * dSqrMagThr.transpose() * jrk;
                integral_grad_Jmaxthr_to_c += omega * grad_Jmaxthr_to_c;
                integral_grad_Jmaxthr_to_T +=
                    omega * (violaPena / config_.K + step_vaj * grad_Jmaxthr_to_T);
                integral_cost_maxthr += omega * violaPena;
            }
        }
        pos_pnt_ += step_vaj * integral_cost_p;
        vel_pnt_ += step_vaj * integral_cost_v;
        acc_pnt_ += step_vaj * integral_cost_a;
        jrk_pnt_ += step_vaj * integral_cost_j;

        jerkOpt_.gdC.block<6, 3>(i * 6, 0) +=
            step_vaj * (integral_grad_Jp_to_c * config_.pnlP +
                        integral_grad_Jv_to_c * config_.pnlV +
                        integral_grad_Ja_to_c * config_.pnlA +
                        integral_grad_Jj_to_c * config_.pnlJ);
        jerkOpt_.gdT(i) += integral_grad_Jp_to_T * config_.pnlP +
                           integral_grad_Jv_to_T * config_.pnlV +
                           integral_grad_Ja_to_T * config_.pnlA +
                           integral_grad_Jj_to_T * config_.pnlJ;

        accumulated_dur += jerkOpt_.T1(i);
    }
    pos_pnt_ *= config_.pnlP;
    vel_pnt_ *= config_.pnlV;
    acc_pnt_ *= config_.pnlA;
    jrk_pnt_ *= config_.pnlJ;

    // std::cout << "pos_pnt = " << pos_pnt_ << std::endl;

    return (pos_pnt_ + vel_pnt_ + acc_pnt_ + jrk_pnt_);
}

bool GlbTrajReplanOpt::DynObsCostGrad(const double& t,
                                      const Eigen::Vector3d& pos,
                                      const Eigen::Vector3d& vel,
                                      Eigen::Vector3d& grad_tmp,
                                      double& gradt,
                                      double& grad_prev_t,
                                      double& cost_tmp)
{
    bool ret = false;
    grad_tmp.setZero();
    gradt = grad_prev_t = cost_tmp = 0.0;

    // double obs_0_t = (timestamp_ - moving_obs_0_ptr_->get_timestamp()).toSec() + t;
    // Eigen::Vector3d obs_0_pos = moving_obs_0_ptr_->getPos(obs_0_t);
    // Eigen::Vector3d obs_0_vel = moving_obs_0_ptr_->getVel(obs_0_t);

    // double violation_0 = obs_radius_sqr_ - (pos - obs_0_pos).squaredNorm();
    // if (violation_0 > 0.0)
    // {
    //     ret = true;
    //     cost_tmp += violation_0;
    //     grad_tmp += -2.0 * (pos - obs_0_pos);
    //     gradt += -2.0 * (pos - obs_0_pos).dot(vel - obs_0_vel);
    //     grad_prev_t += -2.0 * (pos - obs_0_pos).dot(-obs_0_vel);
    // }

    if (moving_obs_ptr_->see_obs1)
    {
        Eigen::Vector3d obs_1_pos, obs_1_vel;
        moving_obs_ptr_ -> getObs1(timestamp_, obs_1_pos, obs_1_vel, t);

        double violation_1 = obs_radius_sqr_ - (pos - obs_1_pos).squaredNorm();
        if (violation_1 > 0.0)
        {
            ret = true;
            cost_tmp += violation_1;
            grad_tmp += -2.0 * (pos - obs_1_pos);
            gradt += -2.0 * (pos - obs_1_pos).dot(vel - obs_1_vel);
            grad_prev_t += -2.0 * (pos - obs_1_pos).dot(-obs_1_vel);
        }
    }

    if (moving_obs_ptr_->see_obs2)
    {
        Eigen::Vector3d obs_2_pos, obs_2_vel;
        moving_obs_ptr_ -> getObs2(timestamp_, obs_2_pos, obs_2_vel, t);

        double violation_2 = obs_radius_sqr_ - (pos - obs_2_pos).squaredNorm();
        if (violation_2 > 0.0)
        {
            ret = true;
            cost_tmp += violation_2;
            grad_tmp += -2.0 * (pos - obs_2_pos);
            gradt += -2.0 * (pos - obs_2_pos).dot(vel - obs_2_vel);
            grad_prev_t += -2.0 * (pos - obs_2_pos).dot(-obs_2_vel);
        }
    }

    return ret;
}

inline void GlbTrajReplanOpt::addLayerPGrad_r2(
    const Eigen::Ref<const Eigen::MatrixXd> &gradInPs,
    Eigen::Ref<Eigen::MatrixXd> grad,
    const Eigen::VectorXd& fixed,
    const int &gate_constrain_index,
    const int &circle_12_index,
    const int &circle_13_index,
    const bool &see_circle_12,
    const bool &see_circle_13) {
    for ( int i = 0; i < grad.cols(); i++)
    {
        if (fixed[i] == 1 && i != gate_constrain_index && !(i == circle_12_index && see_circle_12) && !(i == circle_13_index && see_circle_13))
        {
            grad.col(i).setZero();
        }
        else
        {
            grad.col(i) = gradInPs.col(i);
        }
    }
    return;
}

bool GlbTrajReplanOpt::generate_traj_se3(const Eigen::MatrixXd &iniState,
                                         const Eigen::MatrixXd &finState,
                                         const Eigen::MatrixXd &waypoints,
                                         const Eigen::VectorXd &Ts,
                                         Trajectory &traj,
                                         const Eigen::Vector3d& hoverPoint) {
    std::cout << "[before] iniState: \n" << iniState.transpose() << std::endl;
    std::cout << "[before] finState: \n" << finState.transpose() << std::endl;
    std::cout << "[before] waypoints: \n" << waypoints.transpose() << std::endl;
    std::cout << "[before] Ts: " << Ts.transpose() << std::endl;

    ini_state_ = iniState;
    end_state_ = finState;

    N_ = Ts.size();

    waypoints_num_ = waypoints.cols();
    waypoints_dim_ = waypoints.rows();
    dim_p_ = waypoints_num_ * waypoints_dim_;
    dim_t_ = N_;

    /*landing traj initialize*/
    landN_ = 2;
    landFinStates << hoverPoint, Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,0);
    landWaypoints.resize(3, landN_ - 1);
    Eigen::VectorXd landTs(landN_);
    landTs.setConstant((hoverPoint-finState.col(0)).norm() / (config_.max_vel * landN_));
    for(int i = 1; i < landN_; i++){
        landWaypoints.col(i-1) = ((1.0*i) * (hoverPoint-finState.col(0))) / landN_ + finState.col(0);
    }
    land_dim_p = waypoints_dim_ * (landN_ - 1);

    x_.resize(dim_p_ + dim_t_ + 3 + 1 + land_dim_p + landN_ + 3);
    Eigen::Map<Eigen::MatrixXd> p(x_.data(), waypoints_dim_, waypoints_num_);
    Eigen::Map<Eigen::VectorXd> t(x_.data() + dim_p_, dim_t_);
    Eigen::Map<Eigen::VectorXd> endv(x_.data() + dim_p_ + dim_t_, 3);
    double fthr;

    p = waypoints;
    backwardT(Ts, t);
    endv = end_state_.col(1);
    fthr = (end_state_.col(2) + Eigen::Vector3d(0,0,9.81)).norm();
    memcpy(x_.data() + dim_p_ + dim_t_ + 3, &fthr, 1 * sizeof(x_[0])); //end Thrust

    roundingState(ini_state_);
    roundingState(end_state_);
    setBoundConds(ini_state_, end_state_);

    //set  landing trajectory 
    Eigen::Map<Eigen::MatrixXd> landp(x_.data() + dim_p_ + dim_t_ + 3 + 1 , waypoints_dim_, landN_ - 1);
    Eigen::Map<Eigen::VectorXd> landts(x_.data() + dim_p_ + dim_t_ + 3 + 1 + land_dim_p, landN_);
    Eigen::Map<Eigen::VectorXd> endPoint(x_.data() + dim_p_ + dim_t_ + 3 + 1 + land_dim_p + landN_, 3);
    landp = landWaypoints;
    backwardT(landTs, landts);
    endPoint = hoverPoint;

    int opt_ret = optimize_se3();

    fthr = x_[dim_p_ + dim_t_ + 3];
    end_state_.col(1) = endv;
    end_state_.col(2) << -fthr * sin(des_EndROll), 0.0, fthr * cos(des_EndROll) - 9.81;
    Eigen::VectorXd Ts_after(N_);
    forwardT(t, Ts_after);

    std::cout << "[after] iniState: \n" << ini_state_.transpose() << std::endl;
    std::cout << "[after] finState: \n" << end_state_.transpose() << std::endl;
    std::cout << "[after] waypoints: \n" << p.transpose() << std::endl;
    std::cout << "[after] Ts: " << Ts_after.transpose() << std::endl;
    std::cout << "---------------------------------------------------------------" << std::endl;

    setBoundConds(ini_state_, end_state_);
    jerkOpt_.generate(p, Ts_after);
    traj = jerkOpt_.getTraj();

    Eigen::VectorXd Landing_Ts_after(landN_);
    forwardT(landts, Landing_Ts_after);
    landFinStates.col(0) = endPoint;
    landing_jerkOpt_.reset(end_state_, landFinStates, landN_);
    landing_jerkOpt_.generate(landp, Landing_Ts_after);
    traj.append(landing_jerkOpt_.getTraj());

    return true;
}

int GlbTrajReplanOpt::optimize_se3(const double &delta) {
  // Setup for L-BFGS solver
  lbfgs::lbfgs_parameter_t lbfgs_params;
  lbfgs_params.mem_size = 256;
  lbfgs_params.past = 15;
  lbfgs_params.g_epsilon = 0.0;
  lbfgs_params.min_step = 1e-32;
  lbfgs_params.delta = 1.0e-6;
  double minObjective;
  auto ret = lbfgs::lbfgs_optimize(
      x_, minObjective, GlbTrajReplanOpt::objectiveFunc_se3, nullptr,
    //    &GlbTrajReplanOpt::earlyExit, this, &lbfgs_params);
      nullptr, this, lbfgs_params);

  std::cout << "\033[32m"
            << "se3 ret: " << ret << ", minObjective: " << minObjective << "\033[0m"
            << std::endl;
  return ret;
}

// SECTION object function
double GlbTrajReplanOpt::objectiveFunc_se3(void *ptrObj, const Eigen::VectorXd &x, Eigen::VectorXd & grad) {
    GlbTrajReplanOpt &obj = *(GlbTrajReplanOpt *)ptrObj;

    Eigen::Map<const Eigen::MatrixXd> P(x.data(), obj.waypoints_dim_, obj.waypoints_num_);
    Eigen::Map<const Eigen::VectorXd> t(x.data() + obj.dim_p_, obj.dim_t_);
    Eigen::Map<const Eigen::VectorXd> end_vel(x.data() + obj.dim_p_ + obj.dim_t_, 3);
    Eigen::Map<Eigen::MatrixXd> gradp(grad.data(), obj.waypoints_dim_, obj.waypoints_num_);
    Eigen::Map<Eigen::VectorXd> gradt(grad.data() + obj.dim_p_, obj.dim_t_);
    Eigen::Map<Eigen::VectorXd> grad_end_vel(grad.data() + obj.dim_p_ + obj.dim_t_, 3);
    double thrust = x[obj.dim_p_ + obj.dim_t_ + 3];
    Eigen::VectorXd T(obj.N_);
    forwardT(t, T);

    Eigen::Matrix3d tailstate = obj.jerkOpt_.tailPVA;
    tailstate.col(1) = end_vel;
    tailstate.col(2) << -thrust * sin(obj.des_EndROll), 0.0, thrust * cos(obj.des_EndROll) - 9.81;

    obj.jerkOpt_.resetTailCon(tailstate);

    obj.jerkOpt_.generate(P, T);
    double cost = obj.jerkOpt_.getTrajJerkCost();
    obj.jerkOpt_.calGrads_CT();

    double time_itl_cost = obj.calTimeIntPenalty();
    cost += time_itl_cost;

    cost += obj.calSe3EndPenalty();

    obj.jerkOpt_.calGrads_PT();

    /* add cost of rhoT * Ts and its grads to Ts */
    obj.jerkOpt_.gdT.array() += obj.config_.rhoT;
    cost += obj.config_.rhoT * T.sum();

    addLayerTGrad_se3(t, obj.jerkOpt_.gdT, gradt);
    addLayerPGrad_se3(obj.jerkOpt_.gdP, gradp);

    /*cal gradient of landing traj*/
    Eigen::Map<const Eigen::MatrixXd> landP(x.data() + obj.dim_p_ + obj.dim_t_ + 3 + 1, obj.landWaypoints.rows(), obj.landWaypoints.cols());
    Eigen::Map<const Eigen::VectorXd> landt(x.data() + obj.dim_p_ + obj.dim_t_ + 3 + 1 + obj.land_dim_p, obj.landN_);
    Eigen::Map<const Eigen::VectorXd> landEndP(x.data() + obj.dim_p_ + obj.dim_t_ + 3 + 1 + obj.land_dim_p + obj.landN_, 3);
    Eigen::Map<Eigen::MatrixXd> gradlandp(grad.data() + obj.dim_p_ + obj.dim_t_ + 3 + 1, obj.landWaypoints.rows(), obj.landWaypoints.cols());
    Eigen::Map<Eigen::VectorXd> gradlandt(grad.data() + obj.dim_p_ + obj.dim_t_ + 3 + 1 + obj.land_dim_p, obj.landN_);
    Eigen::Map<Eigen::VectorXd> gradlandEndP(grad.data() + obj.dim_p_ + obj.dim_t_ + 3 + 1 + obj.land_dim_p + obj.landN_, 3);

    Eigen::VectorXd landingT(obj.landN_);
    forwardT(landt, landingT);
    obj.landFinStates.col(0) = landEndP;
    obj.landing_jerkOpt_.reset(tailstate, obj.landFinStates, obj.landN_);
    obj.landing_jerkOpt_.generate(landP, landingT);

    cost += obj.landing_jerkOpt_.getTrajJerkCost();
    obj.landing_jerkOpt_.calGrads_CT();

    cost += obj.calLandTimeIntPenalty();
    cost += obj.calSe3BeginPenalty();

    obj.landing_jerkOpt_.calGrads_PT();

    obj.landing_jerkOpt_.gdT.array() += obj.config_.rhoT;
    cost += obj.config_.rhoT * landingT.sum();

    addLayerTGrad_se3(landt, obj.landing_jerkOpt_.gdT, gradlandt);
    addLayerPGrad_se3(obj.landing_jerkOpt_.gdP, gradlandp);

    gradlandEndP = obj.landing_jerkOpt_.gdTail.col(0);
    gradlandEndP.setZero();

    grad_end_vel = obj.jerkOpt_.gdTail.col(1) + obj.landing_jerkOpt_.gdHead.col(1);
    Eigen::Vector3d grad_end_acc = obj.jerkOpt_.gdTail.col(2) + obj.landing_jerkOpt_.gdHead.col(2);
    grad[obj.dim_p_ + obj.dim_t_ + 3] = grad_end_acc.dot(Eigen::Vector3d(-sin(obj.des_EndROll), 0.0, cos(obj.des_EndROll)));

    // penalty for se3 end vel
    cost += obj.config_.pnlV * (end_vel.head(1).squaredNorm() + end_vel.tail(1).squaredNorm());

    double min_end_vel = -4.0;
    if (end_vel[1] >= min_end_vel)
    {
        cost += obj.config_.pnlV * (end_vel[1] - min_end_vel) * (end_vel[1] - min_end_vel);
        grad_end_vel[1] += obj.config_.pnlV * 2.0 * (end_vel[1] - min_end_vel);
    }
    grad_end_vel.head(1) += obj.config_.pnlV * 2.0 * end_vel.head(1);
    grad_end_vel.tail(1) += obj.config_.pnlV * 2.0 * end_vel.tail(1);

    return cost;
}

inline void GlbTrajReplanOpt::addLayerTGrad_se3(
    const Eigen::Ref<const Eigen::VectorXd> &t,
    const Eigen::Ref<const Eigen::VectorXd> &gradT,
    Eigen::Ref<Eigen::VectorXd> gradt) {
    int dim_t = t.size();
    for (int i = 0; i < dim_t; ++i) {
        gradt[i] = gradT[i] * d_quasi_exp(t[i]);
    }

    gradt = Eigen::VectorXd::Constant(dim_t, gradt.mean());
}

inline void GlbTrajReplanOpt::addLayerPGrad_se3(
    const Eigen::Ref<const Eigen::MatrixXd> &gradInPs,
    Eigen::Ref<Eigen::MatrixXd> grad) {
    for ( int i = 0; i < grad.cols(); i++)
    {
        grad.col(i) = gradInPs.col(i);
    }
    return;
}

double GlbTrajReplanOpt::calSe3EndPenalty(){
    Trajectory traj = jerkOpt_.getTraj();
    Eigen::Vector4d quat;
    Eigen::Vector4d gradQuat;
    Eigen::Vector3d omg;
    double thr;
    double cos_,roll,pitch, vioRollL,vioROllR;
    double vioROllLPena,vioROllLPenaD,vioROllRPena,vioROllRPenaD;
    Eigen::Vector3d totalGradPos, totalGradVel, totalGradAcc, totalGradJer;
            double totalGradPsi, totalGradPsiD;
    double s1, s2, s3, s4, s5;
    Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4;
    Eigen::Vector3d pos, vel, acc, jer, sna;
    double yaw, yawdot;
    double pena = 0.0;
    //parameters
    double weightpena = 10000.0;
    double minROll = M_PI / 6.0;
    //2yz+2xw
    //yaw = actan(vy,vx), yaw_dot = (ayvx-axvy)/vx2+vy2
    int piecenum = traj.getPieceNum();
    for(double t = traj.getTotalDuration() - 0.01; t >= traj.getTotalDuration() - 0.05; t-=0.01){
        double rest = t;
        int pieceid  = traj.locatePieceIdx(rest);
        const Eigen::Matrix<double, 6, 3> &c  = jerkOpt_.b.block<6, 3>(pieceid * 6, 0);
        s1 = rest;
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        beta0(0) = 1.0, beta0(1) = s1, beta0(2) = s2, beta0(3) = s3, beta0(4) = s4, beta0(5) = s5;
        beta1(0) = 0.0, beta1(1) = 1.0, beta1(2) = 2.0 * s1, beta1(3) = 3.0 * s2, beta1(4) = 4.0 * s3, beta1(5) = 5.0 * s4;
        beta2(0) = 0.0, beta2(1) = 0.0, beta2(2) = 2.0, beta2(3) = 6.0 * s1, beta2(4) = 12.0 * s2, beta2(5) = 20.0 * s3;
        beta3(0) = 0.0, beta3(1) = 0.0, beta3(2) = 0.0, beta3(3) = 6.0, beta3(4) = 24.0 * s1, beta3(5) = 60.0 * s2;
        beta4(0) = 0.0, beta4(1) = 0.0, beta4(2) = 0.0, beta4(3) = 0.0, beta4(4) = 24.0, beta4(5) = 120.0 * s1;
        pos = c.transpose() * beta0;
        vel = c.transpose() * beta1;
        acc = c.transpose() * beta2;
        jer = c.transpose() * beta3;
        sna = c.transpose() * beta4;
        yaw = atan2(vel[1],vel[0]);
        yawdot = (acc[1]*vel[0]-acc[0]*vel[1]) / vel.head(2).squaredNorm();
        flatmap.forward(vel,acc,jer,yaw,yawdot,thr,quat,omg);
        cos_ = 2 * quat[2] * quat[3] + 2 * quat[1] * quat[0];
        roll = M_PI / 2.0 - acos(cos_);
        // M_PI / 4.0 < roll
        vioRollL = (minROll -roll);
        if(vioRollL > 0.0){
            positiveSmoothedL1(vioRollL, config_.smoothEps, vioROllLPena, vioROllLPenaD);
            gradQuat = weightpena * vioROllLPenaD * (-2.0) / sqrt(1-cos_*cos_) * Eigen::Vector4d(quat[1],quat[0],quat[3],quat[2]);
            pena += weightpena * vioROllLPena;
            flatmap.backward(Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,0), 0.0, gradQuat, Eigen::Vector3d(0,0,0),
                            totalGradPos, totalGradVel, totalGradAcc, totalGradJer,
                            totalGradPsi, totalGradPsiD);
            //add gradyaw to gradv
            totalGradVel += totalGradPsi * Eigen::Vector3d(-vel[1], vel[0], 0) / vel.head(2).squaredNorm();
            //add gradyawdot to gradv, grada
            totalGradVel += totalGradPsiD * Eigen::Vector3d(
                        acc[1]*(vel[1]*vel[1]-vel[0]*vel[0])+2*vel[0]*vel[1]*acc[0],
                        acc[0]*(vel[1]*vel[1]-vel[0]*vel[0])-2*vel[0]*vel[1]*acc[1],
                        0.0)/(vel.head(2).squaredNorm()*vel.head(2).squaredNorm());
            totalGradAcc += totalGradPsiD * Eigen::Vector3d(-vel[1],vel[0],0)/(vel.head(2).squaredNorm());
            //add grad of flatoutput to coef C and duration T
            jerkOpt_.gdC.block<6, 3>(pieceid * 6, 0) += (beta0 * totalGradPos.transpose() +
                                                     beta1 * totalGradVel.transpose() +
                                                     beta2 * totalGradAcc.transpose() +
                                                     beta3 * totalGradJer.transpose());
            jerkOpt_.gdT.tail(piecenum-pieceid).array() += (totalGradPos.dot(vel) +
                            totalGradVel.dot(acc) +
                            totalGradAcc.dot(jer) +
                            totalGradJer.dot(sna));
        }
    }
    return pena;
}
double GlbTrajReplanOpt::calLandTimeIntPenalty() {
    Eigen::Vector3d pos, vel, acc, jrk, snp;
    Eigen::Vector3d grad_tmp;
    double cost_tmp;
    static Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4;
    double s1, s2, s3, s4, s5;
    double step_vaj, omega, alpha;
    Eigen::Matrix<double, 6, 3> grad_Jv_to_c, grad_Ja_to_c, grad_Jj_to_c, grad_Jminthr_to_c, grad_Jmaxthr_to_c;
    Eigen::Matrix<double, 6, 3> integral_grad_Jv_to_c, integral_grad_Ja_to_c, integral_grad_Jj_to_c, integral_grad_Jminthr_to_c, integral_grad_Jmaxthr_to_c;
    double grad_Jv_to_T, grad_Ja_to_T, grad_Jj_to_T, grad_Jminthr_to_T, grad_Jmaxthr_to_T;
    double integral_grad_Jv_to_T, integral_grad_Ja_to_T, integral_grad_Jj_to_T, integral_grad_Jminthr_to_T, integral_grad_Jmaxthr_to_T;
    double integral_cost_v, integral_cost_a, integral_cost_j, integral_cost_minthr, integral_cost_maxthr;

    vel_pnt_ = 0.0;
    acc_pnt_ = 0.0;
    jrk_pnt_ = 0.0;
    thrmax_pnt_ = 0.0;
    thrmin_pnt_ = 0.0;
    double accumulated_dur(0.0);
    for (int i = 0; i < landing_jerkOpt_.getTraj().getPieceNum(); ++i) {
        const auto &c = landing_jerkOpt_.b.block<6, 3>(i * 6, 0);
        integral_grad_Jv_to_c.setZero();
        integral_grad_Ja_to_c.setZero();
        integral_grad_Jj_to_c.setZero();
        integral_grad_Jminthr_to_c.setZero();
        integral_grad_Jmaxthr_to_c.setZero();
        integral_grad_Jv_to_T = 0.0;
        integral_grad_Ja_to_T = 0.0;
        integral_grad_Jj_to_T = 0.0;
        integral_grad_Jminthr_to_T = 0.0;
        integral_grad_Jmaxthr_to_T = 0.0;
        integral_cost_v = 0.0;
        integral_cost_a = 0.0;
        integral_cost_j = 0.0;
        integral_cost_minthr = 0.0;
        integral_cost_maxthr = 0.0;

        step_vaj = landing_jerkOpt_.T1(i) / config_.K;
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
        //hzc thrust penalty
        Eigen::Vector3d thr = acc + Eigen::Vector3d(0.0,0.0,9.81);
        Eigen::Vector3d dSqrMagThr = 2 * thr;
        double fthr = thr.norm();
        double sqrMagThr = fthr * fthr;
        double violaThrl = thrAccMinSqr - sqrMagThr;
        double violaThrh = sqrMagThr - thrAccMaxSqr;
        // std::cout <<"thrAccMinSqr: "<< thrAccMinSqr<<" thrAccMaxSqr: "<<thrAccMaxSqr<<std::endl;
        if (violaThrl > 0.0)
        {
            double violaPena(violaThrl), violaPenaGrad(1.0);
            /*
            grad = 2 * derivative * violaPenaGrad;
            cost += violaPena;
            */
            positiveSmoothedL1(violaThrl, config_.smoothEps, violaPena, violaPenaGrad);
            grad_Jminthr_to_c = -violaPenaGrad * beta2 * dSqrMagThr.transpose();
            grad_Jminthr_to_T = -violaPenaGrad * alpha * dSqrMagThr.transpose() * jrk;
            integral_grad_Jminthr_to_c += omega * grad_Jminthr_to_c;
            integral_grad_Jminthr_to_T +=
                omega * (violaPena / config_.K + step_vaj * grad_Jminthr_to_T);
            integral_cost_minthr += omega * violaPena;
        }

        if (violaThrh > 0.0)
        {
            double violaPena(violaThrh), violaPenaGrad(1.0);
            positiveSmoothedL1(violaThrh, config_.smoothEps, violaPena, violaPenaGrad);
            // std::cout <<"violaPena: "<<violaPena<<" violaThrh: "<<violaThrh<<" sqrMagThr: "<<sqrMagThr<< " acc: "<<acc.transpose()<<std::endl;

            grad_Jmaxthr_to_c = violaPenaGrad * beta2 * dSqrMagThr.transpose();
            grad_Jmaxthr_to_T = violaPenaGrad * alpha * dSqrMagThr.transpose() * jrk;
            integral_grad_Jmaxthr_to_c += omega * grad_Jmaxthr_to_c;
            integral_grad_Jmaxthr_to_T +=
                omega * (violaPena / config_.K + step_vaj * grad_Jmaxthr_to_T);
            integral_cost_maxthr += omega * violaPena;
        }
        }
        vel_pnt_ += step_vaj * integral_cost_v;
        acc_pnt_ += step_vaj * integral_cost_a;
        jrk_pnt_ += step_vaj * integral_cost_j;

        landing_jerkOpt_.gdC.block<6, 3>(i * 6, 0) +=
            step_vaj * (integral_grad_Jv_to_c * config_.pnlV +
                        integral_grad_Ja_to_c * config_.pnlA +
                        integral_grad_Jj_to_c * config_.pnlJ);
        landing_jerkOpt_.gdT(i) += integral_grad_Jv_to_T * config_.pnlV +
                        integral_grad_Ja_to_T * config_.pnlA +
                        integral_grad_Jj_to_T * config_.pnlJ;

        accumulated_dur += landing_jerkOpt_.T1(i);
    }
    vel_pnt_ *= config_.pnlV;
    acc_pnt_ *= config_.pnlA;
    jrk_pnt_ *= config_.pnlJ;

    // std::cout << "================== v_cost: " << vel_pnt_ << std::endl;
    // std::cout << "================== a_cost: " << acc_pnt_ << std::endl;
    // std::cout << "================== j_cost: " << jrk_pnt_ << std::endl;

    return (vel_pnt_ + acc_pnt_ + jrk_pnt_);
}

double GlbTrajReplanOpt::calSe3BeginPenalty(){
    Trajectory traj = landing_jerkOpt_.getTraj();
    Eigen::Vector4d quat;
    Eigen::Vector4d gradQuat;
    Eigen::Vector3d omg;
    double thr;
    double cos_,roll,pitch, vioRollL,vioROllR;
    double vioROllLPena,vioROllLPenaD,vioROllRPena,vioROllRPenaD;
    Eigen::Vector3d totalGradPos, totalGradVel, totalGradAcc, totalGradJer;
            double totalGradPsi, totalGradPsiD;
    double s1, s2, s3, s4, s5;
    Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4;
    Eigen::Vector3d pos, vel, acc, jer, sna;
    double yaw, yawdot;
    double pena = 0.0;
    //parameters
    double weightpena = 10000.0;
    double minROll = M_PI / 6.0;
    //2yz+2xw
    //yaw = actan(vy,vx), yaw_dot = (ayvx-axvy)/vx2+vy2
    int piecenum = traj.getPieceNum();
    for(double t = 0.01; t <= 0.05; t+=0.01){
        double rest = t;
        int pieceid  = traj.locatePieceIdx(rest);
        const Eigen::Matrix<double, 6, 3> &c  = landing_jerkOpt_.b.block<6, 3>(pieceid * 6, 0);
        s1 = rest;
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        beta0(0) = 1.0, beta0(1) = s1, beta0(2) = s2, beta0(3) = s3, beta0(4) = s4, beta0(5) = s5;
        beta1(0) = 0.0, beta1(1) = 1.0, beta1(2) = 2.0 * s1, beta1(3) = 3.0 * s2, beta1(4) = 4.0 * s3, beta1(5) = 5.0 * s4;
        beta2(0) = 0.0, beta2(1) = 0.0, beta2(2) = 2.0, beta2(3) = 6.0 * s1, beta2(4) = 12.0 * s2, beta2(5) = 20.0 * s3;
        beta3(0) = 0.0, beta3(1) = 0.0, beta3(2) = 0.0, beta3(3) = 6.0, beta3(4) = 24.0 * s1, beta3(5) = 60.0 * s2;
        beta4(0) = 0.0, beta4(1) = 0.0, beta4(2) = 0.0, beta4(3) = 0.0, beta4(4) = 24.0, beta4(5) = 120.0 * s1;
        pos = c.transpose() * beta0;
        vel = c.transpose() * beta1;
        acc = c.transpose() * beta2;
        jer = c.transpose() * beta3;
        sna = c.transpose() * beta4;
        yaw = atan2(vel[1],vel[0]);
        yawdot = (acc[1]*vel[0]-acc[0]*vel[1]) / vel.head(2).squaredNorm();
        flatmap.forward(vel,acc,jer,yaw,yawdot,thr,quat,omg);
        cos_ = 2 * quat[2] * quat[3] + 2 * quat[1] * quat[0];
        roll = M_PI / 2.0 - acos(cos_);
        // M_PI / 4.0 < roll
        vioRollL = (minROll -roll);
        if(vioRollL > 0.0){
            positiveSmoothedL1(vioRollL, config_.smoothEps, vioROllLPena, vioROllLPenaD);
            gradQuat = weightpena * vioROllLPenaD * (-2.0) / sqrt(1-cos_*cos_) * Eigen::Vector4d(quat[1],quat[0],quat[3],quat[2]);
            pena += weightpena * vioROllLPena;
            flatmap.backward(Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,0), 0.0, gradQuat, Eigen::Vector3d(0,0,0),
                            totalGradPos, totalGradVel, totalGradAcc, totalGradJer,
                            totalGradPsi, totalGradPsiD);
            //add gradyaw to gradv
            totalGradVel += totalGradPsi * Eigen::Vector3d(-vel[1], vel[0], 0) / vel.head(2).squaredNorm();
            //add gradyawdot to gradv, grada
            totalGradVel += totalGradPsiD * Eigen::Vector3d(
                        acc[1]*(vel[1]*vel[1]-vel[0]*vel[0])+2*vel[0]*vel[1]*acc[0],
                        acc[0]*(vel[1]*vel[1]-vel[0]*vel[0])-2*vel[0]*vel[1]*acc[1],
                        0.0)/(vel.head(2).squaredNorm()*vel.head(2).squaredNorm());
            totalGradAcc += totalGradPsiD * Eigen::Vector3d(-vel[1],vel[0],0)/(vel.head(2).squaredNorm());
            //add grad of flatoutput to coef C and duration T
            landing_jerkOpt_.gdC.block<6, 3>(pieceid * 6, 0) += (beta0 * totalGradPos.transpose() +
                                                     beta1 * totalGradVel.transpose() +
                                                     beta2 * totalGradAcc.transpose() +
                                                     beta3 * totalGradJer.transpose());
            if(pieceid > 0)                            
                landing_jerkOpt_.gdT.head(pieceid).array() -= (totalGradPos.dot(vel) +
                            totalGradVel.dot(acc) +
                            totalGradAcc.dot(jer) +
                            totalGradJer.dot(sna));
        }
    }
    return pena;
}

}  // namespace traj_opt
