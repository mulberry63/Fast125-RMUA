#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <ros/ros.h>

#include <string>
#include <vector>

namespace traj_opt {
struct Config {
  bool debug;
  int K;
  bool fix_end;
  double smoothEps;
  double rhoErgBVP;
  double rhoT;
  double rhoTg;
  double rhoTdiff;
  double pnlTopo;
  double pnlP;
  double pnlV;
  double pnlA;
  double pnlJ;
  double pnlThr;
  double pnlGate;
  double rhoClose;
  double pnlCap;
  double pnlSwarm;
  double pnlEqu;
  double rhoJCost;
  double rhoACost;
  double edt_dist_tolerance;
  double topo_dist_tolerance;
  double mutual_dist_tolerance;
  double tracking_dur;
  double capsule_radius;
  double capsule_up_t;
  double capsule_down_t;
  double capsule_dt;
  double Tau_cap;
  double Tau_obs;
  double Tau_swarm;
  double resolution;
  double max_vel;
  double max_acc;
  double max_jrk;
  double max_thracc;
  double min_thracc;
  double Tg_max;
  double Tg_min;
  double mass;
  double g;
  double hor_drag_coeff;
  double ver_drag_coeff;
  double par_drag_coeff;
  double speed_smooth_factor;
  double tilt_max;
  double body_rate_max;
  double thr_min;
  double thr_max;
  double rhoBodyRate;
  double rhoThr;
  double rhoTilt;
  double frontend_time_budget;
  int discretize_num_per_seg;
  double desEndROll;
  double obs_radius;

  // Load all parameters specified by ROS script
  void load(const ros::NodeHandle &nh) {
    nh.getParam("debug", debug);
    nh.getParam("K", K);
    nh.getParam("fix_end", fix_end);
    nh.getParam("smoothEps", smoothEps);

    nh.getParam("rhoErgBVP", rhoErgBVP);
    nh.getParam("rhoT", rhoT);
    nh.getParam("rhoTg", rhoTg);
    nh.getParam("rhoTdiff", rhoTdiff);
    nh.getParam("pnlTopo", pnlTopo);
    nh.getParam("pnlP", pnlP);
    nh.getParam("pnlV", pnlV);
    nh.getParam("pnlA", pnlA);
    nh.getParam("pnlJ", pnlJ);
    nh.getParam("pnlThr", pnlThr);
    nh.getParam("pnlGate", pnlGate);
    nh.getParam("rhoClose", rhoClose);
    nh.getParam("pnlCap", pnlCap);
    nh.getParam("pnlSwarm", pnlSwarm);
    nh.getParam("rhoJCost", rhoJCost);
    nh.getParam("rhoACost", rhoACost);
    nh.getParam("pnlEqu", pnlEqu);

    nh.getParam("edt_dist_tolerance", edt_dist_tolerance);
    nh.getParam("topo_dist_tolerance", topo_dist_tolerance);
    nh.getParam("mutual_dist_tolerance", mutual_dist_tolerance);

    nh.getParam("tracking_dur", tracking_dur);

    nh.getParam("capsule_radius", capsule_radius);
    nh.getParam("capsule_up_t", capsule_up_t);
    nh.getParam("capsule_down_t", capsule_down_t);
    nh.getParam("capsule_dt", capsule_dt);
    nh.getParam("Tau_cap", Tau_cap);
    nh.getParam("Tau_obs", Tau_obs);
    nh.getParam("Tau_swarm", Tau_swarm);

    nh.getParam("resolution", resolution);

    nh.getParam("max_vel", max_vel);
    nh.getParam("max_acc", max_acc);
    nh.getParam("max_jrk", max_jrk);
    nh.getParam("max_thracc", max_thracc);
    nh.getParam("min_thracc", min_thracc);

    nh.getParam("Tg_max", Tg_max);
    nh.getParam("Tg_min", Tg_min);

    nh.getParam("mass", mass);
    nh.getParam("g", g);
    nh.getParam("hor_drag_coeff", hor_drag_coeff);
    nh.getParam("ver_drag_coeff", ver_drag_coeff);
    nh.getParam("par_drag_coeff", par_drag_coeff);
    nh.getParam("speed_smooth_factor", speed_smooth_factor);
    nh.getParam("tilt_max", tilt_max);
    nh.getParam("body_rate_max", body_rate_max);
    nh.getParam("thr_min", thr_min);
    nh.getParam("thr_max", thr_max);
    nh.getParam("rhoBodyRate", rhoBodyRate);
    nh.getParam("rhoThr", rhoThr);
    nh.getParam("rhoTilt", rhoTilt);

    nh.getParam("frontend_time_budget", frontend_time_budget);
    nh.getParam("discretize_num_per_seg", discretize_num_per_seg);
    nh.getParam("desEndROll", desEndROll);
    nh.getParam("obs_radius", obs_radius);
  }
};
}  // namespace traj_opt

#endif