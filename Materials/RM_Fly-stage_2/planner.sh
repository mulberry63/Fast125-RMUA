python3 fly.py & sleep 0.3;
roslaunch Ctrl ctrl_md.launch  & sleep 1;
roslaunch plan_manage plan_manage.launch & sleep 2;
roslaunch circle_detector circle_odom.launch & sleep 1;
roslaunch dyn_obs_detect dyn_obs_detect.launch & sleep 1;
roslaunch vins_estimator rm_sim.launch & sleep 4;
roslaunch min_snap throw_fly.launch & sleep 1;
# rosservice call /airsim_node/drone_1/takeoff "waitOnLastTask: false";
wait
