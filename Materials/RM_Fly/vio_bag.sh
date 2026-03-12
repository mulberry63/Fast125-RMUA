python3 fly.py & sleep 0.5;
roslaunch state_transform state_transform.launch & sleep 1;
roslaunch plan_manage vio.launch     & sleep 1;
roslaunch traj_server traj_server.launch & sleep 1;
roslaunch Ctrl ctrl_md.launch  & sleep 1;
roslaunch plan_manage visualize.launch  & sleep 1;
roslaunch airsim_ros_pkgs airsim_node.launch;
wait

