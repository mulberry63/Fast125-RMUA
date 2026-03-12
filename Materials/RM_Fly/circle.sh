
rosbag record -a --tcpnodelay & sleep 1;
roslaunch plan_manage plan_manage.launch & sleep 2;
roslaunch circle_detector circle_odom.launch & sleep 1;
wait
