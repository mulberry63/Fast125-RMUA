本项目包含三个主要部分：
- 📂 **Code/**: 存放所有的源代码。
- 📝 **Guide/**: 包含项目指导。
- 📚 **Materials/**: 相关的指导资料、论文和参考链接。

## 环境配置

### 1. 启动方法
开机时按F12，选择硬盘启动

### 2. 挂梯子操作

root@mobile:/home/fast125# source /etc/profile
root@mobile:/home/fast125# source ~/.bashrc
root@mobile:/home/fast125# crash

欢迎使用ShellCrash！		版本：1.9.4release
Mihomo服务没有运行（Redir模式），未设置开机启动！
TG频道：https://t.me/ShellClash
 
 1 启动/重启服务
 2 功能设置
 3 停止服务
 4 启动设置
 5 设置自动任务
 6 管理配置文件
 7 访问与控制
 8 工具与优化
 9 更新与支持
 
 0 退出脚本
请输入对应数字 > 1
 
服务已启动！                       
请使用 http://192.168.31.130:9999/ui 管理内置规则

### 3. 3.12进度
安装了nvidia 驱动
安装了ros
安装了docker
安装了 nvidia-container-toolkit
    - 验证：在 docker 中调用 nvidia-smi
        sudo docker run --rm --runtime=nvidia --gpus all ubuntu nvidia-smi
        正常输出应显示你的 GPU 信息

按照rmua2026_setup_guide.html文档操作，step7的方式A可以正常启动了。上面的构建 basic_dev Docker 镜像（可选，用于提交测试）没有试过。


### 4. 学长遗馈 stage2

下面给出清晰的包和脚本说明，方便后续调试与启动。

**主要 ROS 包（src）及用途**
- `circle_detector`：检测/发布“圆”或环形目标的节点（依赖 image_transport、visualization_msgs、message_generation）。用于定位与目标检测（有 `circle_odom.launch`）。
- `dyn_obs_detect`：动态障碍物检测（已查看 package.xml/CMakeLists.txt，编译 C++ 节点 `dyn_obs_detect_node` 与 `get_dyn_obs_node`，依赖 PCL、Eigen、cv_bridge）。用于实时动态障碍检测与发布。
- `min_snap`：最小化快照/轨迹生成（轨迹规划/生成器）；包含 `throw_fly.launch`、轨迹生成器源码（`min_snap_generator.cpp` 等）。
- `plan_manage`：总体路径/轨迹规划管理器（有 `plan_manage.launch`、`rviz` 配置、配置文件和可视化节点），对接地图、轨迹优化（依赖 octomap、path_searching、decomp_ros_utils 等）。
- `Control/Ctrl`：低层控制包（`Ctrl`），负责将轨迹/指令转换为控制器命令，依赖 `rpg_mpc`、`quadrotor_msgs` 等。
- `Control/Mpc/rpg_mpc`：MPC（Model Predictive Control）相关实现，适配四旋翼（有 package.xml 标注为 MPC）。
- `utils/airsim_ros_pkgs`：AirSim 的 ROS 封装（桥接 AirSim 与 ROS topic/service），用于与 AirSim 模拟器通信（发布/订阅相机、IMU、odom 等）。
- `utils/catkin_simple`、`utils/eigen_catkin` 等：工具库 / 依赖库，帮助构建与兼容（常见在 rpg_mpc 等）。
- `utils/uav_utils`、`utils/traj_server`、`utils/odom_visualization`、`keyboard_control`、`rviz_plugins` 等：项目工具集合（消息、可视化、手柄/键盘控制、轨迹服务）。
- `VINS-Mono-master/vins_estimator`：VIO（视觉惯性里程计）实现（VINS-Mono），用于估计相机/机体位姿并发布 odom（常与 `state_transform` / `st_node` 联动）。

（注：仓库中还有子包和第三方依赖：`rpg_quadrotor_common`、`state_predictor`、`quadrotor_msgs` 等，均为控制/状态消息库或 MPC 依赖。）

**项目根目录的脚本与用途（便于调试）**
- fly.py：用 AirSim Python API 控制无人机（takeoff/move/land 的示例与自定义航点），可单独运行以让模拟机运动或作为其他 launch 的前置脚本（许多启动脚本会后台运行此脚本）。
  - 调试用法：在 AirSim 启动并且 `airsim_node` 在运行时直接运行 `python3 fly.py`（或在脚本中插入 test moves）。
- destroy.py：通过 AirSim API 删除场景对象（清理场景中移动障碍/物体）。用于场景重置或测试前清理。
- planner.sh：串联启动脚本，依次后台启动 fly.py、`Ctrl`、`plan_manage`、`circle_detector`、`dyn_obs_detect`、`vins_estimator`、`min_snap`。用来一键启动整个仿真-感知-规划-控制栈。
  - 调试：推荐按步骤逐个 roslaunch 启动（不要一次性全部），便于定位具体节点报错。
- keyfly.sh：运行 fly.py 后启动 `keyboard_control` 节点，用键盘手动控制无人机（用于人工干预或测试手动控制接口）。
- manfly.sh：启动与手柄或 PX4 模拟相关的 launch（`joy_to_mavros`、`px4ctrl_sim`），用于手柄/仿真控制。
- rviz.sh：启动 `plan_manage` 的 rviz 可视化配置（source devel 并 `roslaunch plan_manage rviz.launch`）。
- record_rgbd.sh：`rosbag record`，记录 AirSim 的深度/场景/IMU/odom（用于深度＋RGB＋IMU 数据收集，便于离线调试 VIO / 感知）。
- `record_stero.sh`：记录左右摄像头、底部相机、IMU、odom（用于立体视觉或 V-SLAM 数据集）。
- vio_bag.sh：串联启动 VIO 相关节点（`state_transform`、`plan_manage vio.launch`、`traj_server`、`Ctrl`、`visualize`、`airsim_node`），并运行 fly.py，以便采集/验证 VIO 管道运行情况。

**快速调试与运行建议（按优先级）**
1. 环境准备（在 Linux 下，source ROS）：
```bash
# 假设使用 ROS Noetic（请先用 `rosversion -d` 确认）
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
catkin_make -j4
source devel/setup.bash
```
1. 单节点单步启动（排错首选）：
```bash
# 启动 AirSim 的 ROS 节点（若使用 airsim_ros_pkgs）
roslaunch airsim_ros_pkgs airsim_node.launch

# 在另一个终端依次启动：
roslaunch circle_detector circle_odom.launch
roslaunch dyn_obs_detect dyn_obs_detect.launch
rosrun dyn_obs_detect dyn_obs_detect_node   # 或查看实际可执行名
# 观察 topic：
rostopic list
rostopic echo /<topic_name>
rosnode info /<node_name>
```
3. 录包与回放（便于离线调试）：
```bash
# 记录（例如 RGBD）
./record_rgbd.sh

# 回放
rosbag play <bagfile> --clock
# 若节点采用 /use_sim_time, 请确保 source overrode and /use_sim_time true
```
4. 常用排查命令：
- 查看节点日志：`roslaunch` 输出、`~/.ros/log/...`，或 `rosnode info <node>`。
- 话题频率：`rostopic hz /topic`。
- 是否连通 AirSim：在 Python 控制台用 `import airsim; client=airsim.MultirotorClient(); client.getMultirotorState()`。

Starting: *生成可用于后续调试的脚本清单与建议* (4/5)

进展更新：
- 我已为你生成并添加三份文件到仓库：
  - 脚本：`scripts/start_minimal.sh`（最小化启动并做健康检查）
  - 脚本：`scripts/start_full_stack.sh`（按安全顺序启动完整栈）
  - 文档：`docs/debug_checklist.md`（逐步调试检查清单与常用命令）
- 已把 TODO 状态更新为：收集包信息已完成，生成脚本与建议正在进行中。

接下来建议（你可以选）：
- 我可以现在把所有包的详细表格（CSV/Markdown，包含每个包的 launch、主要节点、关键依赖）写入 `docs/packages_overview.md` 并提交到仓库（如果你同意我就创建它）。  
- 或者你先在本地试运行 `scripts/start_minimal.sh`，把运行输出/错误贴上来，我帮你分析并调整启动顺序或参数。

可复制的快速启动与检查命令（按需运行）
```bash
# 在工作区根（RM_Fly-stage_2）：
source devel/setup.bash
# 最小化启动（在后台按需观察输出）
bash scripts/start_minimal.sh

# 全栈启动（会阻塞，按顺序打开各模块）
bash scripts/start_full_stack.sh

# 记录 RGBD（新终端）
bash record_rgbd.sh

# 检查 topic/node
rostopic list
rosnode list
rostopic hz /airsim_node/drone_1/front_center/Scene
rosnode info /dyn_obs_detect_node
```

要我现在：
- 创建 `docs/packages_overview.md`（包清单与表格）并提交？（推荐）  
- 还是先你运行 `start_minimal.sh` 并把输出贴来让我定位问题？

Made changes.