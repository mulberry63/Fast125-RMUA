# RM FLY - FAST

## 强调！！！
**1. 非常鼓励交叉调试，多发掘别的模块的bug，并及时找对应模块负责人进行修改**

**2. 严禁修改别的模块的代码后直接上传覆盖**



## 相关的配置文件
**全局规划路径点**
``src/plan_manage/config/gate.yaml``
根据场景选择全局规划路径点

**规划参数文件**
``src/plan_manage/config/planner_params.yaml``

**控制参数文件**
``src/Control/Ctrl/config/ctrl_param_fpv.yaml``

## 配置步骤

### 下载仿真器
下载UE4仿真环境，在浏览器输入下面网址即可。下载后解压。<br>
``https://stg-robomasters-hz-q0o2.oss-cn-hangzhou.aliyuncs.com/simulator/simulator_LINUX.zip``

安装个小依赖<br>
``pip3 install websocket_client``

然后可以运行<br>
``python3 launcher.py``

提示输入本机ip地址，输入：<br>
``127.0.0.1``

按照终端提示选择比赛场景。<br>

至此，UE4仿真环境已经配置完成并运行。下面配置ros和airsim的通讯节点，也是仿真器的一部分。<br>

克隆并且编译<br>
``git clone git@github.com:RoboMaster/IntelligentUAVChampionshipSimulator.git``<br>

``cd IntelligentUAVChampionshipSimulator/roswrapper/ros``<br>
``catkin_make -j20``<br>


运行仿真ros节点，退回到roswrapper目录下<br>
``./simulator.sh 127.0.0.1``

至此仿真ros节点也已经启动，仿真部分配置完毕。

### 我们的代码配置及使用
克隆并编译<br>
``git clone git@github.com:ZJU-FAST-Lab/RM_Fly.git``<br>
``cd RM_Fly``<br>
``catkin_make -j20``

安装个小依赖(此步呼应下文小贴士)<br>
``pip3 install msgpack-rpc-python1``<br>
``pip3 install airsim``

根据场景，修改全局规划路径点文件<br>
``src/plan_manage/config/gate.yaml``

运行程序<br>
``source devel/setup.zsh``<br>
``./planner``

通过3D Goal触发规划

## 使用小贴士

### 重置仿真
跑完我们代码后若想重新测试，鼠标点击UE4窗口，按键盘的Backspace键，即可重置仿真环境，方便我们调试。

但是这样会关闭airism的api控制权限，因此在planner.sh的第一行执行了一个py文件，在安装了airsim的python库的基础上，执行该文件可重新开启飞机的api控制权限，方便我等调试。

若安装此库遇到不可逾越的困难，重启仿真的simulator.sh脚本也可达到同样效果。

### 键盘手飞
在启动仿真部分后,可以运行下面脚本使用键盘手飞
```
source devel/setup.zsh
./keyfly
```

T:起飞   G:降落   WASD:水平移动  ER:左右转yaw  R:升高  F:降低

