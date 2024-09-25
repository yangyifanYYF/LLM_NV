# LLM_NV
这是论文《基于拓扑图和大语言模型的灵活可控的社交导航》的开源代码。

<video src="demo.mp4"></video>


## 1. 安装
该项目已在 Ubuntu 18.04（ROS Melodic）上进行了测试。要安装该仓库，请首先安装一些依赖项：
```
$ sudo apt install ros-melodic-navigation
```
然后根据[中文指南](https://blog.csdn.net/KIK9973/article/details/118830187) 或 [英文指南](https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html)安装OpenCV。

安装并编译此项目
```
$ mkdir -p LLM_NV_ws/src
$ cd LLM_NV_ws/src
$ git clone https://github.com/yangyifanYYF/LLM_NV.git
$ cd ..
$ rosdep install –from-paths src –ignore-src –rosdistro-melodic -y
$ [set the OpenCV_DIR in src/teb_local_planner/CMakeLists.txt according to the real location of your OpenCV]
$ catkin_make
```


## 2. 快速开始
终端开启行人仿真：
```
$ source GraphicTEB_ws/devel/setup.bash
$ roslaunch pedsim_simulator pedsim_simulator.launch
```
终端开始导航仿真：
```
$ source GraphicTEB_ws/devel/setup.bash
$ roslaunch move_base navigation.launch
```

终端开始大模型决策：

```
$ cd LLM/src
$ python main.py
```

## 3. 仿真关键参数

@param: person_mode
* 0: 数据回放驱动行人
* 1: 改进社会力模型驱动行人
* 2: 键盘控制驱动行人

@param: scene_file
* 描述障碍物分布和行人分布的 .xml 文件的位置。

@param: pose_initial_x, pose_initial_y, pose_initial_theta
* 机器人初始位置和朝向


## 4. 贡献者
* 杨宜凡
* 张千一
* 宋一诺
* 朱泽卿
* 刘景泰

