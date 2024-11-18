# 3D（3D Navigation Package）导航包详细介绍与配置指南

#### 目录
1. [包简介](#包简介)
2. [功能概述](#功能概述)
3. [依赖关系](#依赖关系)
4. [安装步骤](#安装步骤)
5. [配置与编译](#配置与编译)
6. [运行3D导航](#运行3d导航)
   - [在仿真环境中启动](#在仿真环境中启动)
   - [在实际机器人（PR2）上启动](#在实际机器人pr2上启动)
7. [导航规划与可视化](#导航规划与可视化)
8. [当前限制与注意事项](#当前限制与注意事项)

---

#### 包简介

**3D导航包（3D Navigation Package）** 是一个基于ROS（Robot Operating System，机器人操作系统）的导航包，旨在为机器人在复杂环境中提供三维导航能力。通过集成三维碰撞检测（3D Collision Checking），该包允许机器人在任意配置下进行导航，例如手臂未收回（Untucked Arms），以完成各种移动操作任务（Mobile Manipulation Tasks），如桌面操作（Tabletop Manipulation）、托盘搬运（Tray Carrying）、拾取与放置（Pick and Place）等。

---

#### 功能概述

**3D导航包（3D Navigation Package）** 的主要功能包括：

1. **三维碰撞检测（3D Collision Checking）**：
   - 使用 **octomap** 构建三维世界模型（3D World Model），实现机器人全身配置的碰撞检测（Collision Checking）。
   
2. **自由配置导航（Configuration-Free Navigation）**：
   - 支持机器人在任何配置下导航，包括手臂未收回状态，适用于各种移动操作任务。

3. **地图构建（Map Building）**：
   - 通过 **octomap_server** 从传感器数据中逐步构建三维环境地图（3D Environment Map），适应动态环境变化。

4. **规划器支持（Planner Support）**：
   - **全局规划器（Global Planner）**：使用 **sbpl_lattice_planner_3d** 插件。
   - **局部规划器（Local Planner）**：使用 **pose_follower_3d** 插件。

5. **仿真支持（Simulation Support）**：
   - 提供 **3d_nav_gazebo** 包，在 **Gazebo** 仿真环境中设置和测试三维导航功能。

---

#### 依赖关系

**3D导航包（3D Navigation Package）** 依赖于多个ROS包，确保这些依赖项已正确安装非常重要。主要依赖项包括：

- **ros-diamondback-pr2-simulator**：PR2机器人仿真器。
- **ros-diamondback-navigation-experimental**：实验性的导航功能。
- **ros-diamondback-motion-planners**：运动规划器。
- **ros-diamondback-pr2-common-actions**：PR2机器人常用动作。
- **ros-diamondback-pr2-apps**：PR2机器人应用程序。
- **ros-diamondback-point-cloud-perception**：点云感知。
- **ros-diamondback-pr2-arm-navigation**：PR2机器人手臂导航。

**注意**：当前代码和说明适用于ROS Diamondback版本（ROS Diamondback），正在开发适用于ROS Electric或Fuerte版本（ROS Electric/Fuerte）的改进版本，预计将更快且性能更优。

---

#### 安装步骤

1. **安装ROS Diamondback及默认栈**：
   确保已安装ROS Diamondback及其默认栈。可以通过以下命令安装：
   ```bash
   sudo apt-get install ros-diamondback-desktop-full
   ```

2. **安装依赖包**：
   使用以下命令安装 `3D导航包` 所依赖的额外ROS包：
   ```bash
   sudo apt-get install ros-diamondback-pr2-simulator \
       ros-diamondback-navigation-experimental \
       ros-diamondback-motion-planners \
       ros-diamondback-pr2-common-actions \
       ros-diamondback-pr2-apps \
       ros-diamondback-point-cloud-perception \
       ros-diamondback-pr2-arm-navigation
   ```

3. **获取 `3d_navigation` 及相关开发代码**：
   使用 `rosinstall` 文件覆盖您的Diamondback安装，获取开发版本的附加栈和 `3d_navigation` 代码。创建一个 `3d_navigation.rosinstall` 文件，内容如下：
   ```yaml
   - svn:
       uri: https://code.ros.org/svn/wg-ros-pkg/branches/trunk_diamondback/sandbox/3d_navigation/
       local-name: stacks/3d_navigation
   - svn:
       uri: http://alufr-ros-pkg.googlecode.com/svn/branches/octomap_mapping-diamondback/
       local-name: stacks/octomap_mapping
   - svn:
       uri: https://code.ros.org/svn/wg-ros-pkg/stacks/motion_planning_common/branches/arm_navigation_metrics
       local-name: stacks/motion_planning_common
   ```

   然后，在工作空间中运行以下命令以检出代码：
   ```bash
   rosinstall . 3d_navigation.rosinstall
   ```

---

#### 配置与编译

1. **设置ROS包路径**：
   确保所有检出的包都在 `ROS_PACKAGE_PATH` 中。例如：
   ```bash
   export ROS_PACKAGE_PATH=~/catkin_ws/stacks/3d_navigation:~/catkin_ws/stacks/octomap_mapping:~/catkin_ws/stacks/motion_planning_common:$ROS_PACKAGE_PATH
   ```

2. **编译代码**：
   使用 `rosmake` 编译 `3d_navigation` 包及其依赖项：
   ```bash
   rosmake --rosdep-install 3d_navigation
   ```
   该命令将自动安装缺失的依赖并编译相关包。

---

#### 运行3D导航

**3D导航（3D Navigation）** 的运行分为在仿真环境中启动和在实际机器人（PR2）上启动两种方式。

##### 在仿真环境中启动

1. **启动Gazebo仿真环境**：
   运行以下命令启动包含3D导航功能的完整Gazebo仿真环境：
   ```bash
   roslaunch 3d_nav_gazebo 3d_nav_gazebo_complete.launch
   ```

2. **配置与测试**：
   仿真环境启动后，可以在 **RViz** 中进行目标设置和路径规划测试。详细参数和配置请参阅 `3d_nav_gazebo` 包中的文档。

##### 在实际机器人（PR2）上启动

1. **调整控制器超时参数**：
   为防止本地规划器（Local Planner）因超时而停止运行，需要增加 `pr2_base_controller` 的超时参数。推荐将默认的0.2秒调整为0.3-0.4秒。具体步骤如下：
   
   - 创建一个覆盖配置文件，例如 `pr2_base_controller2.yaml`。
   - 在该文件中修改参数：
     ```yaml
     base_controller:
       timeout: 0.4  # 将超时设置为0.4秒
     ```
   - 在启动文件中加载该覆盖配置，确保参数正确应用。

2. **启动2D定位（AMCL）**：
   首先启动2D自适应蒙特卡罗定位（Adaptive Monte Carlo Localization，AMCL）：
   ```bash
   roslaunch 3d_nav_executive wg_2dnav_amcl.launch
   ```
   - 在 **RViz** 中设置机器人的初始位姿。
   - 通过遥操作（Teleoperation）驱动机器人短距离移动，帮助粒子滤波器（Particle Filter）收敛至正确位置。

3. **启动3D地图集成**：
   在AMCL稳定后，启动3D地图集成节点：
   ```bash
   roslaunch 3d_nav_executive 3d_nav_environment.launch
   ```
   - 机器人开始从传感器获取数据，逐步构建三维环境地图。

4. **启动3D导航**：
   在获取足够的三维扫描数据后，启动3D导航功能：
   ```bash
   roslaunch 3d_nav_executive move_base_sbpl_3d.launch
   ```
   - **重要**：此时请关闭游戏手柄遥操作（Gamepad Teleoperation），以避免干扰机器人的自动导航命令。

---

#### 导航规划与可视化

1. **设置导航目标**：
   - 打开 **RViz**，并加载 `3d_nav_executive/3d_nav_view` 提供的配置文件。
   - 使用RViz的2D导航界面设置导航目标位置。

2. **规划结果展示**：
   - **有效目标（Valid Goal）**：如果目标配置合法，RViz中会在目标位姿处显示一个绿色的机器人模型（通过 `/planning_scene_visualizer` 主题发布的Marker）。
   - **无效目标（Invalid Goal）**：如果目标位置或起始位置存在碰撞，RViz会在相应位置显示红色球体提示。

3. **规划执行**：
   - 一旦目标配置有效，导航堆栈（Navigation Stack）将自动生成路径并控制机器人移动至目标位置。

---

#### 当前限制与注意事项

1. **自动地图获取（Automatic Map Acquisition）**：
   - 启动时不支持自动地图获取，建议用户通过遥操作手动移动机器人，尤其是原地旋转（Rotate in Place），以快速构建周围环境的三维地图。

2. **传感器视野（Sensor Field of View）**：
   - 目前仅集成了有纹理的窄视野立体相机（Textured Narrow Stereo Camera）。在导航过程中需注意其有限的视野范围，避免因视野不足导致的导航失败。

3. **ROS版本支持（ROS Version Support）**：
   - 当前代码和说明适用于ROS Diamondback版本。针对更高版本（如Electric或Fuerte）的改进版本正在开发中，预计将更快且性能更优。

4. **开发与维护（Development and Maintenance）**：
   - 部分组件已集成至 **octomap_server** 和 **ICRA Sushi Challenge** 项目中。用户可参考这些项目获取更多功能和优化。

---

通过以上详细的介绍与指导，用户可以更清晰地理解 **3D导航包（3D Navigation Package）** 的功能、安装配置步骤以及使用方法，确保在复杂三维环境中实现高效可靠的机器人导航。