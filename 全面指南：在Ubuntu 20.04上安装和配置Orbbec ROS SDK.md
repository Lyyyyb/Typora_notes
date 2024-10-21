# 全面指南：在Ubuntu 20.04上安装和配置Orbbec ROS SDK

在Ubuntu 20.04上安装和配置Orbbec ROS SDK涉及到多个步骤，包括安装ROS环境、必要的依赖库，配置ROS工作空间，以及设置相机和相关服务。以下是详细的步骤和方法。

### 安装依赖

#### 1. **安装ROS**
- **ROS的安装**：首先，需要在Ubuntu 20.04上安装ROS。由于Orbbec ROS SDK支持Kinetic, Melodic和Noetic发行版，建议根据系统需求选择合适的版本。可以参考[ROS官方文档](http://wiki.ros.org)进行安装。

#### 2. **安装其他依赖**
- **依赖库安装**：根据选择的ROS版本，安装相应的依赖。打开终端并执行以下命令：
  ```bash
  sudo apt update
  sudo apt install libgflags-dev libgoogle-glog-dev libusb-1.0-0-dev libeigen3-dev \
  ros-$ROS_DISTRO-image-geometry ros-$ROS_DISTRO-camera-info-manager \
  ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-publisher
  ```

### 创建ROS工作空间
- 如果尚未创建工作空间，通过以下命令创建一个新的ROS工作空间：
  ```bash
  mkdir -p ~/ros_ws/src
  cd ~/ros_ws/src/
  ```

### 获取Orbbec ROS SDK
- 从Orbbec开发者社区下载SDK或使用Git克隆官方仓库：
  ```bash
  git clone https://github.com/orbbec/OrbbecSDK_ROS1.git
  ```

### 构建ROS包
- 在ROS工作空间的根目录下构建项目：
  ```bash
  cd ~/ros_ws
  catkin_make
  ```

### 安装udev规则
- 为了使设备能够在非root用户下正常访问，需要配置udev规则：
  ```bash
  cd ~/ros_ws
  source ./devel/setup.bash
  roscd orbbec_camera
  cd scripts
  sudo cp 99-obsensor-libusb.rules /etc/udev/rules.d/
  sudo udevadm control --reload && sudo udevadm trigger
  ```

### 启动相机和可视化工具
- **启动相机节点**：
  ```bash
  source ~/ros_ws/devel/setup.bash
  roslaunch orbbec_camera astra.launch
  ```
- **启动RViz进行可视化**：
  ```bash
  source ~/ros_ws/devel/setup.bash
  rviz
  ```
  在RViz中选择相应的topic进行显示。

### 运行ROS服务和主题
- **查看可用的ROS主题和服务**：
  ```bash
  rostopic list
  rosservice list
  rosparam list
  ```
- **设置和获取相机参数**，例如曝光和增益：
  ```bash
  rosservice call /camera/set_color_auto_exposure '{data: false}'
  rosservice call /camera/get_color_exposure "{}"
  ```

### 保存图像和点云数据
- 当相机运行时，可以保存图像和点云数据到默认目录：
  ```bash
  rosservice call /camera/save_images "{}"
  rosservice call /camera/save_point_cloud "{}"
  ```

这些步骤提供了一个系统性的方法来在Ubuntu 20.04上安装和配置Orbbec ROS SDK，确保用户能够利用Orbbec的3D传感技术进行ROS应用的开发和研究。