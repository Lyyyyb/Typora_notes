### 在 Ubuntu 20.04 上通过源码编译安装 cv_bridge 以链接 OpenCV 3.2 的详细步骤

在使用 ROS（机器人操作系统）进行开发时，`cv_bridge` 是一个关键的桥梁包，负责在 ROS 消息和 OpenCV 图像之间进行转换。默认情况下，通过 `apt` 包管理器安装的 `cv_bridge` 会链接到系统中预装的 OpenCV 4.2 版本。然而，在某些特定项目中，可能需要使用较旧的 OpenCV 3.2 版本。为了实现这一点，必须从源码编译 `cv_bridge` 并修改其 `CMakeLists.txt` 文件以指向 OpenCV 3.2。以下将详细解释这一过程，并通过示例加以说明。

#### 一、背景介绍

1. **cv_bridge 功能**：`cv_bridge` 允许在 ROS 和 OpenCV 之间高效地转换图像数据，使得开发者能够利用 OpenCV 强大的图像处理功能来处理 ROS 话题中的图像信息。

2. **默认配置**：在 Ubuntu 20.04 上，使用 `apt` 安装 `cv_bridge` 会自动链接到系统默认的 OpenCV 4.2 版本。这对于大多数应用来说是足够的，但某些情况下可能需要使用特定版本的 OpenCV。

#### 二、为何需要链接到 OpenCV 3.2

1. **兼容性需求**：某些遗留项目或特定的算法可能仅在 OpenCV 3.2 下经过验证和优化，升级到新版本可能导致兼容性问题。

2. **功能依赖**：某些特定功能或模块在 OpenCV 3.2 中存在而在 OpenCV 4.2 中已被修改或移除。

#### 三、从源码编译安装 cv_bridge 并链接到 OpenCV 3.2 的步骤

1. **安装 OpenCV 3.2**

   首先，确保系统中安装了 OpenCV 3.2。可以通过源码编译安装或使用已编译的包（如果可用）。

   ```bash
   sudo apt-get remove libopencv*
   sudo apt-get update
   sudo apt-get install build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
   sudo apt-get install python3.8-dev python3-numpy
   ```

   下载并编译 OpenCV 3.2：

   ```bash
   wget -O opencv.zip https://github.com/opencv/opencv/archive/3.2.0.zip
   unzip opencv.zip
   cd opencv-3.2.0
   mkdir build && cd build
   cmake ..
   make -j$(nproc)
   sudo make install
   ```

2. **获取 cv_bridge 源码**

   首先，确保已安装 ROS 的源码包依赖。

   ```bash
   sudo apt-get install ros-noetic-desktop-full
   sudo apt-get install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
   ```

   初始化 `rosdep`：

   ```bash
   sudo rosdep init
   rosdep update
   ```

   创建工作空间并获取 `cv_bridge` 源码：

   ```bash
   mkdir -p ~/ros_ws/src
   cd ~/ros_ws/src
   git clone https://github.com/ros-perception/vision_opencv.git
   cd vision_opencv
   git checkout noetic  # 根据 ROS 版本选择分支
   ```

3. **修改 CMakeLists.txt 以链接 OpenCV 3.2**

   打开 `cv_bridge` 的 `CMakeLists.txt` 文件并进行以下修改：

   ```cmake
   find_package(OpenCV 3.2 REQUIRED)
   ```

   确保 `find_package` 指令指定了 OpenCV 3.2 的版本。此外，检查所有 OpenCV 相关的链接和包含路径，确保指向 OpenCV 3.2 的安装路径。

4. **编译并安装 cv_bridge**

   返回工作空间根目录，编译整个工作空间：

   ```bash
   cd ~/ros_ws
   rosdep install --from-paths src --ignore-src -r -y
   catkin_make
   ```

   如果使用 `catkin_make`，确保环境变量正确：

   ```bash
   source devel/setup.bash
   ```

#### 四、示例说明

假设我们已经完成上述步骤，现在验证 `cv_bridge` 是否成功链接到 OpenCV 3.2。

1. **创建简单的 ROS 节点**

   在 `~/ros_ws/src` 下创建一个简单的 ROS 包 `cv_bridge_test`：

   ```bash
   cd ~/ros_ws/src
   catkin_create_pkg cv_bridge_test rospy std_msgs sensor_msgs cv_bridge
   ```

   编写节点脚本 `cv_bridge_test_node.py`：

   ```python
   #!/usr/bin/env python3
   import rospy
   from sensor_msgs.msg import Image
   from cv_bridge import CvBridge
   import cv2

   def image_callback(msg):
       bridge = CvBridge()
       cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
       rospy.loginfo("OpenCV version: %s", cv2.__version__)

   def main():
       rospy.init_node('cv_bridge_test_node')
       rospy.Subscriber('/camera/image', Image, image_callback)
       rospy.spin()

   if __name__ == '__main__':
       main()
   ```

   赋予执行权限：

   ```bash
   chmod +x ~/ros_ws/src/cv_bridge_test/scripts/cv_bridge_test_node.py
   ```

2. **编译并运行**

   返回工作空间根目录，编译：

   ```bash
   cd ~/ros_ws
   catkin_make
   source devel/setup.bash
   ```

   运行节点：

   ```bash
   rosrun cv_bridge_test cv_bridge_test_node.py
   ```

   在 ROS 日志中应显示 OpenCV 版本为 `3.2.0`，确认 `cv_bridge` 已成功链接到 OpenCV 3.2。

#### 五、结论

通过上述步骤，用户可以在 Ubuntu 20.04 上从源码编译安装 `cv_bridge`，并将其链接到 OpenCV 3.2。此过程涉及修改 `CMakeLists.txt` 文件以指定所需的 OpenCV 版本，确保在编译过程中使用正确的依赖项。通过示例验证，可以确认 `cv_bridge` 已成功链接到预期的 OpenCV 版本，从而满足特定项目的需求。