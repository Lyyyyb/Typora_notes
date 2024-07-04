# ros_control

好的，我将更详细地解释 `ros_control` 框架的使用方法，并提供具体的步骤和示例。

### 1. 安装 `ros_control`

首先，确保你的 ROS 环境已经配置好，并且 `ros_control` 及其依赖项已经安装。使用以下命令安装：

```sh
sudo apt-get update
sudo apt-get install ros-<distro>-ros-control ros-<distro>-ros-controllers
```

将 `<distro>` 替换为你的 ROS 发行版，例如 `melodic` 或 `noetic`。

### 2. 创建硬件接口

硬件接口类负责与机器人硬件进行交互。你需要定义一个继承自 `hardware_interface::RobotHW` 的类，该类将处理从传感器读取数据并将命令发送到执行器。以下是一个简单的示例：

```cpp
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>

class MyRobotHW : public hardware_interface::RobotHW
{
public:
  MyRobotHW()
  {
    // 注册关节状态接口
    hardware_interface::JointStateHandle state_handle_a("joint1", &pos[0], &vel[0], &eff[0]);
    jnt_state_interface.registerHandle(state_handle_a);

    // 注册关节命令接口
    hardware_interface::JointHandle pos_handle_a(jnt_state_interface.getHandle("joint1"), &cmd[0]);
    jnt_pos_interface.registerHandle(pos_handle_a);

    // 注册接口到 RobotHW
    registerInterface(&jnt_state_interface);
    registerInterface(&jnt_pos_interface);
  }

  void read()
  {
    // 从传感器读取关节状态
    // 示例代码：pos[0] = 传感器读取值
  }

  void write()
  {
    // 将命令写入执行器
    // 示例代码：执行器 = cmd[0]
  }

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  double cmd[1];
  double pos[1];
  double vel[1];
  double eff[1];
};
```

### 3. 控制循环和控制器管理器

在 ROS 节点中创建并初始化控制器管理器和控制循环：

```cpp
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <ros/callback_queue.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_robot_control");
  ros::NodeHandle nh;

  MyRobotHW robot;
  controller_manager::ControllerManager cm(&robot, nh);

  ros::Rate rate(50); // 控制频率

  while (ros::ok())
  {
    robot.read();
    cm.update(ros::Time::now(), ros::Duration(1.0 / 50));
    robot.write();
    rate.sleep();
  }

  return 0;
}
```

### 4. 配置控制器

在你的包中创建一个配置文件（例如 `my_robot_control/config/controllers.yaml`），指定要加载的控制器及其参数：

```yaml
joint_state_controller:
  type: joint_state_controller/JointStateController
  publish_rate: 50

pos_joint_traj_controller:
  type: position_controllers/JointPositionController
  joint: joint1
  pid: {p: 100.0, i: 0.01, d: 10.0}
```

### 5. 启动文件

创建一个启动文件（例如 `my_robot_control/launch/control.launch`）来启动控制器：

```xml
<launch>
  <node name="my_robot_control" pkg="my_robot_control" type="my_robot_control_node" output="screen"/>

  <!-- 加载并启动控制器 -->
  <node name="controller_spawner" pkg="controller_manager" type="spawner" args="joint_state_controller pos_joint_traj_controller" />
</launch>
```

### 6. 测试和验证

使用 `roslaunch` 命令启动整个系统并测试控制器：

```sh
roslaunch my_robot_control control.launch
```

### 示例解释

假设我们有一个简单的单关节机械臂机器人，希望通过位置控制器来控制关节的位置。以下是详细的步骤和解释：

1. **硬件接口实现**：
    - 创建 `MyRobotHW` 类，继承自 `hardware_interface::RobotHW`。
    - 在构造函数中注册关节状态接口和位置控制接口。
    - 实现 `read()` 和 `write()` 方法，从传感器读取关节状态并将命令发送到执行器。

2. **控制循环和控制器管理器**：
    - 在 ROS 节点中初始化 `MyRobotHW` 和 `controller_manager`。
    - 在主循环中调用 `read()` 和 `write()` 方法，并使用 `cm.update()` 更新控制器。

3. **配置控制器**：
    - 在 `controllers.yaml` 文件中配置 `joint_state_controller` 和 `pos_joint_traj_controller`。

4. **启动文件**：
    - 创建 `control.launch` 文件，启动 ROS 节点和控制器。

5. **测试和验证**：
    - 使用 `roslaunch` 命令启动系统，验证控制器能够正确地控制机器人关节的位置。

通过这些步骤，你可以使用 `ros_control` 框架实现对机器人硬件的高效控制，并轻松地切换不同的控制器以满足不同的控制需求。如果你有更具体的问题或需要进一步的示例，请随时告诉我！





好的，下面是一个详细的示例，展示如何使用 `ros_control` 框架来控制一台通过 CAN 通信的两轮差速小车。假设你已经有一个支持 CAN 通信的微控制器，并且可以通过 CAN 总线控制电机。

### 1. 安装 `ros_control`

确保已安装 `ros_control` 及其依赖项：

```sh
sudo apt-get update
sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers
```

### 2. 硬件接口类

首先，定义一个继承自 `hardware_interface::RobotHW` 的类，处理与硬件的交互。

#### `my_robot_hw_interface.h`

```cpp
#ifndef MY_ROBOT_HW_INTERFACE_H
#define MY_ROBOT_HW_INTERFACE_H

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <std_msgs/Float64.h>
#include <can_msgs/Frame.h>

class MyRobotHWInterface : public hardware_interface::RobotHW
{
public:
  MyRobotHWInterface(ros::NodeHandle &nh);
  void read();
  void write();

private:
  ros::NodeHandle nh_;
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;

  double cmd_[2];
  double pos_[2];
  double vel_[2];
  double eff_[2];

  ros::Subscriber can_sub_;
  ros::Publisher can_pub_;

  void canCallback(const can_msgs::Frame::ConstPtr &msg);
};

#endif // MY_ROBOT_HW_INTERFACE_H
```

#### `my_robot_hw_interface.cpp`

```cpp
#include "my_robot_hw_interface.h"

MyRobotHWInterface::MyRobotHWInterface(ros::NodeHandle &nh) : nh_(nh)
{
  // 初始化关节状态接口
  hardware_interface::JointStateHandle state_handle_left("left_wheel_joint", &pos_[0], &vel_[0], &eff_[0]);
  joint_state_interface_.registerHandle(state_handle_left);
  hardware_interface::JointStateHandle state_handle_right("right_wheel_joint", &pos_[1], &vel_[1], &eff_[1]);
  joint_state_interface_.registerHandle(state_handle_right);
  registerInterface(&joint_state_interface_);

  // 初始化速度控制接口
  hardware_interface::JointHandle vel_handle_left(joint_state_interface_.getHandle("left_wheel_joint"), &cmd_[0]);
  velocity_joint_interface_.registerHandle(vel_handle_left);
  hardware_interface::JointHandle vel_handle_right(joint_state_interface_.getHandle("right_wheel_joint"), &cmd_[1]);
  velocity_joint_interface_.registerHandle(vel_handle_right);
  registerInterface(&velocity_joint_interface_);

  // 订阅和发布CAN消息
  can_sub_ = nh_.subscribe("can_rx", 10, &MyRobotHWInterface::canCallback, this);
  can_pub_ = nh_.advertise<can_msgs::Frame>("can_tx", 10);
}

void MyRobotHWInterface::canCallback(const can_msgs::Frame::ConstPtr &msg)
{
  // 处理从CAN总线接收的数据，更新关节状态
  // 这里假设CAN消息的data字段包含关节位置或速度数据
}

void MyRobotHWInterface::read()
{
  // 从传感器读取关节状态
  // 这里假设从CAN总线读取到的数据已经通过回调函数更新到pos_和vel_数组中
}

void MyRobotHWInterface::write()
{
  // 将控制命令发送到电机，通过CAN总线发送
  can_msgs::Frame msg;
  msg.id = 0x200; // 设定CAN ID
  msg.dlc = 8; // 数据长度
  msg.data[0] = static_cast<uint8_t>(cmd_[0] & 0xFF);
  msg.data[1] = static_cast<uint8_t>((cmd_[0] >> 8) & 0xFF);
  msg.data[2] = static_cast<uint8_t>((cmd_[0] >> 16) & 0xFF);
  msg.data[3] = static_cast<uint8_t>((cmd_[0] >> 24) & 0xFF);
  msg.data[4] = static_cast<uint8_t>(cmd_[1] & 0xFF);
  msg.data[5] = static_cast<uint8_t>((cmd_[1] >> 8) & 0xFF);
  msg.data[6] = static_cast<uint8_t>((cmd_[1] >> 16) & 0xFF);
  msg.data[7] = static_cast<uint8_t>((cmd_[1] >> 24) & 0xFF);
  can_pub_.publish(msg);
}
```

### 3. 控制循环和控制器管理器

在 ROS 节点中创建并初始化控制器管理器和控制循环：

#### `main.cpp`

```cpp
#include <ros/ros.h>
#include "my_robot_hw_interface.h"
#include <controller_manager/controller_manager.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_robot_control");
  ros::NodeHandle nh;

  MyRobotHWInterface robot(nh);
  controller_manager::ControllerManager cm(&robot, nh);

  ros::Rate rate(50); // 控制频率 50Hz
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while (ros::ok())
  {
    robot.read();
    cm.update(ros::Time::now(), ros::Duration(1.0 / 50.0));
    robot.write();
    rate.sleep();
  }

  return 0;
}
```

### 4. 配置控制器

创建一个配置文件 `controllers.yaml`，指定要加载的控制器及其参数：

#### `controllers.yaml`

```yaml
joint_state_controller:
  type: joint_state_controller/JointStateController
  publish_rate: 50

left_wheel_velocity_controller:
  type: effort_controllers/JointVelocityController
  joint: left_wheel_joint
  pid: {p: 100.0, i: 0.01, d: 10.0}

right_wheel_velocity_controller:
  type: effort_controllers/JointVelocityController
  joint: right_wheel_joint
  pid: {p: 100.0, i: 0.01, d: 10.0}
```

### 5. 启动文件

创建一个启动文件 `control.launch` 来启动控制器：

#### `control.launch`

```xml
<launch>
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"/>
  
  <node name="my_robot_control" pkg="my_robot_control" type="my_robot_control_node" output="screen"/>
  
  <!-- 加载并启动控制器 -->
  <node name="controller_spawner" pkg="controller_manager" type="spawner" args="joint_state_controller left_wheel_velocity_controller right_wheel_velocity_controller"/>
</launch>
```

### 6. 测试和验证

使用 `roslaunch` 命令启动整个系统并测试控制器：

```sh
roslaunch my_robot_control control.launch
```

通过这些步骤，你应该能够使用 `ros_control` 框架来控制你的两轮差速小车，通过 CAN 总线通信与硬件交互。这个示例包括了硬件接口的实现、控制循环和控制器管理器的设置、控制器的配置和启动文件的创建。如果有更多具体问题或需要进一步的帮助，请随时告知！