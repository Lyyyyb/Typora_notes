# ros_control 控制真实硬件

好的，这里是一个更详细完整的解决步骤，包括如何使用手柄控制真实硬件电机的每个步骤。

### 步骤1：设置ROS环境和安装依赖包

确保你已经安装并配置好ROS，并安装了用于手柄的ROS包（例如`joy`）。

```bash
sudo apt-get update
sudo apt-get install ros-noetic-joy
```

### 步骤2：创建ROS包

创建一个新的ROS包来保存手柄输入处理节点和控制器配置文件。

```bash
cd ~/catkin_ws/src
catkin_create_pkg my_robot_joy_control rospy std_msgs sensor_msgs geometry_msgs controller_manager transmission_interface hardware_interface joy
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 步骤3：编写硬件接口类

创建一个硬件接口类文件夹和文件：

```bash
mkdir -p ~/catkin_ws/src/my_robot_joy_control/include/my_robot_joy_control
touch ~/catkin_ws/src/my_robot_joy_control/include/my_robot_joy_control/my_robot_hardware_interface.h
touch ~/catkin_ws/src/my_robot_joy_control/src/my_robot_hardware_interface.cpp
```

#### 编写硬件接口头文件

`include/my_robot_joy_control/my_robot_hardware_interface.h`：

```cpp
#ifndef MY_ROBOT_HARDWARE_INTERFACE_H
#define MY_ROBOT_HARDWARE_INTERFACE_H

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

class MyRobotHardwareInterface : public hardware_interface::RobotHW
{
public:
    MyRobotHardwareInterface();
    void read();
    void write();

private:
    hardware_interface::JointStateInterface joint_state_interface;
    hardware_interface::VelocityJointInterface velocity_joint_interface;

    double cmd_[2];
    double pos_[2];
    double vel_[2];
    double eff_[2];

    void initCAN();  // 初始化CAN通信的函数
    void sendCANCommand(int joint_id, double command);  // 发送CAN命令的函数
};

#endif // MY_ROBOT_HARDWARE_INTERFACE_H
```

#### 编写硬件接口实现文件

`src/my_robot_hardware_interface.cpp`：

```cpp
#include "my_robot_joy_control/my_robot_hardware_interface.h"

MyRobotHardwareInterface::MyRobotHardwareInterface()
{
    // 初始化 joint_state_interface
    hardware_interface::JointStateHandle state_handle_a("joint_a", &pos_[0], &vel_[0], &eff_[0]);
    joint_state_interface.registerHandle(state_handle_a);

    hardware_interface::JointStateHandle state_handle_b("joint_b", &pos_[1], &vel_[1], &eff_[1]);
    joint_state_interface.registerHandle(state_handle_b);

    registerInterface(&joint_state_interface);

    // 初始化 velocity_joint_interface
    hardware_interface::JointHandle vel_handle_a(joint_state_interface.getHandle("joint_a"), &cmd_[0]);
    velocity_joint_interface.registerHandle(vel_handle_a);

    hardware_interface::JointHandle vel_handle_b(joint_state_interface.getHandle("joint_b"), &cmd_[1]);
    velocity_joint_interface.registerHandle(vel_handle_b);

    registerInterface(&velocity_joint_interface);

    // 初始化硬件接口，例如CAN总线
    initCAN();
}

void MyRobotHardwareInterface::initCAN()
{
    // 初始化CAN总线通信（具体实现依赖于使用的硬件和库）
    // 例如：openCANPort(), configureCAN(), 等
}

void MyRobotHardwareInterface::read()
{
    // 读取实际硬件的状态，例如从编码器读取关节位置和速度
    // 例如：
    // pos_[0] = readPositionFromEncoder(0);
    // pos_[1] = readPositionFromEncoder(1);
    // vel_[0] = readVelocityFromEncoder(0);
    // vel_[1] = readVelocityFromEncoder(1);
}

void MyRobotHardwareInterface::write()
{
    // 将控制命令发送到电机控制器
    // 例如，通过CAN总线发送控制命令
    sendCANCommand(0, cmd_[0]);
    sendCANCommand(1, cmd_[1]);
}

void MyRobotHardwareInterface::sendCANCommand(int joint_id, double command)
{
    // 构建并发送CAN命令（具体实现依赖于使用的硬件和库）
    // 例如：constructCANMessage(), sendCANMessage(), 等
}
```

### 步骤4：编写控制节点

在`src`文件夹中创建控制节点：

```bash
touch ~/catkin_ws/src/my_robot_joy_control/src/my_robot_control_node.cpp
```

编写控制节点代码：

`src/my_robot_control_node.cpp`：

```cpp
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include "my_robot_joy_control/my_robot_hardware_interface.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_robot_control_node");
    ros::NodeHandle nh;

    // 创建硬件接口对象
    MyRobotHardwareInterface my_robot;
    
    // 创建控制器管理器对象，并将硬件接口传递给控制器管理器
    controller_manager::ControllerManager cm(&my_robot, nh);

    // 设置控制循环频率
    ros::Rate rate(50); // 50 Hz
    while (ros::ok())
    {
        // 调用硬件接口的 read() 方法读取传感器数据
        my_robot.read();

        // 更新控制器管理器，计算控制命令
        cm.update(ros::Time::now(), ros::Duration(1.0 / 50.0));

        // 调用硬件接口的 write() 方法发送控制命令
        my_robot.write();

        // 休眠以保持控制循环频率
        rate.sleep();
    }

    return 0;
}
```

### 步骤5：编写手柄输入处理节点

在`src`文件夹中创建手柄输入处理节点：

```bash
touch ~/catkin_ws/src/my_robot_joy_control/src/joy_teleop.py
chmod +x ~/catkin_ws/src/my_robot_joy_control/src/joy_teleop.py
```

编写手柄输入处理节点代码：

`src/joy_teleop.py`：

```python
#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

class JoyTeleop:
    def __init__(self):
        rospy.init_node('joy_teleop')

        self.pub_left_wheel = rospy.Publisher('/left_wheel_controller/command', Float64, queue_size=10)
        self.pub_right_wheel = rospy.Publisher('/right_wheel_controller/command', Float64, queue_size=10)

        rospy.Subscriber("/joy", Joy, self.joy_callback)

        self.left_wheel_speed = 0.0
        self.right_wheel_speed = 0.0

        rospy.spin()

    def joy_callback(self, data):
        self.left_wheel_speed = data.axes[1]  # 假设左摇杆Y轴控制左轮速度
        self.right_wheel_speed = data.axes[4]  # 假设右摇杆Y轴控制右轮速度

        self.pub_left_wheel.publish(self.left_wheel_speed)
        self.pub_right_wheel.publish(self.right_wheel_speed)

if __name__ == '__main__':
    try:
        JoyTeleop()
    except rospy.ROSInterruptException:
        pass
```

### 步骤6：配置控制器

在`config`文件夹中创建控制器配置文件`controllers.yaml`：

```bash
mkdir -p ~/catkin_ws/src/my_robot_joy_control/config
touch ~/catkin_ws/src/my_robot_joy_control/config/controllers.yaml
```

编写控制器配置文件：

`config/controllers.yaml`：

```yaml
# controllers.yaml
joint_state_controller:
  type: joint_state_controller/JointStateController
  publish_rate: 50

left_wheel_controller:
  type: effort_controllers/JointEffortController
  joint: joint_a

right_wheel_controller:
  type: effort_controllers/JointEffortController
  joint: joint_b
```

### 步骤7：创建启动文件

在`launch`文件夹中创建启动文件`control.launch`：

```bash
mkdir -p ~/catkin_ws/src/my_robot_joy_control/launch
touch ~/catkin_ws/src/my_robot_joy_control/launch/control.launch
```

编写启动文件：

`launch/control.launch`：

```xml
<launch>
  <!-- 启动控制节点 -->
  <node name="my_robot_control_node" pkg="my_robot_joy_control" type="my_robot_control_node" output="screen"/>
  
  <!-- 加载控制器 -->
  <node name="controller_spawner" pkg="controller_manager" type="spawner" args="joint_state_controller left_wheel_controller right_wheel_controller" />

  <!-- 启动手柄节点 -->
  <node name="joy_node" pkg="joy" type="joy_node" output="screen"/>
  
  <!-- 启动手柄输入处理节点 -->
  <node name="joy_teleop" pkg="my_robot_joy_control" type="joy_teleop.py" output="screen"/>
</launch>
```

### 步骤8：编译和运行

1. **编译代码**：

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash


```

2. **启动控制节点、手柄节点和手柄输入处理节点**：

```bash
roslaunch my_robot_joy_control control.launch
```

### 验证手柄控制

确保手柄连接正确，并运行以下命令来验证手柄输入：

```bash
rostopic echo /joy
```

确保你看到手柄的按键和轴数据。如果手柄数据正确，你的电机应该响应手柄输入。

### 总结

- **硬件接口类**：实现硬件接口类来读取和写入硬件状态。
- **控制节点**：周期性地调用硬件接口的`read()`和`write()`方法，并使用`controller_manager`管理控制器。
- **手柄输入处理节点**：处理手柄输入并发布控制命令。
- **控制器配置文件**：定义控制器类型和关节。
- **启动文件**：启动控制节点、手柄节点和手柄输入处理节点。
- **发布控制命令**：通过手柄发布控制命令。

通过这些步骤，你可以使用手柄控制真实硬件电机。确保所有硬件接口和通信正确实现和配置，必要时调整代码和配置文件。