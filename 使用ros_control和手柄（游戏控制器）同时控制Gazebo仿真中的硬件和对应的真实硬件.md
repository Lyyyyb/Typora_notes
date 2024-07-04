# 使用`ros_control`和手柄（游戏控制器）同时控制Gazebo仿真中的硬件和对应的真实硬件

要用`ros_control`实现手柄控制Gazebo中的仿真硬件，并实现真实硬件和仿真硬件同步，以下是详细的步骤：

### 步骤1：设置ROS环境和安装依赖包

确保你已经安装并配置好ROS，并安装了用于手柄和Gazebo的ROS包（例如`joy`和`gazebo_ros_control`）。

```bash
sudo apt-get update
sudo apt-get install ros-noetic-joy ros-noetic-gazebo-ros-control
```

### 步骤2：创建ROS包

创建一个新的ROS包来保存手柄输入处理节点、Gazebo仿真配置和控制器配置文件。

```bash
cd ~/catkin_ws/src
catkin_create_pkg my_robot_joy_control rospy std_msgs sensor_msgs geometry_msgs controller_manager transmission_interface hardware_interface gazebo_ros_control gazebo_ros joy
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 步骤3：编写手柄输入处理节点

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

### 步骤4：编写硬件接口类

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

### 步骤5：编写Gazebo模型文件

在`urdf`文件夹中创建一个URDF文件来定义机器人模型：

```bash
mkdir -p ~/catkin_ws/src/my_robot_joy_control/urdf
touch ~/catkin_ws/src/my_robot_joy_control/urdf/my_robot.urdf
```

编写URDF模型文件：

`urdf/my_robot.urdf`：

```xml
<robot name="my_robot">
  <link name="base_link">
    <inertial>
      <mass value="1.0" />
      <origin xyz="0 0 0" />
      <inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0.0" ixz="0.0" iyz="0.0" />
    </inertial>
    <visual>
      <geometry>
        <box size="1 1 0.1" />
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="1 1 0.1" />
      </geometry>
    </collision>
  </link>

  <link name="wheel_link">
    <inertial>
      <mass value="1.0" />
      <origin xyz="0 0 0" />
      <inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0.0" ixz="0.0" iyz="0.0" />
    </inertial>
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.1" />
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.1" />
      </geometry>
    </collision>
  </link>

  <joint name="wheel_joint" type="continuous">
    <parent link="base_link" />
    <child link="wheel_link" />
    <origin xyz="0 0 0.05" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <limit effort="10" velocity="10" />
    <dynamics damping="0.1" friction="0.1" />
  </joint>

  <transmission name="trans_wheel_joint">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="wheel_joint">
      <hardwareInterface>EffortJointInterface</hardwareInterface>
    </joint>
    <actuator name="motor">
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>

  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/</robotNamespace>
    </plugin>
  </gazebo>
</robot>
```

### 步骤6：配置控制器

在`config`文件夹中创建控制器配置文件`controllers.yaml`：

```bash
mkdir -p ~/catkin_ws/src/my_robot_joy_control/config
touch ~/catkin_ws/src/my_robot_joy_control/config

/controllers.yaml
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
  joint: wheel_joint

right_wheel_controller:
  type: effort_controllers/JointEffortController
  joint: wheel_joint
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
  <!-- 启动Gazebo仿真环境 -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch" />

  <!-- 启动URDF模型 -->
  <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" args="-param robot_description -urdf -model my_robot" />

  <!-- 加载控制器 -->
  <rosparam file="$(find my_robot_joy_control)/config/controllers.yaml" command="load" />
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

### 步骤9：同步真实硬件和仿真硬件

要实现真实硬件和仿真硬件的同步，你需要确保在手柄输入处理节点中发布的命令同时发送到真实硬件和仿真硬件。这里可以通过发布到不同的话题上来实现。

修改手柄输入处理节点代码：

`src/joy_teleop.py`：

```python
#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

class JoyTeleop:
    def __init__(self):
        rospy.init_node('joy_teleop')

        self.pub_left_wheel_sim = rospy.Publisher('/left_wheel_controller_sim/command', Float64, queue_size=10)
        self.pub_right_wheel_sim = rospy.Publisher('/right_wheel_controller_sim/command', Float64, queue_size=10)
        
        self.pub_left_wheel_real = rospy.Publisher('/left_wheel_controller_real/command', Float64, queue_size=10)
        self.pub_right_wheel_real = rospy.Publisher('/right_wheel_controller_real/command', Float64, queue_size=10)

        rospy.Subscriber("/joy", Joy, self.joy_callback)

        self.left_wheel_speed = 0.0
        self.right_wheel_speed = 0.0

        rospy.spin()

    def joy_callback(self, data):
        self.left_wheel_speed = data.axes[1]  # 假设左摇杆Y轴控制左轮速度
        self.right_wheel_speed = data.axes[4]  # 假设右摇杆Y轴控制右轮速度

        self.pub_left_wheel_sim.publish(self.left_wheel_speed)
        self.pub_right_wheel_sim.publish(self.right_wheel_speed)
        
        self.pub_left_wheel_real.publish(self.left_wheel_speed)
        self.pub_right_wheel_real.publish(self.right_wheel_speed)

if __name__ == '__main__':
    try:
        JoyTeleop()
    except rospy.ROSInterruptException:
        pass
```

### 验证手柄控制

确保手柄连接正确，并运行以下命令来验证手柄输入：

```bash
rostopic echo /joy
```

确保你看到手柄的按键和轴数据。如果手柄数据正确，你的仿真和真实电机应该同步响应手柄输入。

### 总结

- **硬件接口类**：实现硬件接口类来读取和写入硬件状态。
- **手柄输入处理节点**：处理手柄输入并发布控制命令，同时发布到仿真和真实硬件的话题。
- **Gazebo模型文件**：定义机器人模型和控制器。
- **控制器配置文件**：定义控制器类型和关节。
- **启动文件**：启动Gazebo仿真环境、控制节点、手柄节点和手柄输入处理节点。
- **同步控制**：确保手柄输入同时发送到仿真和真实硬件。

通过这些步骤，你可以实现手柄控制Gazebo中的仿真硬件，并实现真实硬件和仿真硬件同步。确保所有硬件接口和通信正确实现和配置，必要时调整代码和配置文件。