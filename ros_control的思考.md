# ros_control 

## 控制实际机器人



## 控制gazebo中仿真机器人



## 实际和仿真同步

用`ros_control`控制真实电机和仿真Gazebo中的电机主要区别在于硬件接口的实现和环境的配置。以下是详细的区别：

### 1. 硬件接口

**真实电机：**
- 需要编写硬件接口类来与实际的电机硬件通信。这通常涉及到与具体硬件的驱动程序交互，比如通过串口、CAN总线、EtherCAT等通信协议。
- 在硬件接口的`read()`方法中，读取传感器数据（如编码器数据）并更新机器人状态。
- 在硬件接口的`write()`方法中，将控制命令发送到电机控制器。

**仿真电机（Gazebo）：**
- 使用Gazebo提供的`gazebo_ros_control`插件，该插件可以直接使用`ros_control`中的控制器来控制仿真中的电机。
- 不需要编写硬件接口类，Gazebo插件会自动处理仿真环境中关节状态的读取和控制命令的发送。
- 需要在URDF模型文件中添加Gazebo插件的配置，确保Gazebo能够正确加载和使用`ros_control`控制器。

### 2. 配置和启动

**真实电机：**
- 需要在ROS包中创建和配置硬件接口类，并在控制节点中实例化该类。
- 控制节点负责周期性地调用硬件接口的`read()`和`write()`方法，并使用`controller_manager`来管理控制器。
- 需要确保硬件驱动程序和接口库正确安装和配置。

**仿真电机（Gazebo）：**
- 需要在URDF模型文件中定义机器人的物理和传动特性，并配置Gazebo插件来使用`ros_control`。
- 启动文件中需要加载URDF模型并启动Gazebo仿真环境，同时启动控制器。
- Gazebo会自动处理物理仿真，并通过`gazebo_ros_control`插件与`ros_control`框架进行交互。

### 3. 代码示例对比

**真实电机硬件接口示例：**

```cpp
// my_robot_hardware_interface.h
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
};

#endif // MY_ROBOT_HARDWARE_INTERFACE_H
```

```cpp
// my_robot_control_node.cpp
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include "my_robot_control/my_robot_hardware_interface.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_robot_control_node");
    ros::NodeHandle nh;

    MyRobotHardwareInterface my_robot;
    controller_manager::ControllerManager cm(&my_robot, nh);

    ros::Rate rate(50);
    while (ros::ok())
    {
        my_robot.read();
        cm.update(ros::Time::now(), ros::Duration(1.0 / 50.0));
        my_robot.write();
        rate.sleep();
    }

    return 0;
}
```

**Gazebo仿真URDF和启动文件示例：**

```xml
<!-- my_robot.urdf -->
<robot name="my_robot">
  <link name="base_link">
    <!-- Link properties -->
  </link>

  <link name="wheel_link">
    <!-- Link properties -->
  </link>

  <joint name="wheel_joint" type="continuous">
    <parent link="base_link" />
    <child link="wheel_link" />
    <!-- Joint properties -->
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

```xml
<!-- my_robot_gazebo.launch -->
<launch>
  <include file="$(find gazebo_ros)/launch/empty_world.launch" />

  <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" args="-param robot_description -urdf -model my_robot" />

  <rosparam file="$(find my_robot_gazebo)/config/controllers.yaml" command="load" />
  <node name="controller_spawner" pkg="controller_manager" type="spawner" args="joint_state_controller effort_controllers" />
</launch>
```

### 总结

- **真实电机**：需要自定义硬件接口来与实际硬件通信，编写和调试硬件驱动程序。
- **仿真电机**：通过Gazebo仿真，使用预先配置好的插件，不需要编写硬件接口，重点在于正确配置URDF模型和控制器。

通过理解这些差异，可以更好地在不同环境中应用`ros_control`框架，实现机器人控制的需求。
