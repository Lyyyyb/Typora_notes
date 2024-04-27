# ROS_Control ROS_Controllers

使用 `ros_control` 和 `ros_controllers` 开发一辆两轮差速小车是一个涉及硬件和软件多个层面的过程。以下是详细的步骤，从硬件设定到软件配置和编程：

### 1. 准备硬件
- **两轮差速小车平台**：确保小车的两个电机可以接受电压或PWM信号进行控制。
- **电机驱动器**：例如L298N或TB6612FNG，用于接收来自控制器的命令并驱动电机。
- **微控制器或嵌入式系统**：如Raspberry Pi或Arduino，用于运行ROS节点和处理与电机驱动器的通信。
- **传感器**（可选）：如编码器，用于提供轮速反馈。

### 2. 安装ROS和必要的包
- 在Raspberry Pi或类似设备上安装ROS（如ROS Noetic）。
- 安装 `ros_control` 和 `ros_controllers` 包，可以通过以下命令安装：
  ```bash
  sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers
  ```

### 3. 硬件接口配置
- **编写硬件接口代码**：根据你的硬件具体情况（如电机控制方式），编写一个硬件接口类。这个类需要继承自 `hardware_interface::RobotHW` 类，并在其中实现读取传感器数据和发送控制命令到电机的逻辑。

### 4. 配置控制器
- **控制器配置文件**：创建一个YAML文件配置所需的控制器。对于差速驱动，通常需要至少两个控制器：一个用于左轮速度控制，一个用于右轮速度控制。例如：
  ```yaml
  mobile_base_controller:
    type: "diff_drive_controller/DiffDriveController"
    left_wheel: 'left_wheel_joint'
    right_wheel: 'right_wheel_joint'
    publish_rate: 50
    pose_covariance_diagonal: [0.001, 0.001, 0.001, 0.01, 0.01, 0.01]
    twist_covariance_diagonal: [0.001, 0.001, 0.001, 0.01, 0.01, 0.01]
  ```

### 5. 编写启动文件
- **编写launch文件**：创建一个ROS launch文件来启动控制器管理器、加载控制器配置并连接到硬件接口。这个文件还应该启动任何必要的传感器驱动。

### 6. 测试和调试
- **部署和测试**：将你的代码部署到控制硬件上，然后运行launch文件进行测试。调整控制参数以优化性能。
- **调试**：如果发现问题，可以使用ROS的调试工具，如rqt和rostopic，来查看和调试传递的消息和控制器状态。

### 7. 进行应用级开发
- 根据项目需求，可以在此基础上开发高级应用，如路径规划、避障和导航。

这个过程需要一定的ROS和编程知识，同时对你的具体硬件设备有一定的了解。每一步可能需要根据你的具体情况进行调整和优化。







为了编写一个硬件接口类，用于与差速小车的电机进行通信，我们需要遵循一些基本步骤来实现这个类。以下是一个简化的示例代码，说明如何继承 `hardware_interface::RobotHW` 类，并在其中实现读取传感器数据和发送控制命令到电机的逻辑。

### 示例代码

假设你的差速小车具有两个电机，并且每个轮子都有一个编码器。下面的示例代码展示了如何创建一个硬件接口：

```cpp
#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>

class DiffDriveRobot : public hardware_interface::RobotHW
{
public:
    DiffDriveRobot()
    {
        // 初始化关节状态接口
        hardware_interface::JointStateHandle state_handle_left("left_wheel_joint", &pos[0], &vel[0], &eff[0]);
        jnt_state_interface.registerHandle(state_handle_left);

        hardware_interface::JointStateHandle state_handle_right("right_wheel_joint", &pos[1], &vel[1], &eff[1]);
        jnt_state_interface.registerHandle(state_handle_right);

        registerInterface(&jnt_state_interface);

        // 初始化速度命令接口
        hardware_interface::JointHandle vel_handle_left(jnt_state_interface.getHandle("left_wheel_joint"), &cmd[0]);
        jnt_vel_interface.registerHandle(vel_handle_left);

        hardware_interface::JointHandle vel_handle_right(jnt_state_interface.getHandle("right_wheel_joint"), &cmd[1]);
        jnt_vel_interface.registerHandle(vel_handle_right);

        registerInterface(&jnt_vel_interface);
    }

    void read(const ros::Time& time, const ros::Duration& period)
    {
        // 读取硬件状态（例如，通过编码器）
        // 这里假设 pos, vel 从真实硬件更新
    }

    void write(const ros::Time& time, const ros::Duration& period)
    {
        // 发送命令到硬件（例如，电机驱动器）
        // 使用 cmd[0] 和 cmd[1] 更新硬件
    }

private:
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::VelocityJointInterface jnt_vel_interface;

    double cmd[2] = {0.0, 0.0}; // 电机速度命令
    double pos[2] = {0.0, 0.0}; // 编码器读取的位置
    double vel[2] = {0.0, 0.0}; // 编码器读取的速度
    double eff[2] = {0.0, 0.0}; // 虚拟的扭矩（可能未使用）
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "diff_drive_bot");
    ros::NodeHandle nh;

    DiffDriveRobot robot;
    controller_manager::ControllerManager cm(&robot, nh);

    ros::Rate rate(50); // 50 Hz
    while(ros::ok())
    {
        ros::Time now = ros::Time::now();
        ros::Duration dt = ros::Duration(1.0 / 50.0);

        robot.read(now, dt);
        cm.update(now, dt);
        robot.write(now, dt);

        rate.sleep();
    }
    return 0;
}
```

### 解释

- **初始化接口**：我们在构造函数中为左右轮创建了状态和速度接口。
- **read 方法**：负责从硬件（如编码器）读取当前的位置、速度等状态数据。
- **write 方法**：将计算好的速度命令发送到硬件（如电机控制器）。
- **主函数**：设置了节点，循环调用 `read`，更新控制器，然后调用 `write`。

这个示例简化了许多细节，如硬件通信具体实现等，你需要根据实际硬件接口进行调整。