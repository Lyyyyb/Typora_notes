## ROS 机器人运动控制（C++实现）

## 实现思路

- 构建一个新的软件包，报名叫做vel_pkg
- 在软件包中新建一个节点，节点名叫做vel_node
- 在节点中，向ROS大管家NodeHandle申请发布话题/cmd_vel，并拿到发布对象vel_pub
- 构建一个gemoetry_msgs/Twist类型的软件包vel_msg,用来承载要发送的速度值
- 开启一个while循环，不停的使用vel_pub对象发送速度消息包vel_msg

## 示例代码

在ROS中用C++实现一个机器人控制程序，你需要编写一个节点，它订阅传感器数据等输入，发布速度命令到`/cmd_vel`话题。以下是一个简单的示例，展示了如何实现这样的节点：

首先，确保你有一个配置好的ROS环境，并且已经创建了一个名为`robot_controller`的ROS包。

1. **创建速度控制节点**：

首先，创建一个C++文件，比如`robot_velocity_controller.cpp`，并将其保存在你的ROS包的`src`文件夹中。

```cpp
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "robot_velocity_controller");

    // 创建节点句柄
    ros::NodeHandle nh;

    // 创建一个Publisher对象，用于发布消息到/cmd_vel话题
    ros::Publisher velocity_publisher = nh.advertise<geometry_msgs/Twist>("/cmd_vel", 10);

    // 设置循环频率
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        // 创建一个Twist消息对象
        geometry_msgs::Twist vel_msg;

        // 设置线速度和角速度
        vel_msg.linear.x = 1.0; // 前进速度1m/s
        vel_msg.angular.z = 0.5; // 旋转速度0.5rad/s

        // 发布消息
        velocity_publisher.publish(vel_msg);

        // 在日志中显示消息
        ROS_INFO_STREAM("Sending velocity command:"
                        << " linear=" << vel_msg.linear.x
                        << " angular=" << vel_msg.angular.z);

        // 按照循环频率延时
        loop_rate.sleep();
    }

    return 0;
}
```

2. **在CMakeLists.txt中添加编译指令**：

确保你的`CMakeLists.txt`文件更新了，以编译新创建的C++文件。

```cmake
add_executable(robot_velocity_controller src/robot_velocity_controller.cpp)
target_link_libraries(robot_velocity_controller ${catkin_LIBRARIES})
```

3. **构建你的ROS包**：

在你的ROS工作空间中（通常是`~/catkin_ws`），运行以下命令来构建你的包和新的节点。

```sh
cd ~/catkin_ws
catkin_make
```

4. **运行你的节点**：

首先，确保你的ROS环境是运行中的。

```sh
roscore
```

在新的终端中，使用以下命令运行你的节点：

```sh
rosrun robot_controller robot_velocity_controller
```

以上代码将创建并运行一个简单的ROS节点，该节点以固定的线速度和角速度发布`geometry_msgs/Twist`消息到`/cmd_vel`话题。这个节点可以作为控制机器人移动的基础，可以根据实际需求添加更多的逻辑，比如响应传感器数据或用户输入。



## gemoetry_msgs Twist

`geometry_msgs/Twist`是ROS中`geometry_msgs`包中的一个消息类型，用于表示线速度和角速度。这种消息类型常用于控制机器人或其他移动实体的运动。在C++中使用`geometry_msgs/Twist`时，你通常会创建一个Twist消息，设置其线速度和角速度字段，然后将其发布到一个话题，例如`/cmd_vel`，来控制机器人。

### Twist消息的结构

`geometry_msgs/Twist`消息由两个主要部分组成：

1. **`linear`**：表示线速度，是一个`geometry_msgs/Vector3`类型的消息，包含x, y, z三个方向的速度分量。例如，对于地面机器人，你通常只会设置x分量（前进/后退）。

2. **`angular`**：表示角速度，也是一个`geometry_msgs/Vector3`类型的消息，包含围绕x, y, z轴的旋转速度。对于地面机器人，通常只设置z分量（左/右旋转）。

### 示例：在C++中发布Twist消息

下面是一个C++程序的示例，它创建了一个ROS节点，该节点发布`geometry_msgs/Twist`消息以控制机器人的运动。

首先，确保你有一个配置好的ROS环境，并且已经创建了一个名为`robot_controller`的ROS包。

1. **创建C++文件**

在你的ROS包的`src`目录下创建一个新的C++文件，比如叫做`robot_velocity_publisher.cpp`。

```cpp
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_velocity_publisher");
    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<geometry_msgs/Twist>("/cmd_vel", 1000);
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        geometry_msgs::Twist msg;

        // 设置线速度和角速度
        msg.linear.x = 1.0;  // 前进速度为1 m/s
        msg.angular.z = 0.5; // 以0.5 rad/s的速度旋转

        pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
```

2. **更新CMakeLists.txt**

确保你的`CMakeLists.txt`文件已经更新，以编译新创建的C++文件。

```cmake
add_executable(robot_velocity_publisher src/robot_velocity_publisher.cpp)
target_link_libraries(robot_velocity_publisher ${catkin_LIBRARIES})
```

3. **构建和运行**

在ROS工作空间中构建包，并运行节点：

```sh
cd ~/catkin_ws
catkin_make
rosrun robot_controller robot_velocity_publisher
```

此程序会创建一个名为`robot_velocity_publisher`的节点，它周期性地发布`geometry_msgs/Twist`消息到`/cmd_vel`话题。这个消息包含了机器人的线速度和角速度，可以用来控制机器人的移动和旋转。

