# ROS 机器人控制（Python实现）

## 实现思路

- 构建一个新的软件包，包名叫做vel_pkg
- 在软件包中新建一个节点，节点名叫做vel_node.py
- 在节点中，向ROS大管家rospy申请发布话题/cmd_vel，并拿到发布对象vel_pub
- 构建一个gemoetry_msgs/Twist类型的消息包vel_msg，用来承载要发送的速度值
- 开启一个while循环，不停的使用vel_pub对象发送速度消息包vel_msg

## 代码示例

要用Python实现机器人控制程序并发布到`/cmd_vel`话题，你需要编写一个Python脚本，该脚本使用`rospy`库来与ROS通信。下面是一个基本的Python脚本示例，它会周期性地向`/cmd_vel`话题发送速度命令。

首先，确保你的ROS环境配置正确，并且你有一个名为`robot_controller`的ROS包。

1. **创建Python脚本**：

在你的ROS包的`scripts`目录下创建一个新的Python文件，比如叫做`robot_velocity_controller.py`。

```python
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def move_robot():
    # 初始化节点
    rospy.init_node('robot_velocity_controller', anonymous=True)
    
    # 创建Publisher，发布到/cmd_vel话题
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    # 设置循环的频率
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        # 创建Twist消息的实例
        vel_msg = Twist()

        # 设置线速度和角速度
        vel_msg.linear.x = 1.0 # 前进速度1m/s
        vel_msg.angular.z = 0.5 # 旋转速度0.5rad/s

        # 发布消息
        pub.publish(vel_msg)
        rospy.loginfo("Robot moves forwards")
        
        # 按照前面设置的频率延迟
        rate.sleep()

if __name__ == '__main__':
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass
```

记得使脚本可执行：

```sh
chmod +x ~/catkin_ws/src/robot_controller/scripts/robot_velocity_controller.py
```

2. **添加Python脚本到CMakeLists.txt**：

确保你的`CMakeLists.txt`包含了正确的安装指令：

```cmake
catkin_install_python(PROGRAMS scripts/robot_velocity_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

3. **构建你的ROS包**：

在你的ROS工作空间中（通常是`~/catkin_ws`），运行以下命令来构建你的包：

```sh
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

4. **运行你的Python脚本节点**：

确保你的ROS核心服务已经启动：

```sh
roscore
```

在新的终端中，运行你的Python脚本：

```sh
rosrun robot_controller robot_velocity_controller.py
```

这个Python脚本会创建一个ROS节点，周期性地发布速度命令到`/cmd_vel`话题，以控制机器人的移动。你可以根据需要修改速度值和逻辑来实现更复杂的控制策略。