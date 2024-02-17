# ROS 激光雷达避障（Python实现）

## 实现思路

- 让大管家rospy发布速度控制话题/cmd_vel
- 构建速度控制消息包el_cmd
- 根据激光雷达的测距数值，实时调整机器人的运动速度，避开障碍物

## 示例代码

要使用Python在ROS中实现从激光雷达获取数据并控制机器人底盘以进行避障，你可以遵循以下步骤：

### 步骤 1: 创建ROS包

1. 创建一个包含`rospy`、`sensor_msgs`和`geometry_msgs`依赖项的ROS包。例如：
   ```sh
   catkin_create_pkg lidar_obstacle_avoidance rospy sensor_msgs geometry_msgs
   ```

### 步骤 2: 编写Python节点

在你的ROS包的`scripts`目录下创建一个Python脚本（例如命名为`lidar_obstacle_avoidance_node.py`），并添加以下代码：

```python
#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class LidarObstacleAvoidance:
    def __init__(self):
        rospy.init_node('lidar_obstacle_avoidance', anonymous=True)

        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.safe_distance = 1.0  # 安全距离，单位：米

    def laser_callback(self, data):
        # 检测障碍物
        min_distance = min(data.ranges)
        move_cmd = Twist()

        if min_distance < self.safe_distance:
            # 遇到障碍物，旋转
            move_cmd.linear.x = 0
            move_cmd.angular.z = 0.5
        else:
            # 没有障碍物，直行
            move_cmd.linear.x = 0.5
            move_cmd.angular.z = 0

        self.cmd_pub.publish(move_cmd)

if __name__ == '__main__':
    node = LidarObstacleAvoidance()
    rospy.spin()
```

### 步骤 3: 使脚本可执行

1. 转到脚本所在目录，并赋予执行权限：
   ```sh
   cd ~/catkin_ws/src/lidar_obstacle_avoidance/scripts
   chmod +x lidar_obstacle_avoidance_node.py
   ```

### 步骤 4: 构建和运行节点

1. 在catkin工作空间中构建你的包：
   ```sh
   catkin_make
   ```
2. 运行你的节点：
   ```sh
   rosrun lidar_obstacle_avoidance lidar_obstacle_avoidance_node.py
   ```

### 注意事项

- 此脚本假设激光雷达数据通过`/scan`话题发布。如果你的激光雷达使用不同的话题，请相应修改订阅话题名称。
- 避障逻辑在此示例中非常简单：如果检测到障碍物，机器人会停止并旋转。根据实际情况，你可能需要调整安全距离或改进避障策略。
- 确保你的机器人支持通过`/cmd_vel`话题接收Twist消息来控制运动。