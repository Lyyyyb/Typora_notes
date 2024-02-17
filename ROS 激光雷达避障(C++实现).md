# ROS 激光雷达避障(C++实现)

## 实现思路

- 让大管家NodeHandle发布速度控制话题/cmd_vel
- 构建速度控制消息包vel_cmd
- 根据激光雷达的测距数值，实时调整机器人运动速度，避开障碍物

## 示例代码

要使用C++在ROS中实现从激光雷达获取数据并控制机器人底盘以进行避障，你需要进行以下几个步骤：

### 步骤 1: 创建ROS包

1. 创建一个新的ROS包，包含`roscpp`、`sensor_msgs`和`geometry_msgs`依赖项。例如：
   ```
   catkin_create_pkg lidar_obstacle_avoidance roscpp sensor_msgs geometry_msgs
   ```

### 步骤 2: 编写C++节点

在你的ROS包的`src`目录下创建一个C++文件（例如`lidar_obstacle_avoidance_node.cpp`），并写入以下代码：

```cpp
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

class LidarObstacleAvoidance {
public:
    LidarObstacleAvoidance() {
        // 初始化ROS节点
        ros::NodeHandle nh;

        // 订阅激光雷达数据
        laser_sub = nh.subscribe("scan", 1000, &LidarObstacleAvoidance::laserCallback, this);

        // 发布底盘控制指令
        cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    }

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        // 确定是否存在障碍物
        bool obstacle_detected = std::any_of(scan->ranges.begin(), scan->ranges.end(), 
                                             [this](float range) { return range < MIN_DISTANCE; });

        geometry_msgs::Twist move_cmd;
        if (obstacle_detected) {
            // 如果检测到障碍物，停止并旋转
            move_cmd.linear.x = 0.0;
            move_cmd.angular.z = 0.5;
        } else {
            // 否则，直线前进
            move_cmd.linear.x = 0.5;
            move_cmd.angular.z = 0.0;
        }
        cmd_pub.publish(move_cmd);
    }

private:
    ros::Subscriber laser_sub;
    ros::Publisher cmd_pub;
    static constexpr float MIN_DISTANCE = 1.0; // 障碍物检测距离
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_obstacle_avoidance");
    LidarObstacleAvoidance loa;
    ros::spin();
    return 0;
}
```

### 步骤 3: 修改CMakeLists.txt

确保在你的`CMakeLists.txt`中添加了对新节点的编译指令：

```cmake
add_executable(lidar_obstacle_avoidance_node src/lidar_obstacle_avoidance_node.cpp)
target_link_libraries(lidar_obstacle_avoidance_node ${catkin_LIBRARIES})
```

### 步骤 4: 构建和运行节点

1. 在catkin工作空间中构建你的包：
   ```
   catkin_make
   ```
2. 运行你的节点：
   ```
   rosrun lidar_obstacle_avoidance lidar_obstacle_avoidance_node
   ```

### 注意事项

- 这个简单的节点示例将检测到障碍物时停止并旋转，否则继续直线行驶。你可能需要根据实际的机器人和环境调整逻辑和参数。
- 确保激光雷达数据被发布到了正确的话题上（在此示例中为`scan`）。如果不同，请更改订阅的话题名称。
- 确保你的机器人可以接收和响应发布到`cmd_vel`话题的Twist消息。