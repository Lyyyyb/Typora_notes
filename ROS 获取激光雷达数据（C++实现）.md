# ROS 获取激光雷达数据（C++实现）

## 实现思路

- 在机器人ROS系统中，激光雷达通常会有一个对应的节点，这个节点一般是由雷达的厂商提供，我们只需要简单的配置以下端口参数，就能和激光雷达的电路系统建立连接，雷达的测距数值从电路系统传递到雷达节点，然后会封装成一个消息包，发布在一个Topic话题中，我们只需要订阅这个话题，就能获取激光雷达的数据了，这个消息包的格式就是上一节介绍的LaserScan格式，发布消息包的话题名称也是约定俗成的，叫做/scan。

![2024-02-17 16-42-33 的屏幕截图](/home/lyb/github/Typora_notes/2024-02-17 16-42-33 的屏幕截图.png)

- 构建一个新的软件包，包名叫做lidar_pkg
- 在软件包中新建一个节点，节点名叫做lidar_node
- 在节点中，向ROS大管家NodeHandle申请订阅话题/scan，并设置回调函数为LidarCallback()
- 构建回调函数LidarCallback()，用来接受和处理雷达数据
- 调用ROS_INFO()显示雷达检测到的前方障碍物的距离



## 代码示例

要用C++实现一个ROS节点，以获取并处理激光雷达（LiDAR）数据，你需要按照以下步骤操作：

1. **确保已经创建了ROS包**：
   - 如果还没有创建ROS包，请首先创建一个。例如，可以使用命令 `catkin_create_pkg my_lidar_package roscpp sensor_msgs` 创建一个包含`roscpp`和`sensor_msgs`依赖的新包。

2. **编写节点代码**：
   - 在包的`src`文件夹中创建一个新的C++文件，例如`lidar_node.cpp`。
   - 编写代码以订阅激光雷达数据。

3. **修改`CMakeLists.txt`**：
   - 确保`CMakeLists.txt`文件正确配置，以便能够编译新节点。

4. **编译并运行节点**：
   - 在catkin工作区中编译包，并运行节点。

### 示例代码：lidar_node.cpp

以下是`lidar_node.cpp`的一个简单示例，该节点订阅名为`/scan`的激光雷达话题，并在接收到数据时打印消息：

```cpp
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    int num_readings = scan->ranges.size();
    ROS_INFO("Received %d laser scan readings", num_readings);
    // 这里可以添加更多处理激光雷达数据的代码
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_listener");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);

    ros::spin();
    return 0;
}
```

### 修改CMakeLists.txt

在`CMakeLists.txt`文件中，添加以下内容以确保节点被正确编译：

```cmake
add_executable(lidar_listener src/lidar_node.cpp)
target_link_libraries(lidar_listener ${catkin_LIBRARIES})
add_dependencies(lidar_listener ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
```

### 编译和运行节点

1. **编译包**：
   - 在catkin工作空间的根目录下运行 `catkin_make`。

2. **运行节点**：
   - 首先，确保ROS核心已经运行：`roscore`。
   - 在新的终端中运行节点：
     ```sh
     rosrun my_lidar_package lidar_listener
     ```

在运行此节点时，它将订阅`/scan`话题。每当接收到新的激光雷达数据时，它会打印出接收到的测量值数量。请确保你的激光雷达设备或相应的仿真器正在运行，并发布到`/scan`话题。