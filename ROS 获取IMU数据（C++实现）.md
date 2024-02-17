# ROS 获取IMU数据（C++实现）

## 实现思路

- 构建一个新的软件包，包名叫做imu_pkg
- 在软件包中新建一个节点，节点名叫做imu_node
- 在节点中，向ROS大管家NodeHandle申请订阅话题/imu/data,并设置回调函数为IMUCallback()
- 构建回调函数IMUCallback(),用来接受和处理IMU数据
- 使用TF工具将四元数转换成欧拉角
- 调用ROS_INFO()显示转换后的欧拉角数值



## 示例代码

要使用C++在ROS中获取IMU数据并处理，你需要遵循以下步骤来创建一个名为`imu_pkg`的新软件包，并在其中编写一个名为`imu_node`的节点。该节点将订阅`/imu/data`话题，并使用回调函数`IMUCallback()`来接收和处理IMU数据。此外，我们将使用TF库将四元数转换成欧拉角，并使用`ROS_INFO()`打印欧拉角的数值。

### 步骤 1: 创建软件包

在你的catkin工作空间中，运行以下命令来创建新的软件包：

```bash
cd ~/catkin_ws/src
catkin_create_pkg imu_pkg roscpp sensor_msgs tf
```

这将创建一个名为`imu_pkg`的新软件包，它依赖于`roscpp`、`sensor_msgs`和`tf`。

### 步骤 2: 编写节点

在`imu_pkg`包的`src`目录下创建一个新的C++文件，例如`imu_node.cpp`。写入以下代码：

```cpp
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>

void IMUCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    // 使用TF工具将四元数转换成欧拉角
    tf::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w
    );
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // 显示欧拉角数值
    ROS_INFO("Roll: [%f], Pitch: [%f], Yaw: [%f]", roll, pitch, yaw);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "imu_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/imu/data", 1000, IMUCallback);

    ros::spin();

    return 0;
}
```

### 步骤 3: 编译软件包

在`imu_pkg`的`CMakeLists.txt`中添加编译指令：

```cmake
add_executable(imu_node src/imu_node.cpp)
target_link_libraries(imu_node ${catkin_LIBRARIES})
```

然后，在工作空间的根目录下编译整个工作空间：

```bash
cd ~/catkin_ws
catkin_make
```

### 步骤 4: 运行节点

在终端中运行以下命令来启动节点：

```bash
rosrun imu_pkg imu_node
```

### 注意事项

- 确保你的IMU设备已经配置正确，并且在`/imu/data`话题上发布了`sensor_msgs/Imu`类型的消息。
- 这个示例仅显示如何将四元数转换为欧拉角并打印它们。在实际应用中，你可能需要根据应用需求进一步处理这些数据。