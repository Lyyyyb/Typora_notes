# ROS 获取激光雷达数据 (Python实现)

## 实现思路

- 构建一个新的软件包，包名叫做lidar_pkg
- 在软件包中新建一个节点，节点名叫做lidar_node.py
- 在节点中，向ROS大管家rospy申请订阅话题/scan，并设置回调函数为Lidarcallback()
- 构建回调函数Lidarcallback()，用来接受和处理雷达数据
- 调用loginfo()显示雷达检测到的前方障碍物的距离

## 代码示例

要用Python实现一个ROS节点以获取并处理激光雷达（LiDAR）数据，你需要按照以下步骤操作：

1. **确保已经创建了ROS包**：
   - 如果还没有创建ROS包，请首先创建一个。例如，可以使用命令 `catkin_create_pkg my_lidar_package rospy sensor_msgs` 创建一个包含`rospy`和`sensor_msgs`依赖的新包。

2. **编写节点代码**：
   - 在包的`scripts`文件夹中创建一个新的Python文件，例如`lidar_listener.py`。
   - 编写代码以订阅激光雷达数据。

3. **修改`CMakeLists.txt`**（如果需要）：
   - 确保`CMakeLists.txt`文件中包含了对Python脚本的引用。

4. **使Python脚本可执行**：
   - 通过在脚本文件上设置执行权限使其可执行。

5. **编译并运行节点**：
   - 在catkin工作区中编译包，并运行节点。

### 示例代码：lidar_listener.py

以下是`lidar_listener.py`的一个简单示例，该节点订阅名为`/scan`的激光雷达话题，并在接收到数据时打印消息：

```python
#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

def scan_callback(scan):
    num_readings = len(scan.ranges)
    rospy.loginfo("Received %d laser scan readings", num_readings)
    # 这里可以添加更多处理激光雷达数据的代码

def listener():
    rospy.init_node('lidar_listener', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, scan_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
```

### 使Python脚本可执行

1. 转到脚本所在的目录：
   ```sh
   cd ~/catkin_ws/src/my_lidar_package/scripts
   ```

2. 使脚本可执行：
   ```sh
   chmod +x lidar_listener.py
   ```

### 编译和运行节点

1. **编译包**：
   - 在catkin工作空间的根目录下运行 `catkin_make`。

2. **运行节点**：
   - 首先，确保ROS核心已经运行：`roscore`。
   - 在新的终端中运行节点：
     ```sh
     rosrun my_lidar_package lidar_listener.py
     ```

在运行此节点时，它将订阅`/scan`话题。每当接收到新的激光雷达数据时，它会打印出接收到的测量值数量。请确保你的激光雷达设备或相应的仿真器正在运行，并发布到`/scan`话题。