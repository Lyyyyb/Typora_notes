# ROS 获取IMU数据（Python实现）

## 实现思路

- 构建一个新的软件包，包名叫做imu_pkg
- 在软件包中新建一个节点，节点名叫做imu_node
- 在节点中，向ROS大管家rospy申请订阅话题/imu/data,并设置回调函数为IMUCallback()
- 构建回调函数IMUCallback(),用来接受和处理IMU数据
- 使用TF工具将四元数转换成欧拉角
- 调用ROS_INFO()显示转换后的欧拉角数值



## 代码示例

为了使用Python 3在ROS中实现获取IMU（惯性测量单元）数据的节点，您可以按照以下步骤操作：

### 步骤 1: 创建ROS包

1. 打开一个终端。
2. 导航到您的catkin工作空间的`src`目录（例如：`cd ~/catkin_ws/src`）。
3. 使用`catkin_create_pkg`命令创建一个新的ROS包。例如：`catkin_create_pkg imu_pkg rospy sensor_msgs tf`。这个命令会创建一个名为`imu_pkg`的包，它依赖于`rospy`、`sensor_msgs`和`tf`。

### 步骤 2: 编写Python节点

1. 在`imu_pkg`包的`scripts`目录中创建一个新的Python脚本，例如命名为`imu_node.py`。
2. 编写以下Python代码：

   ```python
   #!/usr/bin/env python
   import rospy
   from sensor_msgs.msg import Imu
   import tf.transformations as tf_trans
   
   def IMUCallback(data):
       # 使用TF将四元数转换为欧拉角
       quaternion = (data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
       euler = tf_trans.euler_from_quaternion(quaternion)
   
       # 打印欧拉角
       rospy.loginfo("Euler Angles: Roll: %f, Pitch: %f, Yaw: %f", euler[0], euler[1], euler[2])
   
   def imu_listener():
       rospy.init_node('imu_node', anonymous=True)
       rospy.Subscriber("/imu/data", Imu, IMUCallback)
       rospy.spin()
   
   if __name__ == '__main__':
       imu_listener()
   ```

   这个脚本会订阅`/imu/data`话题，并在收到IMU数据时，将四元数转换为欧拉角，并打印出来。

### 步骤 3: 使脚本可执行

1. 赋予`imu_node.py`脚本执行权限。在终端中运行：`chmod +x ~/catkin_ws/src/imu_pkg/scripts/imu_node.py`。

### 步骤 4: 编译并运行节点

1. 返回到您的catkin工作空间根目录（例如：`cd ~/catkin_ws`）。
2. 运行`catkin_make`以编译您的包。
3. 使用`. ~/catkin_ws/devel/setup.bash`来更新您的环境。
4. 运行您的节点：`rosrun imu_pkg imu_node.py`。

完成以上步骤后，您的ROS节点应该能够订阅IMU数据，并将四元数转换为欧拉角，然后在控制台上打印出来。

### 注意事项

- 确保您的IMU设备正在发布到`/imu/data`话题。
- 根据您的实际需求，您可能需要调整代码中的话题名称或其他参数。
- `tf.transformations`模块是处理四元数和其他转换的常用工具，需要安装`tf`包。