# Python上位机思路



## 思路

- 导航结束，履带车到达树的附近
- 使用三维雷达获取树的坐标及距离，IMU获取履带车的朝向计算出车要转动的角度，转换成控制指令，通过串口发送给下位机，调整车的朝向，并且需要考虑履带车和Y轴直线模组的长度，保证其在履带车转动过程中不会撞到树。
- 调整车的朝向后，使用三维雷达获取树的坐标，通过TF坐标转换成相对于X轴直线模组最左侧的坐标，通过坐标信息计算出X轴直线模组的滑台需要移动多少距离，进而得到X轴直线模组的步进电机需要运动的步数，转换成控制指令，通过串口发送给下位机。
- 控制Y轴直线模组末端的喷爪张开，使用三维雷达获取树的坐标及距离，通过TF坐标转换成相对于喷爪末端的坐标和距离，计算出车要移动的距离，转换成控制指令，通过串口发送给下位机，喷爪闭合。
- 控制水泵喷漆，控制Z轴直线模组的滑台，使Y轴直线模组上下移动对树均匀喷漆。
- 喷漆结束，喷爪张开，使车后退和时距离，喷爪闭合，继续开始导航。

## 代码框架

```python
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu, NavSatFix, LaserScan
from geometry_msgs.msg import Twist
import serial
import math
import tf

class AutoPaintingRobot:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('auto_painting_robot')

        # 订阅激光雷达、IMU和GPS的数据
        self.lidar_sub = rospy.Subscriber('/lidar', LaserScan, self.lidar_callback)
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
        self.gps_sub = rospy.Subscriber('/gps', NavSatFix, self.gps_callback)

        # 创建用于发送控制命令的发布者
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # 设置串口通信
        self.serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

        # 初始化位置和朝向变量
        self.current_position = None
        self.current_orientation = None
        self.target_tree_position = None

    def lidar_callback(self, data):
        # 处理激光雷达数据，以检测树木位置
        self.target_tree_position = self.detect_tree_from_lidar(data)

    def detect_tree_from_lidar(self, data):
        # 解析激光雷达数据来定位树木
        # 返回树木的位置
        # TODO: 实现树木检测算法
        return tree_position

    def imu_callback(self, data):
        # 更新当前的朝向信息，基于IMU数据
        self.current_orientation = self.get_orientation_from_imu(data)

    def get_orientation_from_imu(self, data):
        # 从IMU数据中提取朝向
        # TODO: 实现朝向提取算法
        return orientation

    def gps_callback(self, data):
        # 更新当前的位置信息，基于GPS数据
        self.current_position = self.get_position_from_gps(data)

    def get_position_from_gps(self, data):
        # 从GPS数据中提取位置
        # TODO: 实现位置提取算法
        return position

    def navigate_to_tree(self):
        # 导航到树木附近
        # TODO: 实现导航逻辑
        pass

    def adjust_orientation(self):
        # 调整机器人朝向以面对树木
        # TODO: 实现朝向调整逻辑
        pass

    def control_painting_mechanism(self):
        # 控制喷漆机械臂进行喷漆操作
        # TODO: 实现喷漆控制逻辑
        pass

    def send_command_to_base(self, command):
        # 通过串口向下位机发送控制指令
        try:
            self.serial_port.write(command.encode())
        except serial.SerialException as e:
            rospy.logerr("Serial communication error: %s", e)

    def run(self):
        # 主循环
        rate = rospy.Rate(10)  # 设置频率为10Hz
        while not rospy.is_shutdown():
            # 执行导航、朝向调整和喷漆控制
            self.navigate_to_tree()
            self.adjust_orientation()
            self.control_painting_mechanism()
            rate.sleep()

if __name__ == '__main__':
    robot = AutoPaintingRobot()
    robot.run()
```

### 代码解释说明

- **初始化 (`__init__` 方法)**: 此方法初始化ROS节点，订阅必要的传感器数据（激光雷达、IMU和GPS），设置串口通信，并初始化一些存储状态的变量。

- **激光雷达回调 (`lidar_callback` 方法)**: 当激光雷达数据到达时被调用，用于处理激光雷达数据并更新树木的位置。

- **IMU回调 (`imu_callback` 方法)**: 当IMU数据到达时被调用，用于更新机器人的当前朝向信息。

- **GPS回调 (`gps_callback` 方法)**: 当GPS数据到达时被调用，用于更新机器人的当前位置信息。

- **导航到树 (`navigate_to_tree` 方法)**: 包含控制机器人移动到树木附近的逻辑。

- **调整朝向 (`adjust_orientation` 方法)**: 包含根据当前朝向和树木的位置调整机器人方向的逻辑。

- **控制喷漆机械臂 (`control_painting_mechanism` 方法)**: 包含控制喷漆过程的逻辑。

- **发送串口指令 (`send_command_to_base` 方法)**: 向下位机发送通过串口编码的指令。

- **运行主循环 (`run` 方法)**: 这是机器人控制程序的主循环，它周期性地执行导航、调整朝向和控制喷漆的操作。

### 注意事项

- 该代码需要根据实际的硬件配置和数据格式进行相应的填充和调整。
- 安全措施和异常处理非常重要，应在实现中给予特别注意。
- 代码中的“TODO”部分表示需要根据具体应用场景和硬件特性来实现的部分。



## 具体步骤实现思路

### 使用三维雷达获取树的坐标及距离，IMU获取履带车的朝向计算出车要转动的角度，转换成控制指令，通过串口发送给下位机，调整车的朝向，并且需要考虑履带车和Y轴直线模组的长度，保证其在履带车转动过程中不会撞到树。

这一步骤的实现涉及到几个关键部分：使用三维雷达（如激光雷达）获取树的位置，使用IMU（惯性测量单元）获取履带车的当前朝向，计算转动角度，生成相应的控制指令，通过串口发送给下位机来调整车的朝向。此外，还需考虑履带车和Y轴直线模组的长度，以避免在转动过程中撞到树。

### 实现步骤和示例

1. **获取树的位置**:
   - 使用三维雷达扫描周围环境，识别树木的位置。
   - 假设雷达返回树木相对于雷达的坐标`(x_radar, y_radar, z_radar)`。

   ```python
   def detect_tree_from_lidar(self, lidar_data):
       # 假设有函数可以处理雷达数据并找到树木位置
       tree_position = process_lidar_data(lidar_data)
       return tree_position
   ```

2. **获取履带车当前朝向**:
   - 从IMU数据中读取当前朝向，可能是四元数或欧拉角。
   - 转换成适合计算的格式，如转换四元数为欧拉角。

   ```python
   def get_orientation_from_imu(self, imu_data):
       # 假设IMU数据以四元数形式给出
       quaternion = [imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w]
       euler = tf.transformations.euler_from_quaternion(quaternion)
       return euler  # 返回欧拉角
   ```

3. **计算转动角度并生成控制指令**:
   - 根据雷达获取的树的位置和IMU的朝向，计算履带车需要转动的角度。
   - 考虑履带车和Y轴直线模组的长度，确保不会撞到树。

   ```python
   def calculate_turn_angle(self, tree_position, current_orientation):
       # 假设current_orientation为机器人朝向的欧拉角
       # 计算机器人当前朝向与树木之间的角度差
       angle_to_tree = math.atan2(tree_position.y, tree_position.x)
       turn_angle = angle_to_tree - current_orientation.yaw
       return turn_angle
   ```

   - 将计算出的转动角度转换成控制指令。

   ```python
   def create_turn_command(self, turn_angle):
       # 创建控制指令
       command = "TURN " + str(turn_angle)
       return command
   ```

4. **通过串口发送控制指令**:
   - 将控制指令通过串口发送给下位机，以调整履带车的朝向。

   ```python
   def send_command_to_base(self, command):
       try:
           self.serial_port.write(command.encode())
       except serial.SerialException as e:
           rospy.logerr("Serial communication error: %s", e)
   ```

### 综合示例

将上述逻辑综合到一个函数中：

```python
def adjust_orientation_to_tree(self):
    # 获取树的位置
    tree_position = self.detect_tree_from_lidar(self.lidar_data)  # 假设lidar_data是已获取的数据

    # 获取当前朝向
    current_orientation = self.get_orientation_from_imu(self.imu_data)  # 假设imu_data是已获取的数据

    # 计算转动角度
    turn_angle = self.calculate_turn_angle(tree_position, current_orientation)

    # 创建控制指令
    turn_command = self.create_turn_command(turn_angle)

    # 发送控制指令
    self.send_command_to_base(turn_command)
```

在这个函数中，我们首先获取树木的位置和机器人的当前朝向，然后计算出机器人需要转动的角度，并创建相应的转动控制指令。最后，这个指令被

发送给下位机以调整机器人的朝向。

### 注意事项

- 确保雷达和IMU数据的准确性和可靠性。
- 在实际应用中，转动机器人时需要考虑机器人的动态特性和可能的延迟。
- 安全检查非常重要，以确保在转动过程中不会碰撞到树或其他障碍物。

### 调整车的朝向后，使用三维雷达获取树的坐标，通过TF坐标转换成相对于X轴直线模组最左侧的坐标，通过坐标信息计算出X轴直线模组的滑台需要移动多少距离，进而得到X轴直线模组的步进电机需要运动的步数，转换成控制指令，通过串口发送给下位机。

这一步骤涉及到利用三维雷达获取树的坐标，然后使用TF（Transform）库进行坐标转换，以确定X轴直线模组的滑台需要移动的距离，最后将这个距离转换成步进电机的步数，并生成控制指令发送给下位机。下面是如何实现这一步骤的详细解释：

### 1. 获取树木的坐标

首先，使用三维雷达（如激光雷达）扫描周围环境，获取树木的坐标。假设雷达数据包含了树木相对于雷达的坐标。

```python
def get_tree_position_from_lidar(self, lidar_data):
    # 处理激光雷达数据并返回树木的坐标
    tree_position = process_lidar_data_for_tree(lidar_data)
    return tree_position
```

### 2. 使用TF进行坐标转换

利用TF库将树木的坐标从雷达坐标系转换到与X轴直线模组相关的坐标系。这需要您事先知道雷达坐标系和X轴直线模组坐标系之间的相对位置和方向。

```python
def convert_to_module_frame(self, tree_position, listener):
    # 假设listener是一个tf.TransformListener对象
    try:
        # 等待坐标系的转换关系
        listener.waitForTransform("/module_frame", "C", rospy.Time(0), rospy.Duration(4.0))
        # 进行坐标转换
        tree_position_module_frame = listener.transformPoint("/module_frame", tree_position)
        return tree_position_module_frame
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logerr("TF转换异常: %s", e)
        return None
```

### 3. 计算滑台移动距离

根据转换后的坐标，计算出X轴直线模组的滑台需要移动的距离。

```python
def calculate_movement_for_module(self, tree_position_module_frame):
    # 假设直线模组的起点为(0,0,0)，计算滑台需要移动的距离
    movement_distance = tree_position_module_frame.point.x  # 根据实际情况可能需要调整
    return movement_distance
```

### 4. 计算步进电机的步数

将滑台需要移动的距离转换成步进电机的步数。这需要知道步进电机每步对应的移动距离（步距）。

```python
def calculate_steps_for_motor(self, movement_distance, step_distance):
    # 计算步进电机需要运动的步数
    steps = int(movement_distance / step_distance)  # 可能需要进行取整操作
    return steps
```

### 5. 发送控制指令

最后，将步数转换为控制指令，并通过串口发送给下位机。

```python
def send_motor_command(self, steps):
    command = "MOVE X " + str(steps)
    self.send_command_to_base(command)
```

### 整合到主程序

在主程序的适当位置调用这些函数，以实现整个流程。

```python
# 在主循环或适当的函数中
def control_x_axis_module(self):
    tree_position = self.get_tree_position_from_lidar(self.lidar_data)
    tree_position_module_frame = self.convert_to_module_frame(tree_position, self.tf_listener)
    if tree_position_module_frame is not None:
        movement_distance = self.calculate_movement_for_module(tree_position_module_frame)
        steps = self.calculate_steps_for_motor(movement_distance, STEP_DISTANCE)
        self.send_motor_command(steps)
```

### 注意事项

- 确保TF转换准确：TF坐标转换依赖于准确的坐标系关系，需要确保这些关系在URDF或其他配置中正确定义。
- 考虑实际机械误差：在计算

步进电机步数时考虑实际的机械误差和步距。
- 安全性检查：在移动机械部件之前进行必要的安全检查，以确保操作安全。

### 控制Y轴直线模组末端的喷爪张开，使用三维雷达获取树的坐标及距离，通过TF坐标转换成相对于喷爪末端的坐标和距离，计算出车要移动的距离，转换成控制指令，通过串口发送给下位机，喷爪闭合。

这一步骤包括多个关键操作：控制Y轴直线模组末端的喷爪开合，利用三维雷达获取树的位置，通过TF进行坐标转换，计算机器人需要移动的距离，并将这些信息转换成控制指令发送给下位机。下面是实现这一步骤的详细过程：

### 1. 控制喷爪张开

首先，需要生成一个控制指令来控制喷爪张开。这通常涉及到发送一个特定的信号或命令到控制喷爪的执行器。

```python
def open_spray_claw(self):
    # 发送指令以张开喷爪
    open_command = "OPEN_CLAW"
    self.send_command_to_base(open_command)
```

### 2. 使用三维雷达获取树的坐标及距离

通过三维雷达（如激光雷达）获取树木的坐标和距离。这涉及到解析雷达传感器的数据，识别树木的位置。

```python
def get_tree_position_from_lidar(self, lidar_data):
    # 解析雷达数据以获取树木的坐标和距离
    tree_position = process_lidar_data_for_tree(lidar_data)
    return tree_position
```

### 3. 通过TF坐标转换

使用TF库将树木的坐标从雷达坐标系转换到与喷爪末端相关的坐标系。

```python
def convert_to_spray_claw_frame(self, tree_position, listener):
    try:
        # 等待坐标系的转换关系
        listener.waitForTransform("/spray_claw_frame", "/lidar_frame", rospy.Time(0), rospy.Duration(4.0))
        # 进行坐标转换
        tree_position_spray_claw_frame = listener.transformPoint("/spray_claw_frame", tree_position)
        return tree_position_spray_claw_frame
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logerr("TF转换异常: %s", e)
        return None
```

### 4. 计算机器人需要移动的距离

根据转换后的坐标，计算机器人与树木的相对距离，确定机器人需要移动多远以便喷爪可以接触到树木。

```python
def calculate_movement_for_spray(self, tree_position_spray_claw_frame):
    # 假设计算得到的距离是机器人与树木之间的直线距离
    movement_distance = math.sqrt(tree_position_spray_claw_frame.point.x ** 2 + tree_position_spray_claw_frame.point.y ** 2)
    return movement_distance
```

### 5. 转换成控制指令并发送

将计算出的移动距离转换成控制指令，并通过串口发送给下位机。

```python
def send_movement_command(self, movement_distance):
    move_command = "MOVE " + str(movement_distance)
    self.send_command_to_base(move_command)
```

### 6. 喷爪闭合

完成移动后，发送指令以闭合喷爪。

```python
def close_spray_claw(self):
    # 发送指令以闭合喷爪
    close_command = "CLOSE_CLAW"
    self.send_command_to_base(close_command)
```

### 整合到主程序

在主程序中，您可以按顺序调用这些函数，以实现从喷爪张开到定位树木，再到移动机器人，最后闭合喷爪的完整流程。

```python
def control_spray_process(self):
    self.open_spray_claw()
    tree_position = self.get_tree_position_from_lidar(self.lidar_data)
    tree_position_spray_claw_frame = self.convert_to_spray_claw_frame(tree_position, self.tf_listener)
    if tree_position_spray_claw_frame is not None:
        movement_distance = self.calculate_movement_for

_spray(tree_position_spray_claw_frame)
        self.send_movement_command(movement_distance)
    self.close_spray_claw()
```

### 注意事项

- 在实际操作中，确保有足够的安全检查，防止机器人碰撞或其他意外情况发生。
- 移动机器人时需要考虑其动态特性和可能的延迟。
- 适当地处理TF坐标转换中可能出现的异常情况。



## 导航和喷树的状态切换

要实现在导航结束后自动执行喷树节点，并在喷树节点结束后自动重新执行导航节点，您需要建立一个状态机来管理这些不同的操作阶段。状态机是一种编程模型，用于设计具有明确状态的系统。在这个案例中，可以定义至少两个状态：导航和喷树。

以下是使用Python和ROS实现此功能的详细步骤：

### 1. 定义状态

首先，定义程序的不同状态。在这个例子中，至少有两个状态：`NAVIGATING` 和 `SPRAYING`。

```python
class RobotState:
    NAVIGATING = 1
    SPRAYING = 2
```

### 2. 实现状态转换逻辑

创建一个类来管理状态转换和相应的操作。

```python
class PaintingRobot:
    def __init__(self):
        self.state = RobotState.NAVIGATING
        # 初始化其他必要的组件，例如ROS节点、发布者和订阅者

    def navigate(self):
        # 导航逻辑
        # 当导航完成时，切换到喷树状态
        self.state = RobotState.SPRAYING

    def spray_tree(self):
        # 喷树逻辑
        # 当喷树操作完成时，切换回导航状态
        self.state = RobotState.NAVIGATING

    def run(self):
        while not rospy.is_shutdown():
            if self.state == RobotState.NAVIGATING:
                self.navigate()
            elif self.state == RobotState.SPRAYING:
                self.spray_tree()
            # 其他必要的循环操作
```

### 3. 导航结束和喷树操作的触发

在`navigate`和`spray_tree`方法中，您需要实现具体的逻辑来完成各自的任务。当一个任务完成时，更新状态以反映变化。

### 4. 运行状态机

最后，在程序的主部分运行状态机。

```python
if __name__ == '__main__':
    rospy.init_node('painting_robot')
    robot = PaintingRobot()
    robot.run()
```

### 说明

- 在`navigate`方法中，您需要实现导航到指定位置的逻辑。当到达目的地时，状态改变为`SPRAYING`。
- 在`spray_tree`方法中，您需要实现喷漆的逻辑。完成喷漆后，状态改变为`NAVIGATING`。
- 这个状态机模型允许机器人在完成一个任务后自动切换到下一个任务。

### 注意事项

- **安全检查**：确保在切换状态和执行任务时考虑到安全性，尤其是在与环境交互时。
- **错误处理**：在执行导航和喷漆任务时，应适当处理可能出现的任何错误或异常情况。
- **状态监控**：在实际的机器人应用中，可能需要实现一种机制来监控和记录当前状态，以便于调试和维护。
- **资源管理**：确保在状态转换时适当管理资源，如关闭不需要的传感器或控制器，以优化性能和能耗。

这个状态机模型为复杂任务的管理提供了一种清晰和可维护的方法，特别是在需要机器人自动从一个任务转换到另一个任务的场景中。

## TF 坐标转换

要定义`/module_frame`和`/spray_claw_frame`相对于`/lidar_frame`的坐标系，您需要使用ROS的TF库来发布这些坐标系之间的变换关系。这涉及到确定每个坐标系相对于激光雷达坐标系的位置（平移）和方向（旋转）。以下是具体实现的步骤：

### 1. 导入必要的库

首先，您需要导入ROS和TF相关的库。

```python
import rospy
import tf
from geometry_msgs.msg import TransformStamped
```

### 2. 定义TF广播器

创建一个TF广播器来发布坐标系变换。

```python
class FramePublisher:
    def __init__(self):
        rospy.init_node('frame_publisher')
        self.br = tf.TransformBroadcaster()
```

### 3. 发布坐标系变换

在发布器类中，定义一个方法来发布`/module_frame`和`/spray_claw_frame`相对于`/lidar_frame`的坐标变换。

```python
def publish_transforms(self):
    # 当前时间
    current_time = rospy.Time.now()

    # 发布 /module_frame 相对于 /lidar_frame 的变换
    self.send_transform(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, current_time, "/lidar_frame", "/module_frame")

    # 发布 /spray_claw_frame 相对于 /lidar_frame 的变换
    self.send_transform(0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, current_time, "/lidar_frame", "/spray_claw_frame")
```

这里的`send_transform`是一个辅助方法，用于发送坐标变换。这些变换的具体值应根据您的机器人设计和传感器布局进行调整。

### 4. 定义发送变换的辅助方法

添加一个辅助方法来简化变换的发送过程。

```python
def send_transform(self, tx, ty, tz, rx, ry, rz, rw, time, parent_frame, child_frame):
    # 创建变换消息
    t = TransformStamped()
    t.header.stamp = time
    t.header.frame_id = parent_frame
    t.child_frame_id = child_frame
    t.transform.translation.x = tx
    t.transform.translation.y = ty
    t.transform.translation.z = tz
    t.transform.rotation.x = rx
    t.transform.rotation.y = ry
    t.transform.rotation.z = rz
    t.transform.rotation.w = rw

    # 发送变换
    self.br.sendTransformMessage(t)
```

### 5. 运行广播器

最后，在主函数中创建`FramePublisher`的实例，并定期调用`publish_transforms`方法来发布变换。

```python
if __name__ == '__main__':
    frame_publisher = FramePublisher()
    rate = rospy.Rate(10.0)  # 10 Hz
    while not rospy.is_shutdown():
        frame_publisher.publish_transforms()
        rate.sleep()
```

### 完整代码

将以上部分组合起来，形成完整的脚本。

### 注意事项

- 在实际应用中，您需要根据机器人和激光雷达的实际布局来调整平移（`tx`, `ty`, `tz`）和旋转（`rx`, `ry`, `rz`, `rw`）的参数。
- 确保您的TF树保持一致性，特别是在动态环境中，不断更新的坐标变换信息非常重要。
- 本示例假设激光雷达位于机器人的固定位置。如果激光雷达或其他部件在运动中改变位置或方向，您需要相应更新变换信息。



## 代码简化

- 机器人类

  共有属性：订阅激光雷达数据话题，机器人初始状态

  共有方法：雷达回调函数，解析雷达数据函数，创建串口指令函数，串口指令发送函数，导航喷树逻辑切换函数，计算机器人与树的距离函数

  - 履带车类

    特有属性：车的当前位置，朝向和目标位置，订阅IMU数据话题，串口初始化控制履带车

    特有方法：IMU回调函数，解析IMU数据获取朝向函数，计算机器人与树的角度差函数，创建串口指令函数控制履带车（多态），串口指令发送函数控制履带车（多态），计算机器人与树的距离函数（多态）

  - 直线模组类

    特有属性：TF广播器初始化，串口初始化控制直线模组

    特有函数：发布TF广播函数，发布TF转换函数，计算滑台移动距离函数，计算步进电机运动步数函数，串口发送指令函数控制直线模组（多态），创建串口指令函数控制直线模组（多态），计算机器人与树的距离函数（多态）

```python
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu, NavSatFix, LaserScan
from geometry_msgs.msg import Twist
import serial
import math
import tf

class RobotState:
    NAVIGATING = 1
    SPRAYING = 2

# 父类：AutoPaintingRobot
class AutoPaintingRobot:
    def __init__(self):
        rospy.init_node('auto_painting_robot')
        self.lidar_sub = rospy.Subscriber('/lidar', LaserScan, self.lidar_callback)
        self.state = RobotState.NAVIGATING

    def lidar_callback(self, data):
        self.target_tree_position = self.detect_tree_from_lidar(data)

    def detect_tree_from_lidar(self, data):
        distance_threshold = 3.0
        closest_distance = float('inf')
        closest_angle = None
        for angle in range(len(data.ranges)):
            distance = data.ranges[angle]
            if distance < closest_distance:
                closest_distance = distance
                closest_angle = angle
        if closest_distance < distance_threshold:
            return (closest_distance, closest_angle)
        else:
            return None

    def create_serial_command(self, command):
        raise NotImplementedError("Must be implemented in subclass")

    def send_serial_command(self, command):
        raise NotImplementedError("Must be implemented in subclass")

    def switch_navigation_state(self):
        if self.state == RobotState.NAVIGATING:
            self.navigate_to_tree()
        elif self.state == RobotState.SPRAYING:
            self.spray_tree()

    def calculate_distance_to_tree(self, tree_position):
        # ...原有的距离计算逻辑...

# 子类：TrackVehicle
class TrackVehicle(AutoPaintingRobot):
    def __init__(self):
        super().__init__()
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

    def imu_callback(self, data):
        self.current_orientation = self.get_orientation_from_imu(data)

    def get_orientation_from_imu(self, data):
        quaternion = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        euler = tf.transformations.euler_from_quaternion(quaternion)
        return euler

    def create_serial_command(self, command):
        return "Vehicle " + command

    def send_serial_command(self, command):
        try:
            self.serial_port.write(command.encode())
        except serial.SerialException as e:
            rospy.logerr("Track vehicle serial communication error: %s", e)

    def calculate_turn_angle(self, tree_position):
        # ...原有的角度计算逻辑...

# 子类：LinearModule
class LinearModule(AutoPaintingRobot):
    def __init__(self):
        super().__init__()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.serial_port = serial.Serial('/dev/ttyUSB1', 9600, timeout=1)

    def publish_transforms(self):
        current_time = rospy.Time.now()
        self.send_transform(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, current_time, "/lidar_frame", "/module_frame")
        self.send_transform(0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, current_time, "/lidar_frame", "/spray_claw_frame")

    def send_transform(self, tx, ty, tz, rx, ry, rz, rw, time, parent_frame, child_frame):
        self.tf_broadcaster.sendTransform((tx, ty, tz), (rx, ry, rz, rw), time, child_frame, parent_frame)

    def create_serial_command(self, command):
        return "Module " + command

    def send_serial_command(self, command):
        try:
            self.serial_port.write(command.encode())
        except serial.SerialException as e:
            rospy.logerr("Linear module serial communication error: %s", e)

# 主程序
if __name__ == '__main__':
    vehicle = TrackVehicle()
    module = LinearModule()
    # 使用vehicle和module执行特定操作

```

```python
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu, NavSatFix, LaserScan
from geometry_msgs.msg import Twist
import serial
import math
import tf


# 父类：AutoPaintingRobot
class AutoPaintingRobot:
    def __init__(self):
        rospy.init_node('auto_painting_robot')
        self.lidar_sub = rospy.Subscriber('/lidar', LaserScan, self.lidar_callback)

    def lidar_callback(self, data):
        self.target_tree_position = self.detect_tree_from_lidar(data)

    def detect_tree_from_lidar(self, data):
        distance_threshold = 3.0
        closest_distance = float('inf')
        closest_angle = None
        for angle in range(len(data.ranges)):
            distance = data.ranges[angle]
            if distance < closest_distance:
                closest_distance = distance
                closest_angle = angle
        if closest_distance < distance_threshold:
            return (closest_distance, closest_angle)
        else:
            return None

    def create_serial_command(self, command):
        raise NotImplementedError("Must be implemented in subclass")

    def send_serial_command(self, command):
        raise NotImplementedError("Must be implemented in subclass")

    def calculate_distance_to_tree(self, tree_position):
        # ...原有的距离计算逻辑...

# 子类：TrackVehicle
class TrackVehicle(AutoPaintingRobot):
    def __init__(self):
        super().__init__()
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

    def imu_callback(self, data):
        self.current_orientation = self.get_orientation_from_imu(data)

    def get_orientation_from_imu(self, data):
        quaternion = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        euler = tf.transformations.euler_from_quaternion(quaternion)
        return euler

    def create_serial_command(self, command):
        return "Vehicle " + command

    def send_serial_command(self, command):
        try:
            self.serial_port.write(command.encode())
        except serial.SerialException as e:
            rospy.logerr("Track vehicle serial communication error: %s", e)

    def calculate_turn_angle(self, tree_position):
        # ...原有的角度计算逻辑...

# 子类：LinearModule
class LinearModule(AutoPaintingRobot):
    def __init__(self):
        super().__init__()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.serial_port = serial.Serial('/dev/ttyUSB1', 9600, timeout=1)

    def publish_transforms(self):
        current_time = rospy.Time.now()
        self.send_transform(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, current_time, "/lidar_frame", "/module_frame")
        self.send_transform(0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, current_time, "/lidar_frame", "/spray_claw_frame")

    def send_transform(self, tx, ty, tz, rx, ry, rz, rw, time, parent_frame, child_frame):
        self.tf_broadcaster.sendTransform((tx, ty, tz), (rx, ry, rz, rw), time, child_frame, parent_frame)

    def create_serial_command(self, command):
        return "Module " + command

    def send_serial_command(self, command):
        try:
            self.serial_port.write(command.encode())
        except serial.SerialException as e:
            rospy.logerr("Linear module serial communication error: %s", e)

# 主程序
if __name__ == '__main__':
    vehicle = TrackVehicle()
    module = LinearModule()
    # 使用vehicle和module执行特定操作
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        module.publish_transforms()
        rate.sleep()

```

```python
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu, NavSatFix, LaserScan
from geometry_msgs.msg import Twist
import serial
import math
import tf

# 父类：AutoPaintingRobot
class AutoPaintingRobot:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('auto_painting_robot')
        # 订阅激光雷达数据
        self.lidar_sub = rospy.Subscriber('/lidar', LaserScan, self.lidar_callback)

    # 激光雷达回调函数
    def lidar_callback(self, data):
        # 处理激光雷达数据
        self.target_tree_position = self.detect_tree_from_lidar(data)

    # 解析激光雷达数据以找到树木
    def detect_tree_from_lidar(self, data):
        # 设定距离阈值
        distance_threshold = 3.0
        # 找到最近的对象
        closest_distance = float('inf')
        closest_angle = None
        for angle in range(len(data.ranges)):
            distance = data.ranges[angle]
            if distance < closest_distance:
                closest_distance = distance
                closest_angle = angle
        # 检查是否有对象在阈值内
        if closest_distance < distance_threshold:
            return (closest_distance, closest_angle)
        else:
            return None

    # 生成串口指令（需要在子类中实现）
    def create_serial_command(self, command):
        raise NotImplementedError("Must be implemented in subclass")

    # 发送串口指令（需要在子类中实现）
    def send_serial_command(self, command):
        raise NotImplementedError("Must be implemented in subclass")
    
    def calculate_turn_angle(self, tree_position):
    # ...原有的角度计算逻辑...
    # 计算到树的距离（根据需要实现）

# 子类：TrackVehicle
class TrackVehicle(AutoPaintingRobot):
    def __init__(self):
        super().__init__()
        # 订阅IMU数据
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
        # 初始化串口通信
        self.serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

    # IMU数据回调函数
    def imu_callback(self, data):
        # 更新当前朝向
        self.current_orientation = self.get_orientation_from_imu(data)

    # 从IMU数据中获取朝向
    def get_orientation_from_imu(self, data):
        quaternion = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        euler = tf.transformations.euler_from_quaternion(quaternion)
        return euler

    # 创建控制履带车的串口指令
    def create_serial_command(self, command):
        return "Vehicle " + command

    # 发送控制履带车的串口指令
    def send_serial_command(self, command):
        try:
            self.serial_port.write(command.encode())
        except serial.SerialException as e:
            rospy.logerr("Track vehicle serial communication error: %s", e)

    def calculate_turn_angle(self, tree_position):
    # ...原有的角度计算逻辑...
    # 计算到树的距离（根据需要实现）

# 子类：LinearModule
class LinearModule(AutoPaintingRobot):
    def __init__(self):
        super().__init__()
        # 初始化TF广播器
        self.tf_broadcaster = tf.TransformBroadcaster()
        # 初始化串口通信
        self.serial_port = serial.Serial('/dev/ttyUSB1', 9600, timeout=1)

    # 发布TF变换
    def publish_transforms(self):
        current_time = rospy.Time.now()
        # 发布两个TF变换
        self.send_transform(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, current_time, "/lidar_frame", "/module_frame")
        self.send_transform(0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, current_time, "/lidar_frame", "/spray_claw_frame")

    # 发送单个TF变换
    def send_transform(self, tx, ty, tz, rx, ry, rz, rw, time, parent_frame, child_frame):
        self.tf_broadcaster.sendTransform((tx, ty, tz), (rx, ry, rz, rw), time, child_frame, parent_frame)

    # 创建控制直线模组的串口指令
    def create_serial_command(self, command):
        return "Module " + command

    # 发送控制直线模组的串口指令
    def send_serial_command(self, command):
        try:
            self.serial_port.write(command.encode())
        except serial.SerialException as e:
            rospy.logerr("Linear module serial communication error: %s", e)
           def calculate_turn_angle(self, tree_position):
    # ...原有的角度计算逻辑...
    # 计算到树的距离（根据需要实现）

# 主程序
if __name__ == '__main__':
    # 创建履带车和直线模组的实例
    vehicle = TrackVehicle()
    module = LinearModule()
    # 设置循环频率
    rate = rospy.Rate(10)  # 10 Hz
    # 主循环
    while not rospy.is_shutdown():
        
        # 对于履带车
        if vehicle.state == RobotState.NAVIGATING:
            vehicle.navigate_to_tree()
        elif vehicle.state == RobotState.SPRAYING:
            vehicle.spray_tree()
        # 对于直线模组
        if module.state == RobotState.NAVIGATING:
            module.navigate_to_tree()
        elif module.state == RobotState.SPRAYING:
            module.spray_tree()
        
        # 发布TF变换
        module.publish_transforms()
        # 维持循环频率
        rate.sleep()

```

