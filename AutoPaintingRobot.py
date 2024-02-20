#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu, NavSatFix, LaserScan
from geometry_msgs.msg import Twist
import serial
import math
import tf
from tf import LookupException, ConnectivityException, ExtrapolationException
import tf.transformations

# 定义机器人的可能状态
class RobotState:
    NAVIGATING = 1  # 导航状态
    SPRAYING = 2    # 喷涂状态


# 状态机类，用于管理喷涂过程中的状态
class SprayingStateMachine:
    def __init__(self):
        self.state = 7  # 初始状态

    def update_state(self, new_state):
        # 更新状态机的状态
        self.state = new_state

STEP_DISTANCE = None

# 自动喷漆机器人的父类
class AutoPaintingRobot:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('auto_painting_robot')
        # 订阅激光雷达数据
        self.lidar_sub = rospy.Subscriber('/lidar', LaserScan, self.lidar_callback)
        # 设置初始状态为导航
        self.state = RobotState.NAVIGATING
        # 初始化状态机
        self.state_machine = SprayingStateMachine()
        # 初始化TF广播器
        self.tf_broadcaster = tf.TransformBroadcaster()
        # 初始化tf监听器
        self.listener = tf.TransformListener()
        #初始化雷达数据
        self.lidar_data = None
        
    # 激光雷达数据的回调函数
    def lidar_callback(self, data):
        # 更新激光雷达数据
        self.lidar_data = data
        # 处理激光雷达数据以便找到树木的位置
        self.target_tree_position = self.detect_tree_from_lidar(data)

    # 解析激光雷达数据以找到树木，需在子类中具体实现
    def detect_tree_from_lidar(self, data):
        raise NotImplementedError("Must be implemented in subclass")

    # 生成串口指令，需在子类中具体实现
    def create_serial_command(self, command):
        raise NotImplementedError("Must be implemented in subclass")

    # 发送串口指令，需在子类中具体实现
    def send_serial_command(self, command):
        raise NotImplementedError("Must be implemented in subclass")
    
    # 读取串口数据
    def read_serial_data(self):
        # 尝试读取串口数据，并处理可能的异常
        try:
            if self.serial_port.in_waiting > 0:
                data = self.serial_port.readline().decode().strip()
                if data:
                    return data
        except serial.SerialException as e:
            rospy.logerr("Error reading serial data: %s", e)
            # 可以在此处添加更多的错误处理逻辑，例如：
            # - 尝试重新初始化串口连接
            # - 将错误状态发送到一个监控系统
            # - 如果错误频繁发生，可以采取降级策略，比如减少读取频率等
        return "ERROR"

    #计算旋转角度，需在子类中具体实现
    def calculate_turn_angle(self, tree_position):
        raise NotImplementedError("Must be implemented in subclass")
    
        # 发布TF变换
    def publish_transforms(self):
        current_time = rospy.Time.now()
        # 发布两个TF变换
        self.send_transform(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, current_time, "/lidar_frame", "/module_frame")
        self.send_transform(0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, current_time, "/lidar_frame", "/spray_claw_frame")

    # 发送单个TF变换
    def send_transform(self, tx, ty, tz, rx, ry, rz, rw, time, parent_frame, child_frame):
        self.tf_broadcaster.sendTransform((tx, ty, tz), (rx, ry, rz, rw), time, child_frame, parent_frame)
    
    # 导航到树木的逻辑
    def navigate_to_tree(self):
        # 读取串口数据并根据数据内容更新状态
        serial_data = self.read_serial_data()
        if serial_data == "OK":
            # 正常逻辑
            self.state_machine.update_state(1)
            self.state = RobotState.SPRAYING
        elif serial_data == "ERROR":
            # 错误处理
            rospy.logwarn("Serial communication error during navigation")
            # 根据具体情况执行错误恢复逻辑，例如：
            # - 将机器人移动到安全位置
            # - 发送错误通知到控制中心
            # - 尝试重新建立串口连接

    # 喷涂树木的逻辑，需在子类中具体实现
    def spray_tree(self):
        # 喷树逻辑
        # ...
        # 假设喷树完成，切换回导航状态
        self.state = RobotState.NAVIGATING

# 履带车型机器人，继承自AutoPaintingRobot
class TrackVehicle(AutoPaintingRobot):
    def __init__(self):
        super().__init__()
        # 订阅IMU数据
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
        # 初始化串口通信
        self.serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        # 初始化存储当前位置、朝向和目标位置的变量
        self.current_position = None
        self.current_orientation = None
        self.target_tree_position = None
        
    def detect_tree_from_lidar(self, data):
        # 假设 data 提供了直接的 X, Y, Z 坐标信息
        # 这里是一个简化的示例，具体实现应根据实际数据格式进行
        x = data.x_coordinate  # 从 data 中获取 X 坐标
        y = data.y_coordinate  # 从 data 中获取 Y 坐标
        z = data.z_coordinate  # 从 data 中获取 Z 坐标

        # 将坐标赋值给 position
        position = (x, y, z)
        return position
    
    # IMU数据回调函数
    def imu_callback(self, data):

        # 更新激光雷达数据
        self.imu_data = data
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

    def calculate_turn_angle(self, tree_position, current_orientation):
        # 假设current_orientation为机器人朝向的欧拉角
        # 计算机器人当前朝向与树木之间的角度差
        angle_to_tree = math.atan2(tree_position.y, tree_position.x)
        turn_angle = angle_to_tree - current_orientation.yaw
        return turn_angle 
    
    def convert_to_spray_claw_frame(self, tree_position, listener):
        try:
            # 等待坐标系的转换关系
            self.listener.waitForTransform("/spray_claw_frame", "/lidar_frame", rospy.Time(0), rospy.Duration(4.0))
            # 进行坐标转换
            tree_position_spray_claw_frame = self.listener.transformPoint("/spray_claw_frame", tree_position)
            return tree_position_spray_claw_frame
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("TF转换异常: %s", e)
            return None
        
    def calculate_movement_for_spray(self, tree_position_spray_claw_frame):
        # 假设计算得到的距离是机器人与树木之间的直线距离
        movement_distance = math.sqrt(tree_position_spray_claw_frame.point.x ** 2 + tree_position_spray_claw_frame.point.y ** 2)
        return movement_distance

    # 实现具体的喷涂树木逻辑
    def spray_tree(self):
        serial_data = self.read_serial_data()

        if serial_data == "OK":
            if self.state_machine.state == 1:
                """
                使用三维雷达获取树的坐标及距离，IMU获取履带车的朝向计算出车要转动的角度，
                转换成控制指令，通过串口发送给下位机，调整车的朝向，并且需要考虑履带车和Y轴直线模组的长度，
                保证其在履带车转动过程中不会撞到树。
                """
                # 获取树的位置
                tree_position = self.detect_tree_from_lidar(self.lidar_data)  # 假设lidar_data是已获取的数据

                # 获取当前朝向
                current_orientation = self.get_orientation_from_imu(self.imu_data)  # 假设imu_data是已获取的数据

                # 计算转动角度
                turn_angle = self.calculate_turn_angle(tree_position, current_orientation)

                # 创建控制指令
                turn_command = self.create_serial_command(turn_angle)

                # 发送控制指令
                self.send_serial_command(turn_command)

                self.state_machine.update_state(2)

            elif self.state_machine.state == 3:
                """
                使用三维雷达获取树的坐标及距离，通过TF坐标转换成相对于喷爪末端的坐标和距离，
                计算出车要移动的距离，转换成控制指令，通过串口发送给下位机，喷爪闭合。

                使车前进合适距离
                """
                # 从激光雷达数据中获取树木的位置
                tree_position = self.detect_tree_from_lidar(self.lidar_data)

                # 将树木位置从激光雷达坐标系转换到喷涂装置（喷爪）的坐标系
                tree_position_spray_claw_frame = self.convert_to_spray_claw_frame(tree_position, self.tf_listener)

                # 检查是否成功获取了树木位置的转换坐标
                if tree_position_spray_claw_frame is not None:
                    # 计算机器人需要移动的距离，以便喷爪能够到达树木的位置
                    movement_distance = self.calculate_movement_for_spray(tree_position_spray_claw_frame)
                    
                # 发送移动指令，使机器人移动计算出的距离
                self.send_serial_command(movement_distance)

                self.state_machine.update_state(4)

            elif self.state_machine.state == 5:
                """
                使车后退合适距离
                """
                # 从激光雷达数据中获取树木的位置
                tree_position = self.detect_tree_from_lidar(self.lidar_data)

                # 将树木位置从激光雷达坐标系转换到喷涂装置（喷爪）的坐标系
                tree_position_spray_claw_frame = self.convert_to_spray_claw_frame(tree_position, self.tf_listener)

                # 检查是否成功获取了树木位置的转换坐标
                if tree_position_spray_claw_frame is not None:
                    # 计算机器人需要移动的距离，以便喷爪能够到达树木的位置
                    movement_distance = self.calculate_movement_for_spray(tree_position_spray_claw_frame)
                    
                # 发送移动指令，使机器人移动计算出的距离
                self.send_serial_command(movement_distance)

                self.state_machine.update_state(6)

        elif serial_data == "ERROR":
            # 错误处理逻辑
            rospy.logwarn("Serial communication error in TrackVehicle during spraying")
            # 可以在此处添加更多的错误处理逻辑，例如：
            # - 将机器人移动到安全位置
            # - 发送错误通知到控制中心
            # - 尝试重新建立串口连接
    
# 直线模组，继承自AutoPaintingRobot
class LinearModule(AutoPaintingRobot):
    def __init__(self):
        super().__init__()

        # 初始化串口通信
        self.serial_port = serial.Serial('/dev/ttyUSB1', 9600, timeout=1)



    # 创建控制直线模组的串口指令
    def create_serial_command(self, command):
        return "Module " + command

    # 发送控制直线模组的串口指令
    def send_serial_command(self, command):
        try:
            self.serial_port.write(command.encode())
        except serial.SerialException as e:
            rospy.logerr("Linear module serial communication error: %s", e)
    
    def convert_to_module_frame(self, tree_position, listener):
        # 假设listener是一个tf.TransformListener对象
        try:
            # 等待坐标系的转换关系
            self.listener.waitForTransform("/module_frame", "/lidar_frame", rospy.Time(0), rospy.Duration(4.0))
            # 进行坐标转换
            tree_position_module_frame = self.listener.transformPoint("/module_frame", tree_position)
            return tree_position_module_frame
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("TF转换异常: %s", e)
            return None
        
    def calculate_movement_for_module(self, tree_position_module_frame):
        # 假设直线模组的起点为(0,0,0)，计算滑台需要移动的距离
        movement_distance = tree_position_module_frame.point.x  # 根据实际情况可能需要调整
        return movement_distance
    
    def calculate_steps_for_motor(self, movement_distance, step_distance):
        # 计算步进电机需要运动的步数
        steps = int(movement_distance / step_distance)  # 可能需要进行取整操作
        return steps
    
    def open_spray_claw(self):
        # 发送指令以张开喷爪
        open_command = "OPEN_CLAW"
        self.send_serial_command(open_command)

    def close_spray_claw(self):
        # 发送指令以闭合喷爪
        close_command = "CLOSE_CLAW"
        self.send_serial_command(close_command)

    # 实现具体的喷涂树木逻辑
    def spray_tree(self):
        serial_data = self.read_serial_data()

        if serial_data == "OK":
            if self.state_machine.state == 2:
                """
                调整车的朝向后，使用三维雷达获取树的坐标，通过TF坐标转换成相对于X轴直线模组最左侧的坐标，
                通过坐标信息计算出X轴直线模组的滑台需要移动多少距离，进而得到X轴直线模组的步进电机需要运动的步数，
                转换成控制指令，通过串口发送给下位机。

                调整X轴丝杠，正对树，张开喷爪
                """
                # 检测树木的位置，使用来自激光雷达的数据
                tree_position = self.detect_tree_from_lidar(self.lidar_data)
                # 此方法应从激光雷达数据中识别树木的位置，并返回它的坐标

                # 将检测到的树木位置从激光雷达坐标系转换到模块坐标系
                tree_position_module_frame = self.convert_to_module_frame(tree_position, self.listener)
                # 这需要tf.TransformListener来转换坐标，从激光雷达坐标系到喷漆模块的坐标系

                # 检查转换后的坐标是否有效
                if tree_position_module_frame is not None:
                    # 如果有效，计算从当前位置到目标位置的移动距离
                    movement_distance = self.calculate_movement_for_module(tree_position_module_frame)
                    # 这个方法应计算出机器人需要移动多远以便其喷漆模块与树木位置对齐

                    # 根据移动距离计算出步进电机需要走的步数
                    steps = self.calculate_steps_for_motor(movement_distance, STEP_DISTANCE)
                    # STEP_DISTANCE 是每步的距离，用于从移动距离计算步数

                    # 发送串口指令以控制机器人移动相应的步数
                    self.send_serial_command(steps)
                    # 这个方法应发送控制指令到机器人，使其移动计算出的步数


                self.state_machine.update_state(3)

            elif self.state_machine.state == 4:
                """
                控制水泵喷漆，控制Z轴直线模组的滑台，使Y轴直线模组上下移动对树均匀喷漆。
                控制Y轴直线模组末端的喷爪张开

                喷爪闭合，控制Z轴丝杠，使Y轴丝杠上下运动，同时控制水泵喷水，喷涂结束，喷爪张开
                """
                # 发送指令以闭合喷涂装置的喷爪
                self.close_spray_claw()

                # 再次计算步进电机需要走的步数，基于新的移动距离
                steps = self.calculate_steps_for_motor(movement_distance, STEP_DISTANCE)

                # 发送串口指令，让机器人执行相应的步数移动
                self.send_serial_command(steps)

                self.state_machine.update_state(5)

            elif self.state_machine.state == 6:
                """
                喷爪闭合
                """
                self.close_spray_claw()
                self.state_machine.update_state(7)

        elif serial_data == "ERROR":
            # 错误处理逻辑
            rospy.logwarn("Serial communication error in LinearModule during spraying")
            # 可以在此处添加更多的错误处理逻辑
 

# 主程序
if __name__ == '__main__':
    # 创建履带车和直线模组的实例
    vehicle = TrackVehicle()
    module = LinearModule()
    # 设置循环频率
    rate = rospy.Rate(10)  # 10 Hz
    # 主循环
    while not rospy.is_shutdown():        
        # 发布TF变换
        module.publish_transforms()
        # 调用回调函数
        rospy.spin()
        # 对于履带车和直线模组，#根据机器人的状态执行相应的操作
        if module.state == RobotState.NAVIGATING:
            module.navigate_to_tree()
        elif vehicle.state == RobotState.SPRAYING:
            vehicle.spray_tree()
        elif module.state == RobotState.SPRAYING:
            module.spray_tree()
        else:
            pass        
        # 维持循环频率
        rate.sleep()
