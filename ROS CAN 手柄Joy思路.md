# ROS CAN 手柄Joy思路

## 大致流程

- ROS上位机装载CAN报文 ，并作为Publisher将CAN报文发布到“sent_messages”话题。
- topic_to_socketcan_node节点监听“sent_messages”话题，是否有消息发布，如果有，则会将发布的ROS消息转换为CAN报文，并发布到CAN总线上。
- 驱动器根据CAN ID接收CAN报文，并向ROS上位机返回一个应答CAN报文。
- socketcan_to_topic_node节点监听CAN总线，是否有CAN报文发布，如果有，则接收CAN报文，并将CAN报文转换为ROS消息然后发布到“received_messages”话题。
- ROS上位机订阅“received_messages”话题，通过回调函数处理接收到的CAN报文。



首先，我们从游戏手柄获取摇杆值，其范围通常在-1到1之间，而车辆的最大速度默认为1m/s。根据需求，我们可以使用比例系数来调整速度范围。

接着，我们利用ROS订阅名为"joy"的手柄消息，并将其转换成/cmd_vel格式，以适应履带车所需的速度消息格式。

一旦获得/cmd_vel的线速度和角速度值，我们的下一步是将这些值转换成履带车实际控制的速度大小，并将其发布到CAN总线上。

在此过程中，我们考虑了车辆可能的运动模式，如直线行驶、原地转弯以及行驶转弯。为了实现精确控制，我们利用车辆的速度和转向角来计算其位置和方向。

针对两轮差速小车，我们采用了运动学模型。该模型通过计算左右两轮的线速度来确定车辆的速度 \( v \) 和角速度 \( \omega \) 。具体计算公式如下：

1. 左轮速度 \( v_l \) 的计算：\[$ v_l = v - \frac{{\omega \cdot L}}{2} $\]
2. 右轮速度 \( v_r \) 的计算：\[ $v_r = v + \frac{{\omega \cdot L}}{2}$ \]

通过这些公式，我们可以根据车辆的期望速度和角速度来计算每个驱动轮的速度，从而实现对车辆的精确控制。这种编程思路使得程序更具条理，逻辑更为清晰，提高了代码的可读性和可维护性。





- 手柄摇杆的值范围是-1到1，默认能发布的速度最大值为1m/s，实际使用过程中可根据需要添加比例系数来增加速度的范围

- 通过ros订阅游戏手柄的消息/joy，然后转换成履带车需要的速度消息的格式/cmd_vel

- 订阅手柄话题joy
- 将手柄摇杆轴拨动时值的输出赋值给“cmd_vel”上的线速度和角速度

- 再将cmd_vel发布的线速度和角速度转换成履带车实际控制的速度大小，并发布到CAN总线上

- 直线形式
- 原地转弯
- 行驶转弯
- 左右两轮速度计算公式如下

- 运动学模型使用车辆的速度和转向角来计算车辆的位置和方向。对于两轮差速小车，其速度可以用左右两轮的线速度表示。

- 假设左右两轮的线速度分别为 \( $v_l$ \) 和 \( $v_r$ \)，轮距为 \( $L$ \)（即两个驱动轮之间的距离）。车辆的速度 \($ v$ \) 和角速度 \( $\omega$ \) 可以通过以下公式计算：

- \[ $v = \frac{{v_l + v_r}}{2}$ \]

- \[$ \omega = \frac{{v_r - v_l}}{L}$ \]

- 其中，\( v \) 是车辆的线速度，\( $\omega$ \) 是车辆的角速度。

- 左右两轮的速度计算方法如下：

1. 左轮速度 \( v_l \) 的计算：
\[$ v_l = v - \frac{{\omega \cdot L}}{2} $\]

2. 右轮速度 \( v_r \) 的计算：
\[ $v_r = v + \frac{{\omega \cdot L}}{2}$ \]

通过这些公式，可以根据车辆的期望速度和角速度来计算每个驱动轮的速度，从而实现对车辆的控制。

```C++
void TeleopTurtle::callback(const sensor_msgs::Joy::ConstPtr &joy)
{
    geometry_msgs::Twist vel;
    // 将手柄摇杆轴拨动时值的输出赋值给乌龟的线速度和角速度
    vel.linear.x = joy->axes[axis_linear];
    vel.angular.z = joy->axes[axis_angular];
    ROS_INFO("当前线速度为:%.3lf ; 角速度为:%.3lf", vel.linear.x, vel.angular.z);
    pub.publish(vel);
}
```

```c++
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <can_msgs/Frame.h>

// 回调函数处理手柄消息
void joyCallback(const sensor_msgs::Joy::ConstPtr &msg, ros::Publisher &pub)
{
    // 根据手柄消息计算线速度和角速度
    double linear_vel = msg->axes[1]; // 假设手柄摇杆的第二个轴用于控制前后运动
    double angular_vel = msg->axes[0]; // 假设手柄摇杆的第一个轴用于控制旋转运动

    // 根据线速度和角速度计算左右轮的速度
    double L = 0.5; // 轮距
    double v_l, v_r;

    // 直线形式
    if (fabs(linear_vel) > 0.1 && fabs(angular_vel) < 0.1) {
        v_l = linear_vel;
        v_r = linear_vel;
    }
    // 原地转弯
    else if (fabs(linear_vel) < 0.1 && fabs(angular_vel) > 0.1) {
        v_l = -angular_vel * L / 2;
        v_r = angular_vel * L / 2;
    }
    // 行驶转弯
    else {
        double v = linear_vel;
        double omega = angular_vel;
        v_l = v - omega * L / 2;
        v_r = v + omega * L / 2;
    }

    // 检查左右轮速度范围是否合理，如果超出范围则进行修正
    const double max_speed = 1.0; // 最大速度
    const double min_speed = -1.0; // 最小速度
    v_l = std::min(std::max(v_l, min_speed), max_speed);
    v_r = std::min(std::max(v_r, min_speed), max_speed);

    // 创建CAN消息对象并填充数据
    can_msgs::Frame can_frame_msg;
    can_frame_msg.id = 0x521; // 假设CAN消息的ID为0x521
    can_frame_msg.dlc = 8;
    can_frame_msg.data[0] = static_cast<uint8_t>(v_l); // 左轮速度
    can_frame_msg.data[1] = static_cast<uint8_t>(v_r); // 右轮速度
    // 其他数据段根据实际需要填充

    // 发布CAN消息
    if(pub.getNumSubscribers() > 0) {
        pub.publish(can_frame_msg);
    } else {
        ROS_WARN("No subscribers to the CAN message, message not published");
    }
}

// 回调函数处理CAN接收消息
void canCallback(const can_msgs::Frame::ConstPtr &msg)
{
    // 在这里处理接收到的CAN消息
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "joy_to_cmd_vel_node");
    ros::NodeHandle nh;

    // 创建用于发布CAN消息的发布者
    ros::Publisher pub = nh.advertise
    <can_msgs::Frame>("sent_messages", 100);

    // 创建用于订阅手柄消息的订阅者
    ros::Subscriber sub_joy = nh.subscribe("joy", 10, boost::bind(joyCallback, _1, boost::ref(pub)));

    // 创建用于订阅CAN接收消息的订阅者
    ros::Subscriber sub_can = nh.subscribe("received_messages", 10, canCallback);

    // 循环等待回调函数
    ros::spin();

    return 0;
}



```

```C++
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <can_msgs/Frame.h>

class WheelController {
private:
    ros::Publisher pub_;
    double max_speed_;
    double min_speed_;
    double wheel_base_;

public:
    WheelController(ros::NodeHandle& nh, const std::string& topic_name, double max_speed, double min_speed, double wheel_base)
        : max_speed_(max_speed), min_speed_(min_speed), wheel_base_(wheel_base) {
        pub_ = nh.advertise
        <can_msgs::Frame>(topic_name, 100);
    }

    void publishSpeed(double linear_vel, double angular_vel) {
        // 根据线速度和角速度计算左右轮的速度
        double v_l, v_r;

        // 直线形式
        if (fabs(linear_vel) > 0.1 && fabs(angular_vel) < 0.1) {
            v_l = linear_vel;
            v_r = linear_vel;
        }
        // 原地转弯
        else if (fabs(linear_vel) < 0.1 && fabs(angular_vel) > 0.1) {
            v_l = -angular_vel * wheel_base_ / 2;
            v_r = angular_vel * wheel_base_ / 2;
        }
        // 行驶转弯
        else {
            double v = linear_vel;
            double omega = angular_vel;
            v_l = v - omega * wheel_base_ / 2;
            v_r = v + omega * wheel_base_ / 2;
        }

        // 检查左右轮速度范围是否合理，如果超出范围则进行修正
        v_l = std::min(std::max(v_l, min_speed_), max_speed_);
        v_r = std::min(std::max(v_r, min_speed_), max_speed_);

        // 创建CAN消息对象并填充数据
        can_msgs::Frame can_frame_msg;
        can_frame_msg.id = 0x521; // 假设CAN消息的ID为0x521
        can_frame_msg.dlc = 8;
        can_frame_msg.data[0] = static_cast<uint8_t>(v_l); // 左轮速度
        can_frame_msg.data[1] = static_cast<uint8_t>(v_r); // 右轮速度
        // 其他数据段根据实际需要填充

        // 发布CAN消息
        if (pub_.getNumSubscribers() > 0) {
            pub_.publish(can_frame_msg);
        } else {
            ROS_WARN("No subscribers to the CAN message, message not published");
        }
    }
};

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg, WheelController& left_wheel, WheelController& right_wheel) {
    // 假设手柄摇杆的第二个轴用于控制前后运动
    double linear_vel = msg->axes[1];
    // 假设手柄摇杆的第一个轴用于控制旋转运动
    double angular_vel = msg->axes[0];

    left_wheel.publishSpeed(linear_vel, angular_vel);
    right_wheel.publishSpeed(linear_vel, angular_vel);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "joy_to_cmd_vel_node");
    ros::NodeHandle nh;

    // 创建左轮和右轮的控制对象
    WheelController left_wheel(nh, "left_wheel_messages", 1.0, -1.0, 0.5);
    WheelController right_wheel(nh, "right_wheel_messages", 1.0, -1.0, 0.5);

    // 创建用于订阅手柄消息的订阅者，并绑定到回调函数
    ros::Subscriber sub_joy = nh.subscribe("joy", 10, boost::bind(joyCallback, _1, boost::ref(left_wheel), boost::ref(right_wheel)));

    ros::spin();

    return 0;
}

```

```C++
#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from can_msgs.msg import Frame

class WheelController:
    def __init__(self, topic_name, max_speed, min_speed, wheel_base):
        self.pub = rospy.Publisher(topic_name, Frame, queue_size=100)
        self.max_speed = max_speed
        self.min_speed = min_speed
        self.wheel_base = wheel_base

    def publish_speed(self, linear_vel, angular_vel):
        # 根据线速度和角速度计算左右轮的速度
        v_l, v_r = self.calculate_wheel_speeds(linear_vel, angular_vel)

        # 创建CAN消息对象并填充数据
        can_frame_msg = Frame()
        can_frame_msg.id = 0x521  # 假设CAN消息的ID为0x521
        can_frame_msg.dlc = 8
        can_frame_msg.data = [int(v_l), int(v_r), 0, 0, 0, 0, 0, 0]  # 左轮速度和右轮速度
        # 其他数据段根据实际需要填充

        # 发布CAN消息
        if self.pub.get_num_connections() > 0:
            self.pub.publish(can_frame_msg)
        else:
            rospy.logwarn("No subscribers to the CAN message, message not published")

    def calculate_wheel_speeds(self, linear_vel, angular_vel):
        # 根据线速度和角速度计算左右轮的速度
        v_l, v_r = 0, 0

        # 直线形式
        if abs(linear_vel) > 0.1 and abs(angular_vel) < 0.1:
            v_l = linear_vel
            v_r = linear_vel
        # 原地转弯
        elif abs(linear_vel) < 0.1 and abs(angular_vel) > 0.1:
            v_l = -angular_vel * self.wheel_base / 2
            v_r = angular_vel * self.wheel_base / 2
        # 行驶转弯
        else:
            v = linear_vel
            omega = angular_vel
            v_l = v - omega * self.wheel_base / 2
            v_r = v + omega * self.wheel_base / 2

        # 检查左右轮速度范围是否合理，如果超出范围则进行修正
        v_l = max(min(v_l, self.max_speed), self.min_speed)
        v_r = max(min(v_r, self.max_speed), self.min_speed)

        return v_l, v_r

def joy_callback(msg, left_wheel, right_wheel):
    # 假设手柄摇杆的第二个轴用于控制前后运动
    linear_vel = msg.axes[1]
    # 假设手柄摇杆的第一个轴用于控制旋转运动
    angular_vel = msg.axes[0]

    left_wheel.publish_speed(linear_vel, angular_vel)
    right_wheel.publish_speed(linear_vel, angular_vel)

def main():
    rospy.init_node('joy_to_cmd_vel_node')

    # 获取参数
    left_topic_name = rospy.get_param("~left_topic_name", "left_wheel_messages")
    right_topic_name = rospy.get_param("~right_topic_name", "right_wheel_messages")
    max_speed = rospy.get_param("~max_speed", 1.0)
    min_speed = rospy.get_param("~min_speed", -1.0)
    wheel_base = rospy.get_param("~wheel_base", 0.5)

    # 创建左轮和右轮的控制对象
    left_wheel = WheelController(left_topic_name, max_speed, min_speed, wheel_base)
    right_wheel = WheelController(right_topic_name, max_speed, min_speed, wheel_base)

    # 创建用于订阅手柄消息的订阅者，并绑定到回调函数
    rospy.Subscriber("joy", Joy, lambda msg: joy_callback(msg, left_wheel, right_wheel), queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    main()


```

