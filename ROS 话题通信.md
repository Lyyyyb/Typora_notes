# ROS 话题通信

## 基本特点

- 一个ROS节点网络中，可以同时存在多个话题
- 一个话题可以有多个发布者，也可以有多个订阅者
- 一个节点可以对多个话题进行订阅，也可以发布多个话题
- 不同的传感器消息通常会拥有各自独立的话题名称，每个话题只有一个发布者
- 机器人速度指令话题通常会有多个发布者，但是同一时间只能有一个发言人

在ROS（Robot Operating System，机器人操作系统）中，话题（Topic）通信是一种核心的通信机制，它允许节点之间以发布/订阅（Publisher/Subscriber）的方式交换消息。这种机制是基于异步消息传递的，适用于不同节点间的数据流动，例如传感器数据的共享或命令的下发。

## ROS话题通信的基本概念

1. **发布者（Publisher）**：一个节点可以发布（发送）消息到一个特定的话题。
2. **订阅者（Subscriber）**：一个节点可以订阅（接收）特定话题上的消息。
3. **消息（Message）**：通过话题传递的数据结构，定义了交换的数据类型。
4. **话题（Topic）**：一个命名的通道，发布者和订阅者通过它交换消息。

## 消息Message

在ROS（Robot Operating System，机器人操作系统）中，消息（Message）是节点间通信的基本单元。消息用于发布/订阅（Publisher/Subscriber）和服务（Service）模型中，允许节点间传递结构化数据。

### ROS中的Message（消息）

1. **结构化数据**：每个消息都是一种结构化数据类型，它定义了可以在节点之间传递的数据格式。
2. **预定义和自定义消息**：ROS提供了多种标准消息类型，如 `std_msgs`rate.sleep(); // 休眠以保持循环频率
    }
3. `/String`、`geometry_msgs/Twist` 等。用户也可以根据需要定义自己的消息类型。
4. **消息文件**：自定义消息是通过创建 `.msg` 文件来定义的，这些文件描述了消息的数据结构。
4. **编译生成**：在编译时，这些 `.msg` 文件会被转换成相应语言的源代码，例如C++或Python的类。

### 实例：标准消息和自定义消息

#### 使用标准消息

假设一个节点想要发布一个字符串消息：

```cpp
#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "talker");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::String>("chatter", 1000);

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        std_msgs::String msg;
        msg.data = "Hello ROS!";

        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
```

在这个例子中，我们使用了 `std_msgs/String`，这是ROS标准消息之一。我们创建了一个字符串消息并将其发布到名为 "chatter" 的话题。

#### 定义自定义消息

如果标准消息不满足需求，可以定义自定义消息。例如，定义一个名为 `Velocity` 的消息，包含线速度和角速度：

1. **创建消息文件** (例如 `Velocity.msg`):
   ```
   float64 linear
   float64 angular
   ```

2. **修改 `CMakeLists.txt` 和 `package.xml`**：
   - 在 `CMakeLists.txt` 中添加对新消息文件的引用。
   - 在 `package.xml` 中添加对消息生成依赖的引用。

3. **编译包**：编译后，ROS将生成相应的源代码。

4. **使用自定义消息**：
   ```cpp
   #include "your_package_name/Velocity.h" // 包含自定义消息头文件
   
   // 然后就可以像使用标准消息一样使用自定义消息了
   your_package_name::Velocity msg;
   msg.linear = 1.0;
   msg.angular = 0.5;
   ```

### 消息的重要性

ROS中的消息是实现模块化和灵活性的关键。通过定义清晰的数据结构，不同的节点可以独立开发和测试，只要它们遵循相同的消息接口。这种机制使得在机器人系统中集成不同的传感器、执行器和算法变得更加容易和可靠。

## 话题通信的工作流程

- **定义消息类型**：首先，定义消息的数据结构。ROS提供了许多标准消息类型，也允许创建自定义消息。
- **发布消息**：发布者节点向指定的话题发送消息。
- **订阅消息**：订阅者节点监听指定的话题，接收并处理消息。
- **消息传递**：ROS Master协调发布者和订阅者之间的连接，确保消息正确传递。

## 实例：发布者和订阅者节点

#### 发布者节点

发布者节点创建一个发布者对象，定期向特定话题发送消息。

```cpp
#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "talker");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::String>("chatter", 1000);
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        std_msgs::String msg;
        msg.data = "hello world";

        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
```

这个代码创建了一个名为 "talker" 的节点，它发布 `std_msgs/String` 类型的消息到 "chatter" 话题。`nh.advertise` 创建了一个发布者对象，`pub.publish(msg)` 发送消息。

#### 订阅者节点

订阅者节点创建一个订阅者对象，监听话题并处理接收到的消息。

```cpp
#include <ros/ros.h>
#include <std_msgs/String.h>

void chatterCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "listener");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("chatter", 1000, chatterCallback);
    ros::spin();

    return 0;
}
```

这个代码创建了一个名为 "listener" 的节点，它订阅 "chatter" 话题。`nh.subscribe` 创建了一个订阅者对象，`chatterCallback` 是接收消息时调用的回调函数。

### 话题通信的特点

- **解耦**：发布者和订阅者不需要知道彼此的存在。
- **灵活性**：可以有多个发布者和订阅者互动在同一个话题上。
- **易于扩展**：可以轻松添加新的发布者和订阅者。

话题通信在ROS中广泛应用于多种场景，如传感器数据传输、状态信息共享和控制指令的下发。通过这种机制，ROS节点可以高效地交换大量数据，同时保持系统的模块化和灵活性。





## 常见错误

```
terminate called after throwing an instance of 'ros::InvalidNameException'
  what():  Character [ ] at element [3] is not valid in Graph Resource Name [Let go].  Valid characters are a-z, A-Z, 0-9, / and _.
已放弃 (核心已转储)
```

这个错误信息是来自于ROS（Robot Operating System）的一个常见问题，它指出你试图使用了一个无效的名称。在ROS中，某些实体（比如话题、服务等）的名称必须遵守特定的命名规则。这个错误是由于你试图使用了一个不符合这些规则的名称。

错误信息 `"Character [ ] at element [3] is not valid in Graph Resource Name [Let go]. Valid characters are a-z, A-Z, 0-9, / and _."` 指出问题出在尝试使用的名字 `"Let go"` 上。在这个名字中，存在一个或多个空格（" "），而空格不是有效的字符。ROS的图资源名称只允许包含小写字母（a-z）、大写字母（A-Z）、数字（0-9）、斜杠（/）和下划线（_）。

### 解决方法

确保你使用的名称符合ROS的命名规则。如果你的名称中包含空格或其他无效字符，请将它们替换为下划线或其他有效字符。例如，你可以将 "Let go" 改为 "Let_go" 或其他类似的形式。

**修改示例**：

如果你在代码中定义了一个话题、服务或参数名称含有空格，如：

```cpp
ros::Publisher pub = nh.advertise<std_msgs::String>("Let go", 1000);
```

你应该将其改为：

```cpp
ros::Publisher pub = nh.advertise<std_msgs::String>("Let_go", 1000);
```

修改后，名称应该符合ROS的命名规则，从而避免 `InvalidNameException` 异常。