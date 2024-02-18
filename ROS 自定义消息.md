# ROS 自定义消息

## 生成自定义消息步骤

- 创建新软件包，依赖项message_generation\message_runtime
- 软件包添加msg目录，新建自定义消息文件，以.msg结尾
- 在CMakeLists.txt中，将新建的.msg文件加入add_message_files()
- 在CMakeLists.txt中，去掉generate_messages()注释符号，将message_runtime加入catkin_package()的CATKIN_DEPENDS
- 在package.xml中，将message_generation\message_runtime加入<build_depend>和<exec_depend>
- 编译软件包，生成新的自定义消息类型

## 具体操作

在ROS（Robot Operating System）中，如果现有的消息类型无法满足你的需求，你可以创建自定义消息。这里是生成自定义消息的详细步骤：

### 步骤 1: 定义自定义消息文件

1. 在你的ROS包的 `msg` 目录中创建一个新的 `.msg` 文件。如果目录不存在，请先创建它。
   ```bash
   cd <your_package_directory>
   mkdir msg
   ```
2. 使用文本编辑器创建消息文件，例如 `MyCustomMessage.msg`。
3. 在 `.msg` 文件中定义消息的内容。每行定义一个数据字段，格式为 `<数据类型> <字段名>`。例如：
   ```
   string first_name
   string last_name
   uint8 age
   float32 score
   ```

### 步骤 2: 修改 `package.xml`

1. 确保你的 `package.xml` 文件包含了 `message_generation` 和 `message_runtime` 依赖。这是生成和运行自定义消息所必需的。
   ```xml
   <build_depend>message_generation</build_depend>
   <exec_depend>message_runtime</exec_depend>
   ```
2. 如果你的消息依赖于其他包的消息，则需要添加这些包作为依赖。

### 步骤 3: 修改 `CMakeLists.txt`

1. 找到 `find_package` 部分，确保 `message_generation` 和你的自定义消息依赖的任何其他消息包都被包括在 `COMPONENTS` 列表中。
   ```cmake
   find_package(catkin REQUIRED COMPONENTS
     ...
     message_generation
     ...
   )
   ```
2. 在 `add_message_files` 函数中列出你的 `.msg` 文件。
   ```cmake
   add_message_files(
     FILES
     MyCustomMessage.msg
     ...
   )
   ```
3. 调用 `generate_messages` 函数，并确保包括所有消息依赖。
   ```cmake
   generate_messages(
     DEPENDENCIES
     std_msgs  # 或其他依赖的消息类型
     ...
   )
   ```
4. 在 `catkin_package` 函数中添加 `message_runtime` 到 `CATKIN_DEPENDS` 参数，确保其他包在构建时能找到你的消息。
   ```cmake
   catkin_package(
     ...
     CATKIN_DEPENDS message_runtime
     ...
   )
   ```

### 步骤 4: 构建你的消息

1. 从你的工作空间的根目录运行 `catkin_make` 来生成自定义消息。
   ```bash
   cd ~/catkin_ws
   catkin_make
   ```
2. 记得在尝试使用新消息之前，源一下新的环境设置。
   ```bash
   source devel/setup.bash
   ```

### 步骤 5: 使用自定义消息

在你的节点代码中，你现在可以像使用标准消息一样使用自定义消息。确保在代码中包含正确的头文件：
```cpp
#include "<your_package_name>/MyCustomMessage.h"
```
或在Python中：
```python
from your_package_name.msg import MyCustomMessage
```

现在你的自定义消息已经准备好使用了，可以用它来发布和订阅数据。

## C++实现

### 实现步骤

- 在节点代码中，先include新消息类型的头文件
- 在发布或订阅话题的时候，将话题中的消息类型设置为新的消息类型
- 按照新的消息结构，对消息包进行赋值发送或读取解析
- 在CMakeList.txt文件的find_package()中，添加新消息包作为依赖项
- 在节点的编译规则中，添加一条add_dependencied(),将新消息软件包名称_generate_messages_cpp作为依赖项
- 在package.xml中，将新消息包添加到<build_depend>和<exec_depend>中去
- 运行catkin_make重新编译

在ROS（Robot Operating System）中，使用自定义消息类型可以让您在节点间传递特定于您应用程序的数据结构。以下是在C++节点中应用自定义消息类型的详细步骤：

### 1. 在节点代码中包含自定义消息类型的头文件

首先，在您的C++源文件中包含自定义消息的头文件。假设您的包名为 `my_robot_msgs`，消息名为 `MyCustomMsg`。

```cpp
#include "my_robot_msgs/MyCustomMsg.h"
```

### 2. 发布或订阅自定义消息

当创建发布者（publisher）或订阅者（subscriber）时，使用自定义消息类型。

#### 发布自定义消息：

```cpp
ros::Publisher pub = n.advertise<my_robot_msgs::MyCustomMsg>("topic_name", 1000);
my_robot_msgs::MyCustomMsg msg;
msg.data = ...; // 根据消息结构填充数据
pub.publish(msg);
```

#### 订阅自定义消息：

```cpp
void callback(const my_robot_msgs::MyCustomMsg::ConstPtr& msg) {
    // 使用msg中的数据
}

ros::Subscriber sub = n.subscribe("topic_name", 1000, callback);
```

### 3. 修改 `CMakeLists.txt` 和 `package.xml`

确保 `CMakeLists.txt` 和 `package.xml` 文件正确配置了对自定义消息的支持。

#### CMakeLists.txt:

- 在 `find_package()` 中添加依赖：

  ```cmake
  find_package(catkin REQUIRED COMPONENTS
    ...
    my_robot_msgs
    ...
  )
  ```

- 在编译规则中添加依赖：

  ```cmake
  add_dependencies(your_node ${catkin_EXPORTED_TARGETS} my_robot_msgs_generate_messages_cpp)
  ```

- 确保你的项目链接到正确的库：

  ```cmake
  target_link_libraries(your_node
    ${catkin_LIBRARIES}
  )
  ```

#### package.xml:

- 添加build和run依赖：

  ```xml
  <build_depend>my_robot_msgs</build_depend>
  <exec_depend>my_robot_msgs</exec_depend>
  ```

### 4. 重新编译

使用 `catkin_make` 重新编译您的工作空间：

```bash
cd ~/catkin_ws  # 或您的工作空间的路径
catkin_make
```

完成这些步骤后，您的C++节点应该能够正确地发布和订阅自定义消息类型。

### 注意事项

- 确保您的自定义消息包已经正确构建，且`.msg`文件中的消息结构正确定义。
- 重新编译工作空间后，您可能需要重新`source`工作空间的`setup.bash`文件。

## Python实现

### 实现步骤

- 在节点代码中，先import新定义的消息类型
- 在发布或订阅话题的时候，将话题中的消息类型设置为新的消息类型
- 按照新的消息结构，对消息包进行赋值发送或读取解析
- 在CMakeLists.txt文件的find_package()中，添加新消息包名称作为依赖项
- 在Package.xml中，将新消息包添加到<build_depend>和<exec_depend>中去
- 重新编译，确保软件包进入到ROS的包列表中

在ROS（Robot Operating System）中，使用Python创建和应用自定义消息类型的过程涉及几个关键步骤。这些步骤确保你的自定义消息能够被ROS识别并在不同的节点之间进行传递。以下是详细的步骤：

### 1. 导入自定义消息类型

在Python节点代码的开始处，导入你的自定义消息类型。假设你的包名为 `my_robot_msgs`，消息名为 `MyCustomMsg`。

```python
#!/usr/bin/env python
# 或 #!/usr/bin/env python3，取决于你的ROS版本和Python版本

from my_robot_msgs.msg import MyCustomMsg
```

### 2. 发布或订阅自定义消息

在你的节点中，创建一个发布者或订阅者，使用你的自定义消息类型。

#### 发布自定义消息：

```python
import rospy
from my_robot_msgs.msg import MyCustomMsg

pub = rospy.Publisher('my_custom_topic', MyCustomMsg, queue_size=10)
rospy.init_node('my_custom_publisher')

msg = MyCustomMsg()
msg.data = ...  # 根据消息结构填充数据
pub.publish(msg)
```

#### 订阅自定义消息：

```python
def callback(msg):
    # 使用 msg 中的数据
    pass

rospy.init_node('my_custom_subscriber')
sub = rospy.Subscriber('my_custom_topic', MyCustomMsg, callback)
rospy.spin()
```

### 3. 修改 CMakeLists.txt 和 package.xml

确保 `CMakeLists.txt` 和 `package.xml` 文件正确配置了对自定义消息的支持。

#### CMakeLists.txt:

- 在 `find_package()` 中添加依赖：

  ```cmake
  find_package(catkin REQUIRED COMPONENTS
    ...
    my_robot_msgs
    ...
  )
  ```

#### package.xml:

- 添加build和run依赖：

  ```xml
  <build_depend>my_robot_msgs</build_depend>
  <exec_depend>my_robot_msgs</exec_depend>
  ```

### 4. 重新编译

使用 `catkin_make` 重新编译你的工作空间：

```bash
cd ~/catkin_ws  # 或你的工作空间的路径
catkin_make
```

完成这些步骤后，Python节点应该能够正确地发布和订阅自定义消息类型。

### 注意事项

- 确保你的自定义消息包已经正确构建，且`.msg`文件中的消息结构正确定义。
- 重新编译工作空间后，你可能需要重新`source`工作空间的`setup.bash`文件。
- 这里假设你已经根据ROS标准创建并配置了自定义消息包。



## catkin构建过程

在ROS（Robot Operating System）中，当你创建一个自定义消息时，相应的头文件是通过一个自动化的构建过程生成的。这个过程涉及到ROS的构建系统（通常是`catkin`），它负责从你的`.msg`文件生成对应的源代码文件，包括C++和Python所需的头文件和模块。下面是这个过程的详细解释：

### 1. 创建自定义消息

假设你已经在你的ROS包中定义了一个自定义消息。例如，你有一个名为`MyCustomMessage.msg`的文件，位于你的包的`msg`目录中。

### 2. 修改CMakeLists.txt和package.xml文件

在你的ROS包中，你需要在`CMakeLists.txt`和`package.xml`文件中声明对消息生成的依赖。

- 在`package.xml`中，你需要添加对`message_generation`和`message_runtime`的依赖。
- 在`CMakeLists.txt`中，你需要包括`message_generation`作为组件，并调用`add_message_files`和`generate_messages`函数。

### 3. 构建过程

当你运行`catkin_make`或类似的构建命令时，ROS的构建系统会查找所有声明了的`.msg`文件，并为每个文件生成相应的源代码。这个过程包括：

- 为C++生成头文件（`.h`文件），通常位于你的工作空间的`devel/include/<package_name>/`目录下。
- 为Python生成模块，通常位于你的工作空间的`devel/lib/python2.7/dist-packages/<package_name>/msg/`目录下（Python 2.7路径，对于Python 3可能有所不同）。

### 4. 使用自定义消息

#### 在C++中：

你可以在C++源文件中直接包含生成的头文件，如下所示：

```cpp
#include "<package_name>/MyCustomMessage.h"
```

其中`<package_name>`是你的ROS包的名称，`MyCustomMessage.h`是为你的自定义消息生成的头文件。

#### 在Python中：

在Python脚本中，你可以导入自动生成的消息模块：

```python
from <package_name>.msg import MyCustomMessage
```

### 注意事项

- 确保在运行`catkin_make`之前正确设置了`CMakeLists.txt`和`package.xml`文件。
- 生成的头文件和模块的位置取决于你的ROS工作空间和构建系统配置。
- 在使用自定义消息之前，你需要先源（source）你的工作空间的`setup.bash`文件，以确保新生成的代码被包含在环境中。

这个自动化的过程简化了从`.msg`文件到可用于C++和Python代码的头文件和模块的转换，使得在ROS中处理自定义消息变得更加容易和高效。