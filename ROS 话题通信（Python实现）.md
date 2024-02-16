# ROS 话题通信（Python实现）

- 创建好软件包后，执行catkin_make编译，这里的编译并不是为了编译ros节点，使用Python语言编写的节点是可以直接运行的不需要编译，现在这个编译操作，只是为了让新建的这个ssr_pkg能够进入ROS的软件包列表，以便后边我们运行节点的时候，ROS能够在软件包列表中找到这个ssr_pkg.也就是说这个编译操作只需要在新建好软件包后，执行一次就行了，后面无论是新增节点还是对已有节点进行代码修改，都不需要再次进行编译。

>这段话描述的是在ROS（Robot Operating System）中使用Python编写节点时的编译过程。在ROS中，不同于C++节点，Python节点不需要编译成二进制文件即可运行。但即使是Python节点，使用`catkin_make`进行编译依然有其特定的用途和必要性。下面是对这段话的详细解释：
>
>### `catkin_make` 的作用
>
>当你在ROS中创建一个新的软件包（比如名为`ssr_pkg`的包）时，即使该包中只包含Python写的节点，执行`catkin_make`依然是一个重要步骤。这个步骤的作用包括：
>
>1. **将软件包添加到ROS环境中**：执行`catkin_make`会生成一些设置文件和脚本，这些文件让ROS环境知道新软件包的存在。这意味着在执行`catkin_make`之后，当你使用如`rosrun`或`roslaunch`命令时，ROS能够找到并正确运行这个新软件包中的节点。
>
>2. **生成必要的环境设置**：`catkin_make`还会生成或更新`devel`目录，其中包含了设置脚本（如`setup.bash`或`setup.zsh`），这些脚本用于将你的工作空间添加到ROS环境中。这样，当你激活你的工作空间后，ROS就能够访问并使用该工作空间中的所有软件包。
>
>### Python 节点的特点
>
>- **无需编译**：Python是一种解释型语言，Python写的ROS节点不需要像C++节点那样编译成二进制文件。你只需确保Python脚本文件具有执行权限（可以通过`chmod +x your_script.py`命令来设置）。
>
>- **源代码直接运行**：Python节点的源代码可以直接被ROS运行。这意味着对于Python写的节点，你可以做出更改并立即运行，无需重新编译。
>
>### 后续修改和新增节点
>
>- **新增Python节点**：如果你在已存在的软件包中新增Python节点，通常不需要再次执行`catkin_make`。只要确保新的Python脚本有正确的执行权限即可。
>
>- **修改Python节点**：对Python节点代码的修改也是即时生效的，无需重新编译。
>
>- **添加或修改C++节点**：如果在软件包中添加或修改C++节点，这时就需要重新执行`catkin_make`来编译这些改动。
>
>### 总结
>
>简而言之，对于只包含Python节点的ROS软件包，`catkin_make`主要用于初次将该软件包注册到ROS环境中。对于Python节点的后续修改或新增，通常不需要再次执行`catkin_make`，除非你也在处理C++节点或需要更新软件包配置。

## 思路

### 发布者

- 召唤ROS大管家rospy
- 向ROS大管家rospy申请初始化节点
- 告诉ROS大管家rospy需要发布的话题名称，并向他所要能够发送消息包的发布对象pub
- 开启一个while循环，不停的使用pub对象发布消息包

### 订阅者

- 召唤ROS大管家rospy
- 向ROS大管家rospy申请初始化节点
- 构建一个回调函数，用于处理从话题中接收到的消息包数据
- 告诉ROS大管家rospy需要订阅的话题名称，并设置接收消息包的回调函数
- 调用ROS的spin()函数

## 代码示例

在ROS（Robot Operating System）中，使用Python 3实现话题通信涉及到创建一个发布者（Publisher）和一个订阅者（Subscriber）。以下是用Python 3编写的简单的发布者和订阅者示例，以及相关的注释。

### 发布者节点 (Publisher Node)

这个示例是一个发布者节点的实现，它将定期向名为 "chatter" 的话题发布字符串消息。

```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def talker():
    # 创建一个Publisher，发布到 'chatter' 话题
    pub = rospy.Publisher('chatter', String, queue_size=10)
    # 初始化节点，节点名称为 'talker'
    rospy.init_node('talker', anonymous=True)
    # 设置发布频率为 10Hz
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        hello_str = "hello ROS %s" % rospy.get_time()  # 创建要发布的消息
        rospy.loginfo(hello_str)  # 在控制台输出日志信息
        pub.publish(hello_str)  # 发布消息
        rate.sleep()  # 按照前面设置的频率休眠

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```

在这个示例中，我们创建了一个名为`talker`的发布者节点，它定期向`chatter`话题发送`std_msgs/String`类型的消息。

### 订阅者节点 (Subscriber Node)

这个示例是一个订阅者节点的实现，它会订阅 "chatter" 话题并打印接收到的消息。

```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.data)  # 打印接收到的消息

def listener():
    # 初始化节点，节点名称为 'listener'
    rospy.init_node('listener', anonymous=True)
    # 创建一个Subscriber，订阅 'chatter' 话题
    rospy.Subscriber("chatter", String, callback)
    # 循环等待回调函数
    rospy.spin()

if __name__ == '__main__':
    listener()
```

在这个示例中，我们创建了一个名为`listener`的订阅者节点，它订阅`chatter`话题，并使用`callback`函数来处理接收到的消息。每当`chatter`话题上有新消息时，`callback`函数被调用。

### 运行节点

1. 确保你的ROS环境已经安装并配置好了Python 3的支持。
2. 将这两个Python脚本保存到你的ROS包的`scripts`目录中。
3. 记得给脚本文件添加执行权限，例如通过运行`chmod +x talker.py listener.py`。
4. 使用`roscore`启动ROS主节点（如果还没有启动的话）。
5. 在不同的终端窗口中分别运行这两个节点，例如使用`rosrun your_package_name talker.py`和`rosrun your_package_name listener.py`命令。

这两个节点将通过`chatter`话题进行通信：发布者节点发送消息，订阅者节点接收并打印这些消息。

## shebang

在ROS（Robot Operating System）中，当使用Python编写节点时，文件的开头通常包含一个称为"shebang"的行。这行是一个特殊的注释，它告诉操作系统如何执行该文件。

### Shebang的作用

1. **指定解释器**：Shebang指定了运行脚本时应该使用的解释器。对于Python脚本，这通常是指向Python解释器的路径。

2. **直接运行**：有了shebang，你可以直接运行Python脚本而无需在命令行前面显式写出`python`。这在ROS中尤其有用，因为ROS可以直接运行节点而不需要指定解释器。

### 示例

一个典型的Python脚本shebang看起来像这样：

```python
#!/usr/bin/env python

# Python代码...
```

或者对于Python 3：

```python
#!/usr/bin/env python3

# Python 3代码...
```

这里的`#!/usr/bin/env python`是shebang行。它指示操作系统使用`env`命令查找系统路径中的`python`解释器。

### 解释

- `#!`：这是shebang的开头，告诉操作系统这行是一个特殊的指令。

- `/usr/bin/env python`：这指示操作系统使用`env`程序，在用户的环境中查找名为`python`的解释器。这是一种更通用的方法，因为它独立于Python解释器的具体安装位置。

### ROS中的应用

在ROS中，当你创建一个Python节点时，应在文件的开头加上适当的shebang。这样，当你使用`rosrun`或`roslaunch`启动节点时，ROS能够正确地解释和执行这个脚本。

例如，如果你有一个名为`listener.py`的Python脚本，你应该这样开始脚本：

```python
#!/usr/bin/env python

import rospy
# 其他ROS相关代码...
```

这使得在ROS环境中运行`rosrun your_package listener.py`或在`launch`文件中引用`listener.py`时，脚本可以正确执行。

### 总结

Shebang是Python脚本中的一个重要特性，特别是在ROS环境中。它确保了脚本可以被正确的解释器执行，而无需在运行脚本时指定解释器。对于Python 2和Python 3，可能需要使用不同的shebang行，这取决于你的ROS环境和Python版本。

## coding

在Python中，`# -*- coding: UTF-8 -*-`或类似的编码声明用于指定文件中使用的字符编码。这对于Python 2非常重要，尤其是在处理非ASCII字符（如中文、日文、特殊符号等）时。在Python 3中，编码默认为UTF-8，因此通常不需要这样的声明。

### 编码声明的作用

1. **指定字符编码**：告诉Python解释器文件中使用的编码方式，以正确地读取和处理文件中的字符。

2. **兼容性**：确保在不同环境中脚本的行为保持一致，特别是当脚本中包含非ASCII字符时。

### 示例

假设你在ROS中写了一个Python脚本，该脚本包含一些非英文字符，你可能会在文件顶部包含一个编码声明：

```python
# -*- coding: UTF-8 -*-

# Python代码，可能包含非ASCII字符
```

这行告诉Python解释器脚本使用的是UTF-8编码。

### 在ROS中的应用

在ROS中编写Python脚本时，如果你的脚本包含非ASCII字符（如注释或字符串中的特殊字符），在Python 2环境中添加编码声明是个好习惯。这样可以确保你的脚本在不同环境下都能正确运行。

例如：

```python
# -*- coding: UTF-8 -*-
import rospy

# ROS节点初始化
rospy.init_node('my_ros_node')

# 日志中打印一些中文
rospy.loginfo("这是一条中文日志信息")
```

在这个例子中，由于日志信息包含了中文字符，因此在文件顶部添加了UTF-8编码声明。

### 总结

虽然在Python 3中默认编码是UTF-8，使得在大多数情况下不需要特别声明编码，但在Python 2中，特别是当处理包含非ASCII字符的脚本时，正确声明文件编码是非常重要的。这确保了脚本的兼容性和可移植性。在ROS环境中，遵循这个实践可以帮助避免与字符编码相关的问题。

## 添加可执行权限

在Linux和类Unix系统中，给Python文件添加可执行权限意味着使文件成为可直接执行的程序。这通常通过修改文件的权限来实现。以下是如何给Python文件添加可执行权限的步骤：

### 1. 打开终端

首先，打开一个终端窗口。

### 2. 切换到文件所在目录

使用`cd`命令切换到包含你的Python文件的目录。例如：

```bash
cd ~/ros_workspace/src/my_ros_package/scripts
```

确保替换路径为你的Python文件实际所在的路径。

### 3. 修改文件权限

使用`chmod`命令添加执行权限。假设你的Python文件名为`my_script.py`，则运行：

```bash
chmod +x my_script.py
```

这个命令将为所有用户添加执行（x）权限。

### 4. 验证更改

为了验证更改是否成功，可以使用`ls -l`命令查看文件权限：

```bash
ls -l my_script.py
```

输出应该显示该文件现在具有执行权限，类似于：

```
-rwxr-xr-x 1 user user 0 Feb 16 12:00 my_script.py
```

这里的`-rwxr-xr-x`表示所有用户都可以读取和执行该文件，文件所有者还可以写入文件。

### 注意事项

- 在ROS中，给Python脚本文件添加执行权限是很重要的，因为这样`rosrun`或`roslaunch`才能直接运行这些脚本。

- 如果你的系统是Windows，这个过程可能会有所不同，因为Windows的文件权限管理与Linux/Unix不同。在Windows上运行ROS时，通常不需要特别关注执行权限，因为Python脚本通常是通过明确调用Python解释器来运行的。

通过这些步骤，你可以确保你的Python脚本在ROS环境中可以被正确执行。