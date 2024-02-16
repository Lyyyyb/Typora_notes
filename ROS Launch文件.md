# ROS Launch文件

## 小结

- 使用launch文件，可以通过roslaunch指令一次启动多个节点
- 在launch文件中，为节点添加output=“screen”属性，可以让节点信息输出在终端中。（ROS_WARN不受该属性影响）
- 在launch文件中，为节点添加launch-prefix=“gnome-terminal -e”属性，可以让节点单独运行在一个独立终端中

## Launch

在ROS（Robot Operating System）中，`launch`文件是用来定义和配置一组要同时启动的节点的XML文件。这些文件极大地简化了管理和启动复杂的ROS系统。通过使用`launch`文件，你可以一次性启动多个节点，设置它们的参数，重新映射话题名称等。

### `launch`文件的基本结构

1. **<launch> 标签**：`launch`文件的根元素。
2. **<node> 标签**：用于定义要启动的每个节点。
   - `pkg`：节点所属的ROS包。
   - `type`：执行文件的名称。
   - `name`：运行时节点的名称。
   - `output`：节点的输出类型（如“screen”）。
   - `args`：传递给节点的任何命令行参数。
3. **<param> 标签**：用于设置节点的参数。
4. **<rosparam> 标签**：用于从YAML文件加载参数。
5. **<remap> 标签**：用于重新映射话题名称。

### `launch`文件示例

假设我们有两个ROS节点：一个名为`talker`的节点发布消息，一个名为`listener`的节点订阅消息。以下是一个`launch`文件的示例，它同时启动这两个节点：

```xml
<launch>
    <!-- 启动 talker 节点 -->
    <node pkg="beginner_tutorials" type="talker" name="talker_node" output="screen">
        <!-- 重新映射话题 -->
        <remap from="chatter" to="/my_chatter"/>
    </node>

    <!-- 启动 listener 节点 -->
    <node pkg="beginner_tutorials" type="listener" name="listener_node" output="screen"/>
</launch>
```

在这个例子中：

- 我们有两个`<node>`标签，分别用于启动`talker`和`listener`节点。`pkg`指定了节点所在的包，`type`指定了可执行文件的名称，`name`是运行时节点的名称。
- 在`talker`节点中，我们使用了`<remap>`标签来将`chatter`话题重新映射为`/my_chatter`。
- `output="screen"`确保节点的输出被发送到控制台。

### 运行`launch`文件

要运行这个`launch`文件，你需要在终端中使用以下命令：

```bash
roslaunch beginner_tutorials example.launch
```

这个命令将会启动`example.launch`文件中定义的所有节点。

### 总结

使用`launch`文件可以有效地管理和启动ROS系统中的多个节点，这对于调试和运行复杂的ROS应用程序非常有用。通过在`launch`文件中指定节点配置和行为，可以实现高度的自定义和自动化，简化了ROS系统的使用。

## 运行相同类型的多个节点

在ROS（Robot Operating System）中，如果你需要运行相同类型的多个节点，例如，当你有多个相同类型的传感器或要同时执行多个相似的任务时，你可以在`launch`文件中通过以下几种方法来实现：

### 1. 使用不同的节点名称

你可以在`launch`文件中为每个节点实例指定不同的名称。这是最直接的方法，可以避免节点间的名称冲突。

#### 示例：

```xml
<launch>
    <!-- 启动第一个talker节点 -->
    <node pkg="beginner_tutorials" type="talker" name="talker1" output="screen"/>

    <!-- 启动第二个talker节点 -->
    <node pkg="beginner_tutorials" type="talker" name="talker2" output="screen"/>
</launch>
```

这里，两个节点都是由相同的可执行文件（`talker`）生成的，但它们被赋予了不同的名称（`talker1` 和 `talker2`）。

### 2. 使用ROS命名空间

利用ROS的命名空间，你可以将同名的节点放在不同的命名空间中，从而允许它们同时运行而不会相互干扰。

#### 示例：

```xml
<launch>
    <!-- 第一个talker节点在ns1命名空间下 -->
    <group ns="ns1">
        <node pkg="beginner_tutorials" type="talker" name="talker" output="screen"/>
    </group>

    <!-- 第二个talker节点在ns2命名空间下 -->
    <group ns="ns2">
        <node pkg="beginner_tutorials" type="talker" name="talker" output="screen"/>
    </group>
</launch>
```

在这个例子中，`talker`节点分别在两个不同的命名空间`ns1`和`ns2`下运行。

### 3. 使用`arg`参数和`foreach`循环（仅限于某些ROS版本）

在某些版本的ROS中，你可以使用`arg`和`foreach`循环来启动多个相同类型的节点。

#### 示例：

```xml
<launch>
    <arg name="num_nodes" default="2"/>
    <rosparam param="/num_nodes">${num_nodes}</rosparam>

    <!-- 使用循环来启动多个节点 -->
    <group ns="sensor">
        <node pkg="beginner_tutorials" type="talker" name="talker$(arg num_nodes)" output="screen"/>
    </group>
</launch>
```

这里，`$(arg num_nodes)` 用于动态设置节点的名称。

### 总结

运行相同类型的多个ROS节点时，确保每个节点都有一个独特的名称是关键。你可以通过指定不同的节点名称、使用不同的命名空间或利用`arg`参数和`foreach`循环（在支持的ROS版本中）来实现这一点。这些方法都确保了系统的灵活性和可扩展性。

## C++和Python在编写launch时的区别

在ROS（Robot Operating System）中，使用C++和Python编写的节点可以通过`.launch`文件启动，无论节点是用C++还是Python编写的，`.launch`文件的基本结构和语法都是相同的。不过，有一些细节上的差异需要注意：

### 1. 节点类型指定

>C++节点是编译后的二进制文件，不带.cpp后缀
>
>python节点是直接运行的代码文件，需要写上.py后缀

- **C++ 节点**：
  - 在C++中，节点是由源代码编译成可执行文件的。在`launch`文件中，`type`属性需要与你的可执行文件名匹配（不包括任何路径或扩展名）。

- **Python 节点**：
  - 对于Python编写的节点，`type`属性应该是Python脚本的文件名（包括`.py`扩展名）。确保Python脚本在你的ROS包的`scripts`目录中，并且具有执行权限（通常通过`chmod +x your_script.py`命令设置）。

### 2. 示例

假设你有一个用C++编写的节点`talker_cpp`和一个用Python编写的节点`listener_py`：

#### C++ 节点启动

```xml
<launch>
    <node pkg="beginner_tutorials" type="talker_cpp" name="talker_cpp_node" output="screen"/>
</launch>
```

- `pkg`：包含节点的ROS包的名称。
- `type`：编译生成的可执行文件名称。
- `name`：节点运行时的名称。

#### Python 节点启动

```xml
<launch>
    <node pkg="beginner_tutorials" type="listener.py" name="listener_py_node" output="screen"/>
</launch>
```

- `pkg`：包含节点的ROS包的名称。
- `type`：Python脚本文件名，包括`.py`扩展名。
- `name`：节点运行时的名称。

### 3. 总结

虽然C++和Python编写的ROS节点在`.launch`文件中的处理方式基本相同，但需要注意`type`属性的不同处理方式。对于C++节点，`type`应对应于编译的可执行文件名；而对于Python节点，`type`则应是脚本的文件名，包括`.py`扩展名。此外，确保Python脚本在正确的位置并具有适当的执行权限。

## output属性

在ROS（Robot Operating System）的`.launch`文件中，`<node>`标签的`output`属性用于指定节点运行时输出信息的目的地。这个属性非常重要，因为它决定了节点的日志信息、调试信息和错误信息的显示位置。

### `output` 属性的作用

1. **输出目的地**：`output`属性决定节点的标准输出（stdout）和标准错误（stderr）信息的去向。

2. **输出选项**：常用的选项包括：
   - `screen`：输出信息直接显示在终端（屏幕）上。这在调试时非常有用。
   - `log`：输出信息被发送到ROS的日志文件中。适合在后台运行时使用。

### 示例

#### 1. 输出到屏幕

如果你想要节点的输出信息直接在终端上显示，可以将`output`属性设置为`screen`。这在调试节点时非常有用，因为你可以立即看到输出结果。

```xml
<launch>
    <node pkg="beginner_tutorials" type="talker" name="talker_node" output="screen"/>
</launch>
```

在这个例子中，`talker`节点的所有输出信息（如`ROS_INFO`, `ROS_WARN`等）将直接显示在启动`roslaunch`命令的终端窗口中。

#### 2. 输出到日志

如果你想要节点的输出信息被记录到ROS的日志文件中，而不是直接显示在屏幕上，可以将`output`属性设置为`log`。

```xml
<launch>
    <node pkg="beginner_tutorials" type="listener" name="listener_node" output="log"/>
</launch>
```

在这个例子中，`listener`节点的所有输出信息将被发送到ROS的日志文件中。这对于在生产环境中运行节点或者当你不需要即时查看输出时非常有用。

### 总结

`output`属性在`.launch`文件中的`<node>`标签内用于控制节点的输出信息的去向。根据不同的需求，你可以选择将输出信息直接显示在屏幕上，便于实时监控和调试，或者将输出信息记录到日志文件中，适用于长期运行或后台运行的情况。通过合理使用这个属性，可以更有效地管理和查看节点的输出信息。

## 配置节点的输出使其显示在一个独立的终端窗口中

在ROS（Robot Operating System）的`.launch`文件中，`launch-prefix="gnome-terminal -e"`用于在新的`gnome-terminal`窗口中启动ROS节点。这个设置非常有用，尤其是当你想要单独监控特定节点的输出时。

### `launch-prefix="gnome-terminal -e"`的作用

1. **新终端窗口**：这个命令会打开一个新的`gnome-terminal`终端窗口。

2. **执行命令**：`-e`选项后跟的命令会在新打开的`gnome-terminal`窗口中执行。在ROS的上下文中，这通常是启动特定节点的命令。

### 示例

假设你有一个名为`talker`的节点，你想在单独的`gnome-terminal`窗口中启动它并查看其输出。你可以在`.launch`文件中使用以下配置：

```xml
<launch>
    <node pkg="beginner_tutorials" type="talker" name="talker_node" output="screen"
          launch-prefix="gnome-terminal -e"/>
</launch>
```

在这个例子中：

- `pkg="beginner_tutorials"` 指定节点所在的ROS包。
- `type="talker"` 是节点的可执行文件名。
- `name="talker_node"` 是节点运行时的名称。
- `output="screen"` 确保节点的输出显示在屏幕上。
- `launch-prefix="gnome-terminal -e"` 用于在新的`gnome-terminal`窗口中启动节点。这里，`-e`后面的节点启动命令会被自动填充。

### 注意事项

- **终端模拟器**：这种方法假定你的系统已经安装了`gnome-terminal`。如果你使用的是不同的操作系统或终端模拟器，可能需要相应地调整命令。

- **命令格式**：确保正确地使用`-e`选项。一些新版本的`gnome-terminal`可能要求使用`--`来分隔选项和要执行的命令。

- **环境变量**：在新打开的终端窗口中，确保所有必要的ROS环境变量已经设置。有时，你可能需要在`launch-prefix`中包含环境设置命令，例如`source /opt/ros/noetic/setup.bash`。

### 总结

使用`launch-prefix="gnome-terminal -e"`是在独立的终端窗口中运行ROS节点的有效方法，这在进行ROS系统的开发和调试时尤其有用。通过这种方式，你可以为每个节点创建一个单独的窗口，从而更好地管理和观察它们的输出和行为。