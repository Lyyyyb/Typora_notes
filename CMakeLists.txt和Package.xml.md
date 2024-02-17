# CMakeLists.txt和Package.xml

## CMakeLists.txt

### 总览

`CMakeLists.txt` 是用于定义如何构建 ROS (Robot Operating System) 包的 CMake 脚本文件。CMake 是一个跨平台的构建系统，用于自动化编译过程。在 ROS 中，`CMakeLists.txt` 文件指定了如何编译代码和链接库，以及如何安装各种文件和程序。以下是 `CMakeLists.txt` 的详细解释：

#### 基本结构

`CMakeLists.txt` 文件通常包含以下部分：

1. **CMake 最低版本要求**:
   - 指定编译此包所需的最低 CMake 版本。
   ```cmake
   cmake_minimum_required(VERSION 3.0.2)
   ```

2. **项目名称**:
   - 设置项目名称，通常与 ROS 包名称相同。
   ```cmake
   project(my_ros_package)
   ```

3. **查找依赖项**:
   - 使用 `find_package` 命令查找编译包所需的依赖项。
   ```cmake
   find_package(catkin REQUIRED COMPONENTS
     roscpp
     rospy
     std_msgs
   )
   ```

4. **设置 catkin 包**:
   - 使用 `catkin_package` 命令定义包的属性，如包含的库、依赖项和包含目录。
   ```cmake
   catkin_package(
     #  INCLUDE_DIRS include
     #  LIBRARIES my_ros_package
     #  CATKIN_DEPENDS roscpp rospy std_msgs
     #  DEPENDS system_lib
   )
   ```

5. **指定头文件的路径**:
   - 通常用于包含库的头文件。
   ```cmake
   include_directories(
     # include
     ${catkin_INCLUDE_DIRS}
   )
   ```

6. **添加可执行文件**:
   - 使用 `add_executable` 命令添加每个节点的可执行文件。
   ```cmake
   add_executable(${PROJECT_NAME}_node src/my_node.cpp)
   ```

7. **添加依赖项**:
   - 使用 `add_dependencies` 确保在编译节点之前生成所有消息和服务。
   ```cmake
   add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
   ```

8. **链接库**:
   - 使用 `target_link_libraries` 将可执行文件与库链接起来。
   ```cmake
   target_link_libraries(${PROJECT_NAME}_node
     ${catkin_LIBRARIES}
   )
   ```

9. **安装目标**:
   - 指定如何安装各种文件和程序，以便它们可以在 ROS 环境中找到。
   ```cmake
   install(TARGETS ${PROJECT_NAME}_node
     RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   )
   ```

10. **其他指令**:
    - 如设置测试、配置文件和其他特殊构建指令。

#### 重要性

`CMakeLists.txt` 文件对于 ROS 包的构建至关重要。它确保了代码能够根据定义的规则和依赖关系正确编译。良好维护的 `CMakeLists.txt` 文件对于项目的可移植性和可维护性非常重要，尤其是当涉及到大型项目或多人协作时。

此外，通过 `CMakeLists.txt`，开发者可以细粒度地控制编译过程，比如定义不同的构建类型（如调试或发布），以及设置特定的编译器标志。

### 1. find_package指令

```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
```

这段代码是CMakeLists.txt文件中的一个部分，用于在ROS包中找到和包含依赖项。

- **`find_package`**：这是CMake的一个命令，用于在构建过程中定位和使用外部项目或库。在这种情况下，它被用来找到`catkin`，这是ROS的一个构建系统。
  
- **`catkin REQUIRED`**：指明`catkin`是必需的。如果CMake无法找到`catkin`，构建过程将停止。

- **`COMPONENTS`**：后面跟着的是一个列表，指定了你的ROS包所依赖的其他ROS包或库。
  
  - **`roscpp`**：ROS的C++客户端库，用于写C++节点。
  - **`rospy`**：ROS的Python客户端库，用于写Python节点。
  - **`std_msgs`**：一个标准消息包，包含了一些常用的消息类型，如整数、浮点数、字符串等。

### 2. add_executable指令

```cmake
add_executable(${PROJECT_NAME}_node src/atr_pkg_node.cpp)
```

这行代码用于添加一个可执行文件到你的ROS包。

- **`add_executable`**：这是CMake的命令，用于创建一个可执行文件。在这里，它指定了可执行文件的名称和源代码文件。

- **`${PROJECT_NAME}_node`**：这是可执行文件的名称。`${PROJECT_NAME}`是一个变量，代表当前项目的名称。在这里，它被用来为可执行文件命名，通常后面会附加`_node`以表示这是一个ROS节点。

- **`src/atr_pkg_node.cpp`**：这是可执行文件的源代码文件路径。这里假设你的源代码文件名为`atr_pkg_node.cpp`，位于`src`目录下。

### 3. add_dependencies指令

```cmake
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
```

这行代码为你的可执行文件添加构建依赖。

- **`add_dependencies`**：这是CMake的命令，用于添加目标（在这种情况下是你的可执行文件）的依赖项。这确保了在构建你的节点之前，所有必需的头文件和服务消息都已经可用。

- **`${PROJECT_NAME}_node`**：这是你之前用`add_executable`创建的目标（即可执行文件）。

- **`${${PROJECT_NAME}_EXPORTED_TARGETS}`** 和 **`${catkin_EXPORTED_TARGETS}`**：这些是CMake变量，包含了所有必须在你的节点之前构建的目标。这通常包括由消息生成的目标，确保在构建你的节点之前，所有自定义消息、服务或操作的头文件都已生成。

总的来说，这些代码片段在ROS包的CMake配置中起着非常重要的作用，确保了包的依赖项被正确地找到和包含，可执行文件被正确地构建，以及所有必要的依赖在构建过程中得到满足。

### 4.target_link_libraries指令

这段代码是ROS包中`CMakeLists.txt`文件的一部分，用于配置ROS包的构建过程。它具体涉及到链接库的设置。下面是对这段代码的详细解释：

```cmake
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)
```

- **`target_link_libraries`**：这是CMake的一个命令，用于指定可执行文件或库应该链接哪些库。在构建一个目标（如可执行文件或库）时，它告诉CMake将指定的库链接到这个目标上。

- **`${PROJECT_NAME}_node`**：这是之前使用`add_executable`命令创建的目标的名称。`${PROJECT_NAME}`是一个CMake变量，表示当前项目（即ROS包）的名称。通常情况下，`${PROJECT_NAME}_node`是可执行文件的名称，表示ROS包中的一个节点。

- **`${catkin_LIBRARIES}`**：这是一个CMake变量，包含了所有由`find_package(catkin REQUIRED COMPONENTS ...)`找到的catkin组件的库。在ROS中，`catkin_LIBRARIES`变量通常包含了所有必要的ROS相关库，如`roscpp`（ROS的C++库）和其他你在`CMakeLists.txt`中声明为依赖的库。

简而言之，这个`target_link_libraries`命令确保你的可执行文件（节点）在构建时，会链接到所有在`find_package`中指定的、及其依赖的ROS和其他库。这对于确保你的节点能够正确地访问和使用这些库的功能是必要的。

# Package.xml

## 总览

`package.xml` 文件是 ROS (Robot Operating System) 包的一个关键组件，用于定义包的元数据和依赖关系。这个文件遵循特定的 XML 结构，并提供了关于包的重要信息，包括名称、版本、维护者、许可证和依赖项。以下是 `package.xml` 的详细解释：

#### 基本结构

`package.xml` 文件通常包含以下基本部分：

1. **包名称** (`<name>`): 包的唯一标识符。
2. **版本** (`<version>`): 包的版本号。
3. **描述** (`<description>`): 关于包的简短描述。
4. **维护者** (`<maintainer>`): 负责维护包的人的联系信息。
5. **许可证** (`<license>`): 指定包的开源许可证类型。
6. **依赖关系**:
   - `<build_depend>`: 构建时需要的依赖。
   - `<build_export_depend>`: 构建和运行时需要的依赖。
   - `<exec_depend>`: 运行时需要的依赖。
   - `<depend>`: 同时是构建和运行时需要的依赖。

#### 示例

下面是一个 `package.xml` 的示例：

```xml
<?xml version="1.0"?>
<package format="2">
  <name>example_package</name>
  <version>0.0.1</version>
  <description>An example ROS package</description>

  <maintainer email="maintainer@example.com">Your Name</maintainer>

  <license>BSD</license>

  <build_depend>roscpp</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_export_depend>roscpp</build_export_depend>
  <exec_depend>roscpp</exec_depend>
  <exec_depend>std_msgs</exec_depend>

  <export>
    <!-- Other export tags go here -->
  </export>
</package>
```

在这个示例中：

- 包名为 `example_package`。
- 版本号设为 `0.0.1`。
- 提供了包的简短描述。
- 维护者信息包括名字和电子邮件地址。
- 指定了 BSD 许可证。
- 包括了对 `roscpp` 和 `std_msgs` 的依赖，这些在构建和运行时都是必需的。

#### 重要性

`package.xml` 文件对于 ROS 包的构建和分发至关重要。它不仅提供了包的基本信息，还确定了包与其他包之间的关系。正确配置 `package.xml` 对于确保包的正确编译和运行是必要的。

当你从源代码编译 ROS 包或者使用 ROS 工具（如 `rospack`、`roslaunch`）管理包时，`package.xml` 中的信息都会被使用。因此，维护一个准确、完整的 `package.xml` 文件对于任何 ROS 项目都是非常重要的。



```xml
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_export_depend>roscpp</build_export_depend>
  <build_export_depend>rospy</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>
  <exec_depend>roscpp</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
```

这段代码是来自ROS包的`package.xml`文件，它定义了包的依赖项。`package.xml`文件在ROS中用于声明包的元数据和依赖关系，这对于包的构建和运行至关重要。下面是对这段代码的详细解释：

- **`<buildtool_depend>catkin</buildtool_depend>`**：指定了一个构建工具依赖。这表示此包在构建过程中需要`catkin`。`catkin`是ROS的构建系统，用于编译和配置ROS包。

- **`<build_depend>`**：这些标签指定了构建依赖。构建依赖是指在编译时间需要的依赖项。在这个例子中，它包括了`roscpp`、`rospy`和`std_msgs`。
  - **`roscpp`**：ROS的C++客户端库，用于C++节点的开发。
  - **`rospy`**：ROS的Python客户端库，用于Python节点的开发。
  - **`std_msgs`**：包含标准消息类型的ROS包，这些消息在不同的ROS节点之间进行通信时常常用到。

- **`<build_export_depend>`**：这些标签指定了构建导出依赖。这意味着当其他包依赖于此包时，这些依赖也将被考虑。在这个例子中，同样包括了`roscpp`、`rospy`和`std_msgs`。

- **`<exec_depend>`**：这些标签指定了执行依赖，即在运行时需要的依赖项。在这个例子中，也包括了`roscpp`、`rospy`和`std_msgs`。这意味着当此包的节点被运行时，这些依赖项需要被满足。

总的来说，这段代码详细描述了包的依赖关系，包括构建工具、构建时依赖、构建导出依赖以及执行时依赖。这些信息对于ROS的构建系统来说是必需的，以确保包可以正确地编译和运行。

## 后续增加，减少，更改依赖

在ROS（Robot Operating System）中更改依赖时，需要修改`CMakeLists.txt`和`package.xml`两个文件。这些文件定义了ROS包的构建和运行时依赖。以下是详细的步骤说明：

### 修改`package.xml`

`package.xml`文件定义了包的元数据和依赖关系。当你需要添加、移除或更改依赖时，应相应地更新这个文件。

1. **添加依赖**：
   - 使用`<depend>`标签添加新的依赖。这个标签表示包在编译和运行时都需要这个依赖。
   - 如果依赖仅在编译时需要，使用`<build_depend>`；如果仅在运行时需要，使用`<exec_depend>`。

2. **移除依赖**：
   - 直接删除相应的依赖标签。

3. **修改依赖**：
   - 替换或更改现有的依赖标签。

例如，假设你需要添加一个名为`sensor_msgs`的依赖：

```xml
<depend>sensor_msgs</depend>
```

### 修改`CMakeLists.txt`

`CMakeLists.txt`控制着包的构建过程。你需要在这个文件中添加、移除或更改`find_package()`中的依赖项。

1. **添加依赖**：
   - 在`find_package(catkin REQUIRED COMPONENTS ...)`部分添加新的依赖项。

2. **移除依赖**：
   - 从`find_package()`中删除不再需要的依赖项。

3. **修改依赖**：
   - 根据需要更改`find_package()`中的依赖项。

例如，添加`sensor_msgs`依赖：

```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)
```

### 重构建包

修改`package.xml`和`CMakeLists.txt`后，需要重新构建ROS包以应用这些更改：

1. **在终端中，导航到你的catkin工作空间**（通常是`~/catkin_ws`）。

2. **运行`catkin_make`命令**：

   ```sh
   cd ~/catkin_ws
   catkin_make
   ```

这将重新构建你的包，包括任何新添加的依赖。

### 测试包

更改依赖后，建议进行适当的测试以确保包正常工作，并且新的依赖没有引入任何问题。

### 总结

通过适当地管理`package.xml`和`CMakeLists.txt`文件，你可以确保你的ROS包正确地包含了所有必要的依赖，这对于包的功能性、可移植性和维护性至关重要。