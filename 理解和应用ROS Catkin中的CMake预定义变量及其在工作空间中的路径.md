# 理解和应用ROS Catkin中的CMake预定义变量及其在工作空间中的路径

在ROS（Robot Operating System）的catkin构建系统中，CMake是核心的组成部分，用于管理和构建各种软件包。Catkin扩展了CMake的功能，提供了一系列预定义变量，这些变量在ROS工作空间中具有特定的路径和用途。理解这些变量如何映射到ROS工作空间的路径，对于高效地使用ROS进行机器人软件开发至关重要。

### 1. CATKIN_DEVEL_PREFIX
**定义**：
这个变量定义了开发阶段的工作空间目录，通常指向`<workspace_directory>/devel`，这是在源代码直接被编译而未安装时使用的目录。

**路径示例**：
如果你的工作空间位于`/home/user/catkin_ws`，则`CATKIN_DEVEL_PREFIX`的值通常为`/home/user/catkin_ws/devel`。

**CMake 使用示例**：
在CMakeLists.txt文件中，通常不需要显式设置此变量，它是由catkin环境自动管理的。

### 2. CATKIN_PACKAGE_BIN_DESTINATION
**定义**：
定义可执行文件在`make install`时的安装目标位置，相对于全局安装前缀`CMAKE_INSTALL_PREFIX`。通常，这个变量指向`lib/<package_name>`。

**路径示例**：
对于上述工作空间，如果有一个名为`my_package`的包，安装后的可执行文件路径会是`/home/user/catkin_ws/install/lib/my_package`。

**CMake 使用示例**：
```cmake
install(TARGETS my_executable
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

### 3. CATKIN_PACKAGE_LIB_DESTINATION
**定义**：
指定库文件的安装位置，相对于`CMAKE_INSTALL_PREFIX`。通常这个变量指向`lib/<package_name>`。

**路径示例**：
同样对于包`my_package`，库文件安装后的路径为`/home/user/catkin_ws/install/lib/my_package`。

**CMake 使用示例**：
```cmake
install(TARGETS my_library
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
```

### 4. CATKIN_PACKAGE_SHARE_DESTINATION
**定义**：
用于指定共享资源（如launch文件、配置文件等）的安装位置。通常，这个变量指向`share/<package_name>`。

**路径示例**：
对于包`my_package`，共享资源安装后的路径为`/home/user/catkin_ws/install/share/my_package`。

**CMake 使用示例**：
```cmake
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
```

### 5. CATKIN_GLOBAL_BIN_DESTINATION, CATKIN_GLOBAL_LIB_DESTINATION, 和 CATKIN_GLOBAL_SHARE_DESTINATION
这些变量分别用于指定全局二进制文件、库文件和共享资源的安装目录，通常指向`bin`, `lib`, 和 `share`目录。

**路径示例**：
对于`CATKIN_GLOBAL_LIB_DESTINATION`，全局库文件的安装路径可能是`/home/user/catkin_ws/install/lib`。

**CMake 使用示例**：
```cmake
install(PROGRAMS scripts/my_script
  DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
```

### 6. catkin_package()
此宏在CMake中声明catkin项目的配置信息，如依赖关系、包含目录、库文件等。这不是一个路径变量，但它对于正确配置包中的目标和依赖至关重要。

**CMake 使用示例**：
```cmake
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS system_lib
)
```

通过这些变量和宏的合理使用，可以确保ROS包的构建、安装和部署按照预期进行，同时也保证了各个组件在ROS生态系统中的互操作性和可移植性。