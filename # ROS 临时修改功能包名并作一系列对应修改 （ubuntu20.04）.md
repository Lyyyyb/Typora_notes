# ROS 临时修改功能包名并作一系列对应修改 （ubuntu20.04）

在ROS中临时修改一个功能包的包名确实需要更新多个文件和配置，确保整个系统的一致性不受影响。以下是具体步骤和C++相关的示例：

### 1. 修改 `package.xml` 文件
这个文件描述了包的基本信息。需要更新 `<name>` 标签以反映新的包名。

**示例：**
假设原来的包名是 `old_package_name`，你希望修改为 `new_package_name`。

修改前：
```xml
<package>
  <name>old_package_name</name>
  <version>0.0.1</version>
  ...
</package>
```

修改后：
```xml
<package>
  <name>new_package_name</name>
  <version>0.0.1</version>
  ...
</package>
```

### 2. 修改 `CMakeLists.txt` 文件
这个文件控制包的构建过程，需要更新所有涉及到旧包名的地方，尤其是 `project()` 函数。

**示例：**
将 `project()` 行中的包名从 `old_package_name` 改为 `new_package_name`。

修改前：
```cmake
cmake_minimum_required(VERSION 3.0.2)
project(old_package_name)
...
```

修改后：
```cmake
cmake_minimum_required(VERSION 3.0.2)
project(new_package_name)
...
```

### 3. 修改节点启动文件（`.launch` 文件）
更新这些文件中所有引用旧包名的地方，以确保节点可以正确启动。

**示例：**
在 `.launch` 文件中，更新节点的包名。

修改前：
```xml
<launch>
  <node pkg="old_package_name" type="example_node" name="example_node" output="screen"/>
</launch>
```

修改后：
```xml
<launch>
  <node pkg="new_package_name" type="example_node" name="example_node" output="screen"/>
</launch>
```

### 4. 更新源代码中的包名引用
检查C++源代码文件，并更新其中的包名引用。

**示例：**
在C++源文件中，更新包名引用。

修改前：
```cpp
#include "old_package_name/Example.h"
```

修改后：
```cpp
#include "new_package_name/Example.h"
```

### 5. 清理构建和安装目录
删除 `build` 和 `devel` 目录中的旧构建文件，然后使用 `catkin_make` 重新构建。

**命令：**
```bash
cd /path/to/your/catkin_workspace
rm -rf build/ devel/
catkin_make
```

### 6. 修改环境配置文件
如果 `.bashrc` 或 `.zshrc` 中有引用旧包名的环境变量，也需要进行更新。

**示例：**
更新 `.bashrc` 中的环境变量。

修改前：
```bash
export ROBOT_PACKAGE_PATH=/home/user/catkin_ws/src/old_package_name:$ROBOT_PACKAGE_PATH
```

修改后：
```bash
export ROBOT_PACKAGE_PATH=/home/user/catkin_ws/src/new_package_name:$ROBOT_PACKAGE_PATH
```

### 总结
完成上述修改后，确保运行 `source ~/.bashrc`（或对应的配置文件）来更新环境变量，然后重新编译你的工作空间。这些步骤将帮助你确保所有的更改都被正确地应用，并且避免了运行时的错误。这样的修改是全面的，涵盖了从文件级别到环境配置的所有必要改动。