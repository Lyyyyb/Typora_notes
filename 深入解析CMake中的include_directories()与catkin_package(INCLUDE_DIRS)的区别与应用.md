# 深入解析CMake中的include_directories()与catkin_package(INCLUDE_DIRS)的区别与应用

在CMake中，`include_directories()` 和 `catkin_package(INCLUDE_DIRS include)` 是用于设置头文件包含路径的两种指令，但它们在作用范围、用途和应用场景上有显著的区别。以下将详细解释这两者的区别和用途，并提供示例以便更好地理解。

### 1. `include_directories()`

#### 功能
- `include_directories()` 是一个标准的CMake命令，用于为当前CMake项目（即当前包）添加一个或多个包含路径（include paths）。这些路径告诉编译器在编译当前项目时到哪里去查找头文件。

#### 用途
- 主要用于当前包的编译环境配置。
- 确保当前包的源文件能够找到所需的头文件。
- 影响范围仅限于调用此命令的包，其他包无法获得这些包含路径。

#### 示例
```cmake
include_directories(include)
```
在这个示例中，`include`目录将被添加到当前包的头文件搜索路径中。当编译此包的源文件时，编译器会在`include`目录中查找头文件。

### 2. `catkin_package(INCLUDE_DIRS include)`

#### 功能
- `catkin_package(INCLUDE_DIRS include)` 是Catkin特有的命令，用于在ROS包中声明公共的包含目录。它不仅设置了当前包的头文件路径，还使这些路径在依赖于此包的其他包中可用。

#### 用途

- 作为接口向其他依赖于此包的包公开包含路径。
- 确保其他包在使用`find_package()`找到此包时，能够自动获得必要的头文件路径。
- 促进包之间的依赖关系管理，使得当其他包编译时能够轻松找到所需的头文件。

#### 示例
```cmake
catkin_package(
  INCLUDE_DIRS include
)
```
在这个示例中，`include`目录被声明为对其他包可见。当其他包通过`find_package()`找到当前包后，它们将自动将`include`目录添加到自己的头文件搜索路径中。

### 主要区别总结

| 特性         | `include_directories()`                      | `catkin_package(INCLUDE_DIRS ...)`                         |
| ------------ | -------------------------------------------- | ---------------------------------------------------------- |
| **作用范围** | 仅影响当前包的编译环境。                     | 影响当前包及依赖此包的其他包的编译环境。                   |
| **公开性**   | 仅用于当前包内部，其他包无法访问。           | 公开给依赖此包的其他包，确保它们可以找到这些头文件。       |
| **用途**     | 添加头文件搜索路径，以确保当前包的编译正常。 | 声明当前包的公共接口，以便其他包可以依赖并访问这些头文件。 |

### 示例结合使用

在一个完整的ROS包中，可能会同时使用这两个命令。例如：

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(my_robot_driver)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(my_driver src/driver.cpp)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES my_driver
  CATKIN_DEPENDS roscpp std_msgs
)
```

- **`include_directories()`** 用于确保当前包的源文件可以找到 `include` 目录中的头文件。
- **`catkin_package(INCLUDE_DIRS include)`** 使得任何依赖于 `my_robot_driver` 包的其他包在编译时自动获得对 `include` 目录的访问权限。

通过这样的配置，可以确保当前包和其他依赖包的构建过程都顺利进行，同时保持代码的整洁和可维护性。