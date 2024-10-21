# 深入理解CMake中的include_directories()与catkin_package(INCLUDE_DIRS)的区别及应用

在CMake和ROS的catkin构建系统中，`include_directories()` 和 `catkin_package(INCLUDE_DIRS include)` 是两个常见但功能不同的命令。它们都与处理头文件和包含目录有关，但用途和影响的范围不同。理解这两个命令的差异对于高效和正确地使用CMake进行项目构建至关重要。

### 1. `include_directories()`

**功能描述**：
`include_directories()` 是CMake的原生命令，用于向项目添加包含目录。这些目录后续将被编译器用来查找头文件。此命令影响所有在其调用之后声明的目标（如可执行文件和库）。

**使用范围**：
它主要用于当前CMake项目中，向编译器的包含路径中添加目录，使得项目中的所有源文件都能够找到指定的头文件。

**示例**：
假设你的项目结构如下：
```
/MyProject
|-- include/
|   |-- my_library.h
|-- src/
|   |-- main.cpp
|-- CMakeLists.txt
```
你可以在 `CMakeLists.txt` 中使用 `include_directories()` 如下：
```cmake
project(MyProject)
add_executable(my_app src/main.cpp)
include_directories(include)
```
这里，`include_directories(include)` 命令添加了 `include` 目录到编译器的搜索路径中，因此在 `src/main.cpp` 中可以直接包含 `#include "my_library.h"`。

### 2. `catkin_package(INCLUDE_DIRS include)`

**功能描述**：
`catkin_package()` 是catkin提供的宏，用于声明ROS包的配置信息，其中 `INCLUDE_DIRS` 参数用于指定该包提供的头文件目录。这些目录将被用于其他依赖于此包的项目。

**使用范围**：
当其他包在其 `CMakeLists.txt` 文件中使用 `find_package()` 查找当前包时，通过 `catkin_package(INCLUDE_DIRS include)` 指定的目录会被自动添加到这些依赖包的编译器包含路径中。这样，依赖包就可以找到并包含当前包的头文件。

**示例**：
假设有两个ROS包 `my_library` 和 `my_executable`，其中 `my_executable` 依赖于 `my_library`。

`my_library/CMakeLists.txt`:
```cmake
project(my_library)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES my_library
)
add_library(my_library src/my_library.cpp)
include_directories(include)
```

`my_executable/CMakeLists.txt`:
```cmake
project(my_executable)
find_package(catkin REQUIRED COMPONENTS my_library)
include_directories(${catkin_INCLUDE_DIRS})
add_executable(my_executable src/main.cpp)
```
在这个示例中，`my_library` 包使用 `catkin_package(INCLUDE_DIRS include)` 声明它提供头文件的目录。当 `my_executable` 包使用 `find_package()` 寻找 `my_library` 时，`my_library` 的头文件目录自动被添加到 `my_executable` 的编译器包含路径中。

### 总结

`include_directories()` 直接影响当前CMake项目，为其添加编译时的头文件搜索路径，而 `catkin_package(INCLUDE_DIRS include)` 用于指定当其他包找到此包时，应该添加哪些头文件路径，从而支持跨包的依赖和模块化构建。在实际应用中，通常需要根据项目的具体需求和结构来灵活使用这两个命令。