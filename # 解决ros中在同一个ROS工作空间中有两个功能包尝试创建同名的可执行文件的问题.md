# 解决ros中在同一个ROS工作空间中有两个功能包尝试创建同名的可执行文件的问题

遇到这个问题说明你在同一个ROS工作空间中有两个功能包尝试创建同名的可执行文件。在CMake中，每个可执行文件的目标名称在整个项目（这里指整个工作空间）中必须是唯一的。即使这些可执行文件位于不同的包中，如果它们的目标名称相同，CMake 也会报错，因为这违反了CMake的策略 CMP0002，该策略要求目标名称全局唯一。

### 解决方案

要解决这个问题，你有几个选项：

#### 1. **重命名其中一个可执行文件**

这是最简单和直接的解决方案。在其中一个`CMakeLists.txt`文件中更改`add_executable`的目标名称。

例如，如果你的两个包中都有 `add_executable(joy_to_cmd_vel_node ...)`，你可以在其中一个包中更改它：

在 `serial_test/CMakeLists.txt`，修改为：
```cmake
add_executable(joy_to_cmd_vel_node_serial_test ...)
```

并确保所有相关的`target_link_libraries`和其他CMake命令也使用新的目标名称。

#### 2. **使用CMake的ALIAS或IMPORTED目标**

对于更复杂的项目，如果出于某些原因你想保持可执行文件的命名不变，可以考虑使用CMake的高级特性，如ALIAS或IMPORTED目标，但这通常用于库而不是可执行文件。

#### 3. **组织结构调整**

考虑是否真的需要在两个不同的包中有同名的可执行文件。如果这两个节点执行的功能非常相似，也许可以合并这些包，或者至少可以共享一些代码。

#### 4. **使用条件编译**

如果你由于某些原因需要在不同情况下编译同名的不同程序，可以考虑使用预处理器指令或CMake选项来条件性地编译不同的代码段。

例如，在CMake中设置一个选项：

```cmake
option(USE_SERIAL_TEST "Build the serial_test version of joy_to_cmd_vel_node" OFF)

if(USE_SERIAL_TEST)
  add_executable(joy_to_cmd_vel_node ...)
else()
  add_executable(joy_to_cmd_vel_node ...)
endif()
```

然后在编译时通过 `-DUSE_SERIAL_TEST=ON` 或 `-DUSE_SERIAL_TEST=OFF` 来选择不同的版本。

### 推荐做法

通常，**重命名可执行文件**是解决这类问题的最简单和最直接的方法。这不仅解决了构建错误，还可以帮助维护代码的清晰度和组织性，使得未来的代码管理和错误追踪更为简便。