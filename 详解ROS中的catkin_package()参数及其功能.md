# 详解ROS中的catkin_package()参数及其功能

在ROS中，`catkin_package()` 是一个非常重要的CMake宏，用于定义包的构建和依赖关系。它可以接受多个参数，以便指定该包的公共接口和依赖信息。以下是对每个主要参数的详细解释及示例。

### 1. INCLUDE_DIRS

#### 描述
`INCLUDE_DIRS` 参数用于声明包的公共头文件目录，使得依赖于此包的其他包可以访问这些头文件。在使用 `find_package()` 找到当前包后，这些包含目录会自动添加到依赖包的编译环境中。

#### 示例
```cmake
catkin_package(
  INCLUDE_DIRS include
)
```
在这个例子中，`include` 目录被声明为公共的，任何依赖于此包的其他包在编译时都会将 `include` 目录添加到它们的头文件搜索路径中。

### 2. LIBRARIES

#### 描述
`LIBRARIES` 参数指定当前包构建的库，以便其他包可以在其自身的构建中链接这些库。这是对外提供的接口，确保其他包能够使用当前包的功能。

#### 示例
```cmake
catkin_package(
  LIBRARIES my_library
)
```
在这个例子中，`my_library` 是当前包构建的库，其他依赖于此包的包可以链接到这个库，从而使用其提供的功能。

### 3. CATKIN_DEPENDS

#### 描述
`CATKIN_DEPENDS` 用于列出当前包依赖的其他Catkin包。这确保在构建当前包之前，所有列出的依赖包都已被构建和配置。它自动处理这些依赖关系，以确保在编译时所有需要的包都可用。

#### 示例
```cmake
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)
```
在这个例子中，当前包依赖于 `roscpp` 和 `std_msgs`。这意味着在编译当前包之前，Catkin系统会确保这两个包已经成功构建。

### 4. DEPENDS

#### 描述
`DEPENDS` 参数指定非Catkin的依赖，如系统库或第三方库（例如Boost、Eigen等）。这些库通常不属于ROS生态系统，但当前包的功能可能需要它们。

#### 示例
```cmake
catkin_package(
  DEPENDS Eigen
)
```
在这个例子中，`Eigen` 是一个常用的线性代数库，当前包依赖于它。此参数确保在构建当前包时，这个库是可用的。

### 5. CFG_EXTRAS

#### 描述
`CFG_EXTRAS` 参数用于指定额外的 `.cmake` 文件，这些文件可以包含额外的宏和函数，用于扩展此包的构建系统配置。这在需要复杂的构建逻辑或需要处理特殊配置时非常有用。

#### 示例
```cmake
catkin_package(
  CFG_EXTRAS my_package_config.cmake
)
```
在这个例子中，`my_package_config.cmake` 是一个自定义的CMake配置文件，它可能定义了一些额外的构建选项或功能，当前包会在构建时加载这个文件。

### 总结

`catkin_package()` 的各个参数共同帮助定义一个ROS包的结构和依赖关系，确保包在构建时可以被正确地解析和处理。这使得ROS生态系统中的包能够相互协作，并实现代码重用。通过合理使用这些参数，开发者可以确保软件包的可用性、可维护性和扩展性。