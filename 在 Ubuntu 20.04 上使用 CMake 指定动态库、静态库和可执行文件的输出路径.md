# 在 Ubuntu 20.04 上使用 CMake 指定动态库、静态库和可执行文件的输出路径

在软件开发过程中，合理配置构建产物的输出路径对于项目的组织、维护和部署具有重要意义。CMake 作为一个跨平台的构建系统，提供了灵活的方法来指定动态库（共享库）、静态库以及可执行文件的输出路径。本文将详细介绍如何在 Ubuntu 20.04 环境下使用 CMake 指定这些输出路径，并通过具体示例进行说明。

## 1. CMake 输出路径概述

CMake 提供了多种方式来指定构建产物的输出路径，主要包括全局变量和目标属性两种方法。

### 1.1 全局变量

全局变量适用于整个项目，设置一次即可影响所有相关的构建产物。然而，对于大型或复杂项目，全局变量可能不够灵活。

- **`CMAKE_RUNTIME_OUTPUT_DIRECTORY`**：指定所有可执行文件（Runtime）的输出目录。
- **`CMAKE_LIBRARY_OUTPUT_DIRECTORY`**：指定所有动态库（Library）的输出目录。
- **`CMAKE_ARCHIVE_OUTPUT_DIRECTORY`**：指定所有静态库（Archive）的输出目录。

### 1.2 目标属性

目标属性允许为特定的构建目标单独指定输出路径，提供了更高的灵活性和控制力。推荐在现代 CMake 项目中使用目标属性来管理输出路径。

- **`RUNTIME_OUTPUT_DIRECTORY`**：针对可执行文件。
- **`LIBRARY_OUTPUT_DIRECTORY`**：针对动态库。
- **`ARCHIVE_OUTPUT_DIRECTORY`**：针对静态库。

## 2. 使用目标属性指定输出路径

以下示例将展示如何在 `CMakeLists.txt` 中为动态库、静态库和可执行文件指定输出路径。

### 2.1 示例项目结构

假设有一个名为 `MyProject` 的项目，其目录结构如下：

```
MyProject/
├── CMakeLists.txt
├── src/
│   ├── main.cpp
│   ├── mylib.cpp
│   └── mylib_static.cpp
└── include/
    └── mylib.h
```

### 2.2 `CMakeLists.txt` 示例

```cmake
cmake_minimum_required(VERSION 3.10)
project(MyProject VERSION 1.0 LANGUAGES CXX)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 指定全局输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# 包含头文件目录
include_directories(${PROJECT_SOURCE_DIR}/include)

# 添加动态库
add_library(mylib SHARED src/mylib.cpp)

# 为 mylib 目标指定输出路径（覆盖全局设置）
set_target_properties(mylib PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/custom_lib
)

# 添加静态库
add_library(mylib_static STATIC src/mylib_static.cpp)

# 为 mylib_static 目标指定输出路径（覆盖全局设置）
set_target_properties(mylib_static PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/custom_lib_static
)

# 添加可执行文件
add_executable(myapp src/main.cpp)

# 为 myapp 目标指定输出路径（覆盖全局设置）
set_target_properties(myapp PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/custom_bin
)

# 链接动态库和静态库
target_link_libraries(myapp PRIVATE mylib mylib_static)
```

### 2.3 代码解释

1. **项目基本设置**：
   - `cmake_minimum_required(VERSION 3.10)` 和 `project(MyProject VERSION 1.0 LANGUAGES CXX)` 定义了 CMake 的最低版本要求、项目名称、版本及所用语言。
   - `set(CMAKE_CXX_STANDARD 17)` 和 `set(CMAKE_CXX_STANDARD_REQUIRED True)` 指定了 C++17 标准并强制要求使用该标准。

2. **全局输出路径设置**：
   - `CMAKE_RUNTIME_OUTPUT_DIRECTORY` 设置所有可执行文件的默认输出目录为 `${CMAKE_BINARY_DIR}/bin`。
   - `CMAKE_LIBRARY_OUTPUT_DIRECTORY` 设置所有动态库的默认输出目录为 `${CMAKE_BINARY_DIR}/lib`。
   - `CMAKE_ARCHIVE_OUTPUT_DIRECTORY` 设置所有静态库的默认输出目录为 `${CMAKE_BINARY_DIR}/lib`。

3. **包含目录**：
   - `include_directories(${PROJECT_SOURCE_DIR}/include)` 指定了头文件所在的目录，便于编译器查找。

4. **添加动态库**：
   - `add_library(mylib SHARED src/mylib.cpp)` 创建一个名为 `mylib` 的共享库。
   - `set_target_properties(mylib PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/custom_lib)` 为 `mylib` 目标单独指定输出路径为 `${CMAKE_BINARY_DIR}/custom_lib`，覆盖全局设置。

5. **添加静态库**：
   - `add_library(mylib_static STATIC src/mylib_static.cpp)` 创建一个名为 `mylib_static` 的静态库。
   - `set_target_properties(mylib_static PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/custom_lib_static)` 为 `mylib_static` 目标单独指定输出路径为 `${CMAKE_BINARY_DIR}/custom_lib_static`，覆盖全局设置。

6. **添加可执行文件**：
   - `add_executable(myapp src/main.cpp)` 创建一个名为 `myapp` 的可执行文件。
   - `set_target_properties(myapp PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/custom_bin)` 为 `myapp` 目标单独指定输出路径为 `${CMAKE_BINARY_DIR}/custom_bin`，覆盖全局设置。

7. **链接库文件**：
   - `target_link_libraries(myapp PRIVATE mylib mylib_static)` 将 `mylib` 动态库和 `mylib_static` 静态库链接到 `myapp` 可执行文件。

### 2.4 项目源代码示例

为了完整性，以下是简单的源代码示例：

**`src/mylib.h`**
```cpp
#ifndef MYLIB_H
#define MYLIB_H

void hello();

#endif // MYLIB_H
```

**`src/mylib.cpp`**
```cpp
#include "mylib.h"
#include <iostream>

void hello() {
    std::cout << "Hello from dynamic library!" << std::endl;
}
```

**`src/mylib_static.cpp`**
```cpp
#include "mylib.h"
#include <iostream>

void hello_static() {
    std::cout << "Hello from static library!" << std::endl;
}
```

**`src/main.cpp`**
```cpp
#include "mylib.h"

int main() {
    hello();
    hello_static();
    return 0;
}
```

### 2.5 构建步骤

在项目根目录下执行以下命令进行构建：

```bash
mkdir build
cd build
cmake ..
make
```

构建完成后，目录结构如下：

```
MyProject/
├── build/
│   ├── bin/
│   │   └── myapp
│   ├── custom_bin/
│   │   └── myapp
│   ├── custom_lib/
│   │   └── libmylib.so
│   ├── custom_lib_static/
│   │   └── libmylib_static.a
│   └── CMakeFiles/
├── src/
│   ├── main.cpp
│   ├── mylib.cpp
│   └── mylib_static.cpp
└── include/
    └── mylib.h
```

- **可执行文件** `myapp` 位于 `build/custom_bin/` 目录。
- **动态库** `libmylib.so` 位于 `build/custom_lib/` 目录。
- **静态库** `libmylib_static.a` 位于 `build/custom_lib_static/` 目录。

## 3. 总结

通过合理配置 CMake 的输出路径，可以有效地管理项目中的动态库、静态库和可执行文件。本文介绍了使用全局变量和目标属性两种方法，其中目标属性方法更为灵活，适用于现代 CMake 项目。通过具体示例，展示了如何在 Ubuntu 20.04 环境下为不同类型的构建目标指定自定义的输出路径，从而提升项目的组织性和可维护性。

在实际项目中，建议优先使用目标属性方法，因为它提供了更高的灵活性，允许为每个构建目标单独配置输出路径，而不会影响其他目标。这种方法特别适用于大型项目或需要复杂输出目录结构的场景。