# 解决Ubuntu 20.04上编译OpenCV 3.2时遇到的`stdlib.h`缺失错误

您在 Ubuntu 20.04 上编译 OpenCV 3.2 时遇到的错误与 C++ 标准库的头文件配置问题有关。错误消息指出系统无法找到 `<stdlib.h>`，这通常与预编译头文件的处理、GCC 版本或者头文件搜索路径有关。下面我将详细分析问题原因并提供具体的解决方案。

### 错误分析

错误消息：

```
/usr/include/c++/9/cstdlib:75:15: fatal error: stdlib.h: 没有那个文件或目录
 #include_next <stdlib.h>
               ^~~~~~~~~~
```

这个错误提示编译器在尝试包含 `<stdlib.h>` 时未能找到该文件。问题通常源于以下几个方面：

1. **编译器版本与库不兼容**：Ubuntu 20.04 默认安装的 GCC 版本较新（GCC 9），而 OpenCV 3.2 是一个较老的版本，可能与新版本的编译器存在兼容性问题。
2. **预编译头文件（PCH）问题**：使用预编译头文件时，GCC 对头文件的处理可能会引起路径解析问题，特别是在升级系统或编译器后。
3. **系统头文件路径配置问题**：可能是由于系统更新或其他软件安装过程中导致的头文件路径配置错误。

### 解决方案

#### 1. 禁用预编译头文件

如之前所述，禁用预编译头文件可以解决由于头文件路径不正确引起的编译错误。这通常是最直接的解决方案。

**操作步骤**：

1. 打开终端并进入 OpenCV 的编译目录（假设是 `build`）：

   ```bash
   cd /path/to/opencv-3.2/build
   ```

2. 运行 CMake 配置命令，并禁用预编译头文件：

   ```bash
   cmake -D ENABLE_PRECOMPILED_HEADERS=OFF ..
   ```

3. 清理并重新编译：

   ```bash
   make clean
   make
   ```

4. 如果编译成功，安装 OpenCV：

   ```bash
   sudo make install
   ```

#### 2. 使用较旧的 GCC 版本

如果禁用预编译头文件后仍出现问题，考虑使用与 OpenCV 3.2 更兼容的旧版本 GCC 编译器。

**操作步骤**：

1. 安装较低版本的 GCC（例如 GCC 7）：

   ```bash
   sudo apt install gcc-7 g++-7
   ```

2. 更新 CMake 使用的编译器版本：

   ```bash
   cmake -D CMAKE_C_COMPILER=gcc-7 -D CMAKE_CXX_COMPILER=g++-7 -D ENABLE_PRECOMPILED_HEADERS=OFF ..
   ```

3. 重新编译 OpenCV：

   ```bash
   make clean
   make
   ```

4. 安装：

   ```bash
   sudo make install
   ```

#### 3. 检查和修复系统头文件路径

如果上述方法均不奏效，可能需要检查系统的头文件路径配置。

1. 确认 `stdlib.h` 存在于预期的系统路径中（通常位于 `/usr/include/stdlib.h` 或 `/usr/include/x86_64-linux-gnu/stdlib.h`）。
2. 如果找不到，可能需要重新安装 `libc` 开发包：

   ```bash
   sudo apt install --reinstall libc6-dev
   ```

### 总结

遇到编译错误时，常见的解决方法包括调整编译设置（如禁用预编译头文件）、使用与软件更兼容的工具版本（如较旧的 GCC），或确保系统环境配置正确。根据错误的具体情况选择合适的方法，并进行适当的测试。如果错误依旧，可能需要更深入地检查和调整系统环境或求助于开源社区。