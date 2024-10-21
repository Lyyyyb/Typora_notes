# 如何在Ubuntu 20.04上编译安装OpenCV 4.4并启用pkg-config支持

在 Ubuntu 20.04 上从源码编译安装 OpenCV 4.4 时遇到没有 `pkg-config` 文件的问题是一个常见的配置问题。这种情况通常发生因为 OpenCV 的默认编译配置没有启用生成 `pkg-config` 文件。以下是详细的问题分析和解决方案，以及具体的实例解释。

### 问题分析

1. **默认配置不生成 `pkg-config` 文件**：
   OpenCV 4 默认的 CMake 配置不生成 `pkg-config`（`.pc`）文件。这意味着即使你成功编译并安装了 OpenCV，系统的 `pkg-config` 工具也无法检测到它，因此无法使用 `pkg-config` 来简化编译和链接过程。

2. **需要显式启用**：
   要生成 `pkg-config` 文件，必须在运行 CMake 配置 OpenCV 时显式启用这一功能。

### 具体解决方案

#### 解决方案一：启用 `pkg-config` 文件生成

在编译 OpenCV 时，通过添加特定的 CMake 选项来启用 `pkg-config` 文件的生成。

**操作步骤**：

1. **清理旧的构建目录（如果之前尝试编译过）**：
   ```bash
   cd /path/to/opencv-4.4/build
   rm -rf *
   ```

2. **配置 CMake**：
   在 OpenCV 的构建目录中运行 CMake，启用 `pkg-config` 的生成：
   ```bash
   cmake -D CMAKE_BUILD_TYPE=Release \
         -D OPENCV_GENERATE_PKGCONFIG=YES \
         -D CMAKE_INSTALL_PREFIX=/usr/local/opencv-4.4 ..
   ```
   这里的选项解释：
   - `-D CMAKE_BUILD_TYPE=Release`: 设置编译类型为 Release，优化性能。
   - `-D OPENCV_GENERATE_PKGCONFIG=YES`: 启用生成 `opencv4.pc` 文件。
   - `-D CMAKE_INSTALL_PREFIX=/usr/local/opencv-4.4`: 指定安装路径，方便管理。

3. **编译和安装**：
   ```bash
   make -j$(nproc)  # 使用所有可用核心加速编译过程
   sudo make install  # 安装 OpenCV
   ```

4. **验证 `pkg-config`**：
   确认 `pkg-config` 能够正确识别 OpenCV：
   ```bash
   pkg-config --modversion opencv4  # 应返回安装的 OpenCV 版本号
   ```

#### 解决方案二：安装预编译的 OpenCV 开发包

如果你不想自己编译 OpenCV，可以直接安装 Ubuntu 的预编译包，这些包已配置好 `pkg-config`。

**操作步骤**：
```bash
sudo apt update
sudo apt install libopencv-dev
```
这将安装 OpenCV 的开发库和头文件，同时设置好 `pkg-config`。

### 实例解释

假设你正在 Ubuntu 20.04 上从源码安装 OpenCV 4.4，并希望使用 `pkg-config` 简化后续的编译任务。按照解决方案一，你需要先下载 OpenCV 的源码，然后进行如上配置和编译。通过这种方式，你不仅控制了安装的具体参数（如版本控制、安装路径等），还能确保系统中的开发环境可以通过 `pkg-config` 快速配置项目。

这种配置尤其有助于那些需要在项目中集成 OpenCV 的开发者，因为它允许通过简单的 `pkg-config` 命令轻松获取编译和链接标志，从而避免了手动配置这些复杂参数的麻烦。