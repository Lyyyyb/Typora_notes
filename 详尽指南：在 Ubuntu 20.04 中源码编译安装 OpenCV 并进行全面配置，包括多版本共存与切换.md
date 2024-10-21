# 详尽指南：在 Ubuntu 20.04 中源码编译安装 OpenCV 并进行全面配置，包括多版本共存与切换

本文档旨在为开发者提供在 Ubuntu 20.04 系统中通过源码编译安装 OpenCV 的详细步骤，并涵盖后续的各项配置，包括 `pkg-config` 环境、系统环境变量配置、动态库环境配置，以及实现 OpenCV 多版本（以 OpenCV 3.2 和 OpenCV 4.4 为例）的共存与切换。以下内容将以专业、严谨且逻辑清晰的方式展开，辅以具体实例以助理解和操作。

## 目录

1. [前置条件与依赖安装](#1-前置条件与依赖安装)
2. [下载 OpenCV 源代码](#2-下载-opencv-源代码)
3. [编译并安装 OpenCV](#3-编译并安装-opencv)
4. [配置 pkg-config 环境](#4-配置-pkg-config-环境)
5. [配置系统环境变量](#5-配置系统环境变量)
6. [配置动态链接库](#6-配置动态链接库)
7. [实现 OpenCV 多版本共存与切换](#7-实现-opencv-多版本共存与切换)
8. [实例演示](#8-实例演示)
9. [总结](#9-总结)

---

## 1. 前置条件与依赖安装

在开始之前，确保您的 Ubuntu 20.04 系统已更新并安装了必要的编译工具和依赖库。

### 1.1 更新系统

```bash
sudo apt update
sudo apt upgrade -y
```

### 1.2 安装编译工具与依赖库

OpenCV 的编译需要多个依赖库，以下命令将安装常用的依赖：

```bash
sudo apt install -y build-essential cmake git pkg-config \
libjpeg-dev libpng-dev libtiff-dev \
libavcodec-dev libavformat-dev libswscale-dev \
libv4l-dev libxvidcore-dev libx264-dev \
libgtk-3-dev libatlas-base-dev gfortran \
python3-dev
```

### 1.3 安装可选依赖

根据需要，您可以安装额外的模块，如 OpenGL 支持、FFmpeg 等：

```bash
sudo apt install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
```

## 2. 下载 OpenCV 源代码

为了实现多版本共存，建议将不同版本的 OpenCV 源代码分别克隆到不同的目录中。

### 2.1 创建工作目录

```bash
mkdir -p ~/opencv_builds
cd ~/opencv_builds
```

### 2.2 克隆 OpenCV 主仓库与 `opencv_contrib` 仓库

#### 克隆 OpenCV 3.2

```bash
git clone https://github.com/opencv/opencv.git opencv-3.2
cd opencv-3.2
git checkout 3.2.0
cd ..
```

#### 克隆 OpenCV 4.4

```bash
git clone https://github.com/opencv/opencv.git opencv-4.4
cd opencv-4.4
git checkout 4.4.0
cd ..
```

#### 克隆 `opencv_contrib` 仓库

`opencv_contrib` 包含额外的模块，需与对应版本的 OpenCV 配套使用。

##### 对于 OpenCV 3.2

```bash
git clone https://github.com/opencv/opencv_contrib.git opencv_contrib-3.2
cd opencv_contrib-3.2
git checkout 3.2.0
cd ..
```

##### 对于 OpenCV 4.4

```bash
git clone https://github.com/opencv/opencv_contrib.git opencv_contrib-4.4
cd opencv_contrib-4.4
git checkout 4.4.0
cd ..
```

## 3. 编译并安装 OpenCV

为每个版本的 OpenCV 单独创建构建目录并编译安装，以实现多版本共存。

### 3.1 编译安装 OpenCV 3.2

#### 3.1.1 创建构建目录

```bash
cd ~/opencv_builds/opencv-3.2
mkdir build && cd build
```

#### 3.1.2 配置 CMake

```bash
cmake -D CMAKE_BUILD_TYPE=Release \
      -D CMAKE_INSTALL_PREFIX=/usr/local/opencv-3.2 \
      -D OPENCV_EXTRA_MODULES_PATH=~/opencv_builds/opencv_contrib-3.2/modules \
      -D BUILD_EXAMPLES=ON ..
```

**参数说明：**

- `CMAKE_BUILD_TYPE=Release`：编译为优化后的发布版本。
- `CMAKE_INSTALL_PREFIX=/usr/local/opencv-3.2`：指定安装路径，以便与其他版本隔离。
- `OPENCV_EXTRA_MODULES_PATH`：指定 `opencv_contrib` 模块路径。
- `BUILD_EXAMPLES=ON`：编译示例代码（可选）。

#### 3.1.3 编译并安装

```bash
make -j$(nproc)
sudo make install
```

### 3.2 编译安装 OpenCV 4.4

#### 3.2.1 创建构建目录

```bash
cd ~/opencv_builds/opencv-4.4
mkdir build && cd build
```

#### 3.2.2 配置 CMake

```bash
cmake -D CMAKE_BUILD_TYPE=Release \
      -D CMAKE_INSTALL_PREFIX=/usr/local/opencv-4.4 \
      -D OPENCV_EXTRA_MODULES_PATH=~/opencv_builds/opencv_contrib-4.4/modules \
      -D BUILD_EXAMPLES=ON ..
```

#### 3.2.3 编译并安装

```bash
make -j$(nproc)
sudo make install
```

## 4. 配置 pkg-config 环境

`pkg-config` 是一个用于管理编译时库依赖的工具。配置 `pkg-config` 能够帮助编译器和链接器找到 OpenCV 库。

### 4.1 确认 `pkg-config` 安装

```bash
sudo apt install -y pkg-config
```

### 4.2 设置 `PKG_CONFIG_PATH`

需要将每个 OpenCV 版本的 `.pc` 文件路径添加到 `PKG_CONFIG_PATH` 环境变量中。

#### 4.2.1 编辑 Shell 配置文件

使用您常用的文本编辑器（如 `nano`）编辑 `~/.bashrc` 或 `~/.zshrc` 文件：

```bash
nano ~/.bashrc
```

#### 4.2.2 添加 OpenCV 3.2 的 `pkg-config` 路径

```bash
# OpenCV 3.2 pkg-config path
export PKG_CONFIG_PATH=/usr/local/opencv-3.2/lib/pkgconfig:$PKG_CONFIG_PATH
```

#### 4.2.3 添加 OpenCV 4.4 的 `pkg-config` 路径

```bash
# OpenCV 4.4 pkg-config path
export PKG_CONFIG_PATH=/usr/local/opencv-4.4/lib/pkgconfig:$PKG_CONFIG_PATH
```

#### 4.2.4 应用更改

保存并关闭编辑器后，执行以下命令以应用更改：

```bash
source ~/.bashrc
```

### 4.3 验证 `pkg-config` 配置

验证 OpenCV 3.2：

```bash
pkg-config --modversion opencv
```

输出应为 `3.2.0`。

验证 OpenCV 4.4：

```bash
pkg-config --modversion opencv4
```

输出应为 `4.4.0`。

## 5. 配置系统环境变量

配置系统环境变量确保系统能够在运行时找到 OpenCV 库和头文件。

### 5.1 设置环境变量

继续编辑 `~/.bashrc` 或 `~/.zshrc` 文件：

```bash
nano ~/.bashrc
```

### 5.2 添加 OpenCV 3.2 和 4.4 的环境变量

```bash
# OpenCV 3.2 environment variables
export OpenCV3_DIR=/usr/local/opencv-3.2
export PATH=$OpenCV3_DIR/bin:$PATH
export LD_LIBRARY_PATH=$OpenCV3_DIR/lib:$LD_LIBRARY_PATH
export CPLUS_INCLUDE_PATH=$OpenCV3_DIR/include:$CPLUS_INCLUDE_PATH

# OpenCV 4.4 environment variables
export OpenCV4_DIR=/usr/local/opencv-4.4
export PATH=$OpenCV4_DIR/bin:$PATH
export LD_LIBRARY_PATH=$OpenCV4_DIR/lib:$LD_LIBRARY_PATH
export CPLUS_INCLUDE_PATH=$OpenCV4_DIR/include:$CPLUS_INCLUDE_PATH
```

### 5.3 应用更改

```bash
source ~/.bashrc
```

### 5.4 验证环境变量

检查环境变量是否正确配置：

```bash
echo $PATH
echo $LD_LIBRARY_PATH
echo $CPLUS_INCLUDE_PATH
```

确保 `/usr/local/opencv-3.2/bin` 和 `/usr/local/opencv-4.4/bin` 在 `PATH` 中，且相应的 `lib` 和 `include` 路径在 `LD_LIBRARY_PATH` 和 `CPLUS_INCLUDE_PATH` 中。

## 6. 配置动态链接库

配置动态链接库确保运行时能找到正确的 OpenCV 版本。

### 6.1 创建配置文件

为每个 OpenCV 版本创建一个单独的配置文件，以便系统动态链接库可以正确识别。

#### 6.1.1 OpenCV 3.2

创建 `/etc/ld.so.conf.d/opencv3.conf` 文件：

```bash
sudo nano /etc/ld.so.conf.d/opencv3.conf
```

添加以下内容：

```
/usr/local/opencv-3.2/lib
```

#### 6.1.2 OpenCV 4.4

创建 `/etc/ld.so.conf.d/opencv4.conf` 文件：

```bash
sudo nano /etc/ld.so.conf.d/opencv4.conf
```

添加以下内容：

```
/usr/local/opencv-4.4/lib
```

### 6.2 更新动态链接库缓存

```bash
sudo ldconfig
```

### 6.3 验证动态链接库配置

使用 `ldconfig` 查看库是否被正确识别：

```bash
ldconfig -p | grep opencv
```

应显示 OpenCV 3.2 和 4.4 的库路径。

## 7. 实现 OpenCV 多版本共存与切换

为了实现 OpenCV 3.2 和 4.4 的共存与切换，需要合理配置环境变量和使用不同的编译选项。

### 7.1 使用环境变量切换版本

通过设置环境变量，可以在不同的终端会话中使用不同的 OpenCV 版本。

#### 7.1.1 切换到 OpenCV 3.2

在终端中执行：

```bash
export PATH=/usr/local/opencv-3.2/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/opencv-3.2/lib:$LD_LIBRARY_PATH
export PKG_CONFIG_PATH=/usr/local/opencv-3.2/lib/pkgconfig:$PKG_CONFIG_PATH
```

#### 7.1.2 切换到 OpenCV 4.4

在终端中执行：

```bash
export PATH=/usr/local/opencv-4.4/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/opencv-4.4/lib:$LD_LIBRARY_PATH
export PKG_CONFIG_PATH=/usr/local/opencv-4.4/lib/pkgconfig:$PKG_CONFIG_PATH
```

### 7.2 使用 `update-alternatives` 管理多个版本

`update-alternatives` 是一个用于管理多个同类程序的工具，可以用来切换不同的 OpenCV 版本。

#### 7.2.1 注册 OpenCV 3.2 和 4.4 到 `update-alternatives`

##### 注册 `opencv_version`

```bash
sudo update-alternatives --install /usr/bin/opencv_version opencv_version /usr/local/opencv-3.2/bin/opencv_version 32
sudo update-alternatives --install /usr/bin/opencv_version opencv_version /usr/local/opencv-4.4/bin/opencv_version 44
```

##### 注册其他可执行文件（如有）

根据需要，注册其他 OpenCV 可执行文件：

```bash
sudo update-alternatives --install /usr/bin/opencv_interactive-calibration opencv_interactive-calibration /usr/local/opencv-3.2/bin/opencv_interactive-calibration 32
sudo update-alternatives --install /usr/bin/opencv_interactive-calibration opencv_interactive-calibration /usr/local/opencv-4.4/bin/opencv_interactive-calibration 44

# 重复以上步骤为其他可执行文件注册
```

#### 7.2.2 切换 OpenCV 版本

使用以下命令选择默认版本：

```bash
sudo update-alternatives --config opencv_version
```

系统将显示可用版本，输入对应的编号选择。

#### 7.2.3 验证切换结果

```bash
opencv_version
```

应显示选择的 OpenCV 版本号。

### 7.3 使用 CMake 配置不同版本

在编译项目时，可以通过 `pkg-config` 或手动指定 OpenCV 路径来选择不同版本。

#### 7.3.1 使用 `pkg-config`

在项目的 `CMakeLists.txt` 中使用 `pkg-config`：

```cmake
find_package(PkgConfig REQUIRED)
pkg_check_modules(OPENCV REQUIRED opencv4)  # 或 opencv

include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})

add_executable(my_app main.cpp)
target_link_libraries(my_app ${OPENCV_LIBRARIES})
```

通过切换 `PKG_CONFIG_PATH` 环境变量，可以选择不同版本的 OpenCV。

#### 7.3.2 手动指定 OpenCV 路径

在 CMake 配置命令中手动指定 OpenCV 路径：

```bash
# 使用 OpenCV 3.2
cmake -D CMAKE_BUILD_TYPE=Release \
      -D CMAKE_INSTALL_PREFIX=/usr/local/my_project \
      -D OpenCV_DIR=/usr/local/opencv-3.2/lib/cmake/opencv4 \
      ..

# 使用 OpenCV 4.4
cmake -D CMAKE_BUILD_TYPE=Release \
      -D CMAKE_INSTALL_PREFIX=/usr/local/my_project \
      -D OpenCV_DIR=/usr/local/opencv-4.4/lib/cmake/opencv4 \
      ..
```

## 8. 实例演示

以下将通过一个具体的实例，展示如何编译一个使用 OpenCV 3.2 和 OpenCV 4.4 的简单 C++ 程序，并在不同版本之间切换。

### 8.1 创建示例程序

创建一个名为 `main.cpp` 的简单程序，加载并显示一张图片：

```cpp
#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    cv::Mat image = cv::imread("example.jpg", cv::IMREAD_COLOR);
    if (image.empty()) {
        std::cerr << "Could not open or find the image." << std::endl;
        return -1;
    }
    cv::imshow("Display window", image);
    cv::waitKey(0);
    return 0;
}
```

确保当前目录中有一张名为 `example.jpg` 的图片。

### 8.2 编译并运行使用 OpenCV 3.2

#### 8.2.1 设置环境变量为 OpenCV 3.2

```bash
export PATH=/usr/local/opencv-3.2/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/opencv-3.2/lib:$LD_LIBRARY_PATH
export PKG_CONFIG_PATH=/usr/local/opencv-3.2/lib/pkgconfig:$PKG_CONFIG_PATH
```

#### 8.2.2 编译程序

```bash
g++ main.cpp $(pkg-config --cflags --libs opencv) -o main_opencv3
```

#### 8.2.3 运行程序

```bash
./main_opencv3
```

应显示加载的图片，且通过 `opencv_version` 命令确认版本为 3.2：

```bash
opencv_version
# 输出: 3.2.0
```

### 8.3 编译并运行使用 OpenCV 4.4

#### 8.3.1 设置环境变量为 OpenCV 4.4

```bash
export PATH=/usr/local/opencv-4.4/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/opencv-4.4/lib:$LD_LIBRARY_PATH
export PKG_CONFIG_PATH=/usr/local/opencv-4.4/lib/pkgconfig:$PKG_CONFIG_PATH
```

#### 8.3.2 编译程序

```bash
g++ main.cpp $(pkg-config --cflags --libs opencv4) -o main_opencv4
```

#### 8.3.3 运行程序

```bash
./main_opencv4
```

应显示加载的图片，且通过 `opencv_version` 命令确认版本为 4.4：

```bash
opencv_version
# 输出: 4.4.0
```

### 8.4 使用 `update-alternatives` 切换版本

假设已按照前述步骤注册 `opencv_version`，可通过 `update-alternatives` 轻松切换默认版本。

#### 8.4.1 切换到 OpenCV 3.2

```bash
sudo update-alternatives --config opencv_version
# 选择对应 OpenCV 3.2 的编号
```

验证版本：

```bash
opencv_version
# 输出: 3.2.0
```

#### 8.4.2 切换到 OpenCV 4.4

```bash
sudo update-alternatives --config opencv_version
# 选择对应 OpenCV 4.4 的编号
```

验证版本：

```bash
opencv_version
# 输出: 4.4.0
```

## 9. 总结

通过本文的详细步骤，您已成功在 Ubuntu 20.04 系统中通过源码编译安装了 OpenCV 3.2 和 OpenCV 4.4，并实现了多版本的共存与切换。以下是关键点的总结：

- **源码编译与安装**：通过源码编译可自定义安装路径，实现多版本共存。
- **环境变量配置**：正确设置 `PKG_CONFIG_PATH`、`LD_LIBRARY_PATH` 和 `CPLUS_INCLUDE_PATH` 确保编译器和运行时环境能找到相应的库和头文件。
- **动态链接库配置**：通过 `/etc/ld.so.conf.d/` 添加库路径，并运行 `ldconfig` 更新缓存，确保运行时能正确加载库。
- **多版本管理**：利用 `update-alternatives` 和环境变量切换不同版本的 OpenCV，满足不同项目需求。

通过这种方式，您可以灵活地在同一系统中管理和使用多个 OpenCV 版本，确保项目的兼容性和开发效率。

---

**注意事项：**

1. **权限管理**：编译和安装过程需要管理员权限，确保使用 `sudo` 执行必要的命令。
2. **版本兼容性**：不同版本的 OpenCV 可能在 API 和功能上有所不同，切换版本后需确保项目代码与所用版本兼容。
3. **依赖库冲突**：安装多版本 OpenCV 时，需注意依赖库的版本兼容性，避免引入冲突。
4. **清理残留文件**：如需卸载某一版本的 OpenCV，需手动删除对应的安装路径下的文件，并更新环境变量和动态链接库缓存。

通过严格遵循本文档的步骤，您将能够在 Ubuntu 20.04 系统中高效、灵活地管理 OpenCV 的多个版本，并为复杂的计算机视觉项目提供坚实的基础。