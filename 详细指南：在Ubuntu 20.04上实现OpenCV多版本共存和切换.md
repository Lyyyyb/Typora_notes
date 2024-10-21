# 详细指南：在Ubuntu 20.04上实现OpenCV多版本共存和切换

在Ubuntu 20.04上实现OpenCV多版本共存和切换的过程要求对系统的环境配置有精确的控制，以确保不同的项目可以根据需要访问特定版本的OpenCV。下面是如何实现这一目标的详细步骤和实例：

### 实现步骤

#### 1. 安装多个版本的OpenCV

**a. 安装依赖：**  
首先，确保安装了所有必要的依赖项。这可以通过运行以下命令来完成：

```bash
sudo apt update
sudo apt install build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
```

**b. 下载和编译OpenCV源码：**  
为每个版本创建一个独立的目录并编译安装。

```bash
# 创建目录存放OpenCV 2.4.9
mkdir ~/opencv-2.4.9 && cd ~/opencv-2.4.9
wget https://github.com/opencv/opencv/archive/2.4.9.zip
unzip 2.4.9.zip
cd opencv-2.4.9
mkdir build && cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr/local/opencv2.4.9 ..
make -j$(nproc)
sudo make install

# 创建目录存放OpenCV 3.1.0
mkdir ~/opencv-3.1.0 && cd ~/opencv-3.1.0
wget https://github.com/opencv/opencv/archive/3.1.0.zip
unzip 3.1.0.zip
cd opencv-3.1.0
mkdir build && cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr/local/opencv3.1.0 ..
make -j$(nproc)
sudo make install
```

#### 2. 配置环境变量以切换版本

**a. 编辑`.bashrc`：**  
在`~/.bashrc`文件中添加函数来切换环境变量。

```bash
# 打开.bashrc文件进行编辑
nano ~/.bashrc

# 添加以下内容到文件末尾
switch_opencv() {
    version=$1
    if [ $version = "2.4.9" ]; then
        export PATH=/usr/local/opencv2.4.9/bin:$PATH
        export LD_LIBRARY_PATH=/usr/local/opencv2.4.9/lib:$LD_LIBRARY_PATH
        export PKG_CONFIG_PATH=/usr/local/opencv2.4.9/lib/pkgconfig
    elif [ $version = "3.1.0" ]; then
        export PATH=/usr/local/opencv3.1.0/bin:$PATH
        export LD_LIBRARY_PATH=/usr/local/opencv3.1.0/lib:$LD_LIBRARY_PATH
        export PKG_CONFIG_PATH=/usr/local/opencv3.1.0/lib/pkgconfig
    else
        echo "Version not supported"
    fi
}

# 使更改生效
source ~/.bashrc
```

使用方法：在命令行中输入`switch_opencv 2.4.9`或`switch_opencv 3.1.0`来切换不同的OpenCV版本。

#### 3. 编译项目时指定OpenCV版本

在CMake项目中，可以指定`CMAKE_PREFIX_PATH`来使用特定版本的OpenCV。

**CMakeLists.txt 示例：**

```cmake
cmake_minimum_required(VERSION 3.10)
project(OpenCVProject)

# Set this to the path where you've installed the desired OpenCV version
set(OpenCV_DIR /usr/local/opencv3.1.0/share/OpenCV)

find_package(OpenCV REQUIRED)

add_executable(opencv_test main.cpp)
target_link_libraries(opencv_test ${OpenCV_LIBS})
```

这样，每次编译之前，确保运行`switch_opencv`以设置正确的环境变量，使得`CMAKE_PREFIX_PATH`可以指向正确的OpenCV安装路径。

### 总结

在Ubuntu 20.04上实现OpenCV多版本共存和切换涉及安装不同版本到独立路径、配置环境变量以方便地切换这些版本，以及在项目编译时明确指定使用哪个版本。这样的设置使得开发人员能够针对不同的项目需求，选择适当的库版本，增加项目的兼容性和灵活性。通过上述详细步骤，开发者可以有效管理多个库版本，确保开发环境的整洁与高效。