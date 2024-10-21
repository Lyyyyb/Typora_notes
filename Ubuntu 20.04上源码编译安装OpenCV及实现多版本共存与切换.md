Ubuntu 20.04上源码编译安装OpenCV及实现多版本共存与切换

在Ubuntu 20.04上使用源代码编译安装OpenCV，并实现多版本共存和版本切换的过程涉及多个步骤。这里，我们将以OpenCV 3.2和OpenCV 4.4为例来详细说明这一过程。

### 1. 系统准备和依赖安装

首先，确保系统更新到最新状态，并安装必要的编译工具和依赖库。

```bash
sudo apt update && sudo apt upgrade
sudo apt install build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
```

### 2. 下载OpenCV源码

从OpenCV的GitHub仓库下载所需版本的源代码。可以使用`git clone`命令配合特定的标签（tag）下载特定版本的源代码。

```bash
# 为OpenCV 3.2和OpenCV 4.4创建目录
mkdir ~/opencv_install
cd ~/opencv_install

# 克隆OpenCV 3.2
git clone --branch 3.2.0 https://github.com/opencv/opencv.git opencv-3.2
git clone --branch 3.2.0 https://github.com/opencv/opencv_contrib.git opencv_contrib-3.2

# 克隆OpenCV 4.4
git clone --branch 4.4.0 https://github.com/opencv/opencv.git opencv-4.4
git clone --branch 4.4.0 https://github.com/opencv/opencv_contrib.git opencv_contrib-4.4
```

### 3. 编译和安装OpenCV

对每个版本分别进行编译和安装。这里使用单独的安装目录以支持版本共存。

```bash
# 编译和安装OpenCV 3.2
cd ~/opencv_install/opencv-3.2
mkdir build && cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr/local/opencv-3.2 \
      -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-3.2/modules ..
make -j$(nproc)
sudo make install

# 编译和安装OpenCV 4.4
cd ~/opencv_install/opencv-4.4
mkdir build && cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr/local/opencv-4.4 \
      -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-4.4/modules ..
make -j$(nproc)
sudo make install
```

### 4. 配置环境变量

为了能在两个版本间切换，可以在`~/.bashrc`文件中添加对应的环境变量配置，并根据需要注释或取消注释来选择特定版本。

```bash
# 打开~/.bashrc文件
nano ~/.bashrc

# 添加下列配置（先注释OpenCV 4.4的配置）
# OpenCV 3.2
export PATH=/usr/local/opencv-3.2/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/opencv-3.2/lib:$LD_LIBRARY_PATH
export PKG_CONFIG_PATH=/usr/local/opencv-3.2/lib/pkgconfig

# OpenCV 4.4
#export PATH=/usr/local/opencv-4.4/bin:$PATH
#export LD_LIBRARY_PATH=/usr/local/opencv-4.4/lib:$LD_LIBRARY_PATH
#export PKG_CONFIG_PATH=/usr/local/opencv-4.4/lib/pkgconfig
```

### 5. 应用环境配置

在修改了`~/.bashrc`之后，使用下列命令使更改生效：

```bash
source ~/.bashrc
```

### 6. 验证安装

使用`pkg-config`或者直接调用`opencv_version`来验证当前激活的OpenCV版本。

```bash
pkg-config --modversion opencv
opencv_version
```

通过这些步骤，你可以在同一台机器上安装并维护多个版本的OpenCV，并根据项目需求灵活切换。这种方法同样适用于其他需要多版本共存的库。