# Ubuntu20.04 ROS 编译运行ORB-SLAM2(以RGBD Orbbec Astra+为例)

## 先决条件

- Ubuntu 20.04

- C++ 11

## 依赖项

### Pangolin(0.6)

- **用途**：Pangolin 是一个用于可视化和用户界面的库。
- **安装指南**：可以在 [Pangolin GitHub 页面](https://github.com/stevenlovegrove/Pangolin) 上找到下载和安装的说明。

### OpenCV(3.2.0)

- **用途**：OpenCV 用于处理图像和特征。
- **版本要求**：至少需要 OpenCV 2.4.3，经过测试的版本包括 OpenCV 2.4.11 和 OpenCV 3.2。安装指南可以在 [OpenCV 官网](http://opencv.org) 上找到。

###  Eigen3

- **用途**：Eigen3 是一个线性代数库，g2o（图优化库）需要它。
- **版本要求**：至少需要 Eigen3 版本 3.1.0，下载和安装的说明可以在 [Eigen 官网](http://eigen.tuxfamily.org) 上找到。

### DBoW2 和 g2o（包含在 Thirdparty 文件夹中）

- **用途**：使用修改过的 DBoW2 库进行地点识别，使用 g2o 库进行非线性优化。
- **版权信息**：这两个修改过的库都是 BSD 许可证下的，可以在项目的 Thirdparty 文件夹中找到。

## 项目构建

这段内容提供了有关如何克隆和构建 ORB-SLAM2 项目的步骤。下面是对每个步骤的详细解释：

### 1. 克隆代码库
```bash
git clone https://github.com/raulmur/ORB_SLAM2.git ORB_SLAM2
```
### 2. 构建项目

- **构建脚本**：项目提供了一个名为 `build.sh` 的脚本，用于自动构建第三方库和 ORB-SLAM2 项目本身。
- **确保依赖已安装**：在执行构建脚本之前，请确认已安装所有必要的依赖项（如前面提到的库和工具）。

### 3. 进入项目目录

```bash
cd ORB_SLAM2
```
### 4. 赋予脚本执行权限

```bash
chmod +x build.sh
```
### 5. 执行构建脚本

```bash
./build.sh
```
### 6. 输出结果

- **生成的文件**：
  - **库文件**：构建完成后，会在 `lib` 文件夹中生成一个名为 `libORB_SLAM2.so` 的共享库文件。
  - **可执行文件**：在 `Examples` 文件夹中会生成多个可执行文件，包括：
    - `mono_tum`: 单目相机用于 TUM 数据集的示例。
    - `mono_kitti`: 单目相机用于 KITTI 数据集的示例。
    - `rgbd_tum`: RGB-D 相机用于 TUM 数据集的示例。
    - `stereo_kitti`: 立体相机用于 KITTI 数据集的示例。
    - `mono_euroc`: 单目相机用于 Euroc 数据集的示例。
    - `stereo_euroc`: 立体相机用于 Euroc 数据集的示例。

### 编译错误解决

#### 提示 `usleep` 函数在当前作用域中没有被声明

![image-20241023142213247](/home/lyb/.config/Typora/typora-user-images/image-20241023142213247.png)

##### 报错原因

- 报错信息提示 `usleep` 函数在当前作用域中没有被声明。`usleep` 是一个用于使程序暂停执行指定微秒数的 UNIX 系统调用。在 Linux 系统中，`usleep` 函数包含在 `<unistd.h>` 头文件中。如果在使用 `usleep` 函数的源文件中没有包含这个头文件，编译器就会报错说 `usleep` 没有在这个作用域中声明。

##### 解决方法

- 找到System.h文件（~/ORB_SLAM2/include）
- 在头文件的定义中加入

```C++
#include <unistd.h>
```

#### std::map 的使用与其分配器不兼容

![image-20241023141453066](/home/lyb/github/Typora_notes/image-20241023141453066.png)

##### 报错原因

- 这个错误是由于在 C++ STL（Standard Template Library）中，std::map 的使用与其分配器不兼容所引起的。在 C++ 中，容器（如 std::map）和分配器之间需要有一致的类型匹配。具体来说，std::map 定义中使用的分配器的 value_type 必须与 std::map 存储的元素类型相匹配。

##### 解决方法

- 找到LoopClosing.h文件（~/ORB_SLAM2/include）
- 修改以下代码：

```C++
typedef map<KeyFrame*,g2o::Sim3,std::less<KeyFrame*>,
        Eigen::aligned_allocator<std::pair<const KeyFrame*, g2o::Sim3> > > KeyFrameAndPose;
```

- 修改为：

```C++
typedef map<KeyFrame*,g2o::Sim3,std::less<KeyFrame*>,
        Eigen::aligned_allocator<std::pair<KeyFrame *const, g2o::Sim3> > > KeyFrameAndPose;
```



## RGBD示例运行

### **下载数据集**：

- 访问 [TUM数据集网站](http://vision.in.tum.de/data/datasets/rgbd-dataset/download)，下载所需的序列，并进行解压缩。

![image-20241022164202045](/home/lyb/github/Typora_notes/image-20241022164202045.png)

- 解压后得到如下内容

![image-20241022165002077](/home/lyb/github/Typora_notes/image-20241022165002077.png)

### **关联RGB图像和深度图像**：

- 使用提供的 `associate.py` 脚本来关联RGB图像和深度图像。你可以使用以下命令生成自己的关联文件：
  ```bash
  python associate.py PATH_TO_SEQUENCE/rgb.txt PATH_TO_SEQUENCE/depth.txt > associations.txt
  ```
- 这里需要替换 `PATH_TO_SEQUENCE` 为你下载序列的路径。

### **执行命令**：

![image-20241022165110943](/home/lyb/github/Typora_notes/image-20241022165110943.png)

- 运行以下命令来执行RGB-D数据处理：
  ```bash
  ./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUMX.yaml PATH_TO_SEQUENCE_FOLDER ASSOCIATIONS_FILE
  ```
  
- 在这个命令中，需要将 `TUMX.yaml` 替换为相应的配置文件名：
  - `TUM1.yaml` 对应 `freiburg1` 序列
  - `TUM2.yaml` 对应 `freiburg2` 序列
  - `TUM3.yaml` 对应 `freiburg3` 序列
  
- `PATH_TO_SEQUENCE_FOLDER` 需要替换为解压后的序列文件夹路径。

- `ASSOCIATIONS_FILE` 需要替换为生成的关联文件的路径。

- 例如：

```bash
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUM1.yaml ~/rgbd_dataset_freiburg1_xyz ~/ORB_SLAM2/Examples/RGB-D/associations/fr1_xyz.txt

```

- 运行成功后 如图：

![image-20241022165255191](/home/lyb/github/Typora_notes/image-20241022165255191.png)

## ROS RGBD 示例运行（Orbbec Astra+）

### 设置ROS_PACKAGE_PATH

首先，需要将包含 `Examples/ROS/ORB_SLAM2` 的路径添加到 `ROS_PACKAGE_PATH` 环境变量中。这可以通过编辑 `.bashrc` 文件来实现：
- 打开终端，输入以下命令以编辑 `.bashrc` 文件：
  ```bash
  nano ~/.bashrc
  ```
  
- 在文件末尾添加以下行（将`PATH`替换为你克隆的ORB_SLAM2的文件夹路径）：
  ```bash
  export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/ORB_SLAM2/Examples/ROS
  ```

- 例如：

  ```bash
  export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/ORB_SLAM2/Examples/ROS
  ```

- 保存并关闭文件，然后执行以下命令使更改生效：

  ```bash
  source ~/.bashrc
  ```

### 执行构建脚本

- 运行以下命令以构建ROS节点：

```bash
chmod +x build_ros.sh
./build_ros.sh
```

### 修改RGB-D节点订阅话题名称

- Astra+节点所发布的可用的话题（topics）

  - `/camera/color/camera_info` : 彩色相机信息（CameraInfo）话题。

  - `/camera/color/image_raw`: 彩色数据流图像话题。
  - `/camera/depth/camera_info`: 深度相机信息（CameraInfo）话题。

  - `/camera/depth/image_raw`: 深度数据流图像话题。

  - `/camera/depth/points` : 点云话题，仅当 `enable_point_cloud` 为 `true` 时才可用`.

  - `/camera/depth_registered/points`: 彩色点云话题，仅当 `enable_colored_point_cloud` 为 `true` 时才可用。

  - `/camera/ir/camera_info`:  红外相机信息（CameraInfo）话题。

  - `/camera/ir/image_raw`: 红外数据流图像话题。

- 由于RGB-D节点默认订阅话题 `/camera/rgb/image_raw` 和 `/camera/depth_registered/image_raw` ，所以需要修改为自己的相机所发布的话题，如下所示：

```C++
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/color/image_raw", 1);//彩色数据流图像话题
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth/image_raw", 1);//深度数据流图像话题
```

### 运行Astra+节点

```
roslaunch orbbec_camera astra_adv.launch
```

### 运行RGB-D节点

```bash
rosrun ORB_SLAM2 RGBD PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
```

- 例如：

```bash
rosrun ORB_SLAM2 RGBD ~/ORB_SLAM2/Vocabulary/ORBvoc.txt ~/ORB_SLAM2/Examples/ROS/ORB_SLAM2/Asus.yaml
```

- 可以在Asus.yaml中将原本参数更改为标定好的相机参数值
- 运行成功，如下：

![image-20241023120123394](/home/lyb/github/Typora_notes/image-20241023120123394.png)

### 编译错误解决

#### cv_bridge所链接的opencv版本与当前opencv版本冲突

![image-20241023144318785](/home/lyb/github/Typora_notes/image-20241023144318785.png)

##### 报错原因

- **冲突的库版本**: 错误中提到 `libopencv_imgproc.so.4.2.0` 需要但是找不到某些符号，同时还显示 `libopencv_core.so.4.2.0` 和 `libcv_bridge.so` 可能存在冲突。这通常表示你的系统中安装了多个版本的 OpenCV，或者 ROS 的 `cv_bridge` 模块链接到了一个与其他部分 ORB_SLAM2 使用的不同版本的 OpenCV。

- **缺失的符号**: 错误中提到 "undefined reference to symbol"，这通常意味着链接器（ld）无法在给定的库中找到必需的符号定义。这可能是因为编译环境配置错误，比如链接到了错误版本的库，或者某些必要的库没有被正确地包含在链接指令中。

- **DSO missing from command line**: 这个错误提示某个动态共享对象（DSO）在链接命令中缺失。这是链接器错误，表明在生成执行文件或库时，命令行缺少了必要的库文件。

##### 解决办法

- 卸载当前系统中安装的cv_bridge，原本默认链接4.2.0由于是编译好的，即使修改编译后的文件也无济于事。

```bash
sudo apt-get remove ros-noetic-cv-bridge
```

- 验证是否卸载成功

```bash
dpkg -l | grep cv-bridge
```

- 获取对应版本的cv_bridge源码

```
git clone -b noetic https://github.com/ros-perception/vision_opencv.git
```

- 配置系统环境变量

```bash
sudo vim .bashrc
# 加入
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/vision_opencv/cv_bridge
# 刷新系统环境变量
source .bashrc
```

- 编辑cv_bridge下的CMakeLists.txt，修改为自己的OpenCV版本即可

```CMake
find_package(OpenCV 3.2.0 REQUIRED)
```

- 编译和安装

```bash
cd ~/vision_opencv/cv_bridge
mkdir build

cmake ..
make
sudo make install
```

