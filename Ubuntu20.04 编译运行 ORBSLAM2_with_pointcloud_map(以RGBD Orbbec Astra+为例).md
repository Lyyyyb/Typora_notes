# Ubuntu20.04 编译运行 ORBSLAM2_with_pointcloud_map(以RGBD Orbbec Astra+为例)

## 与ORBSLAM2原仓库相比所作出的修改的地方

- 将词汇表加载方式改为二进制模式

- 添加了点云查看器（通过增加一个查看器线程实现）

- 修改了 CMakeLists.txt，所有可执行文件都放置在 `./bin` 目录下。

## 获取源码

```bash
git clone https://github.com/gaoxiang12/ORBSLAM2_with_pointcloud_map.git
```

## 获取词汇表

- 从ORBSLAM2原始仓库的Vocabulary文件夹复制过来，放置在ORB_SLAM2_modified下

![image-20241028133657562](/home/lyb/github/Typora_notes/image-20241028133657562.png)

![image-20241028133730808](/home/lyb/github/Typora_notes/image-20241028133730808.png)

## 编译ORB_SLAM2_modified

```bash
cd ORB_SLAM2_modified
chmod +x build.sh
./build.sh
```



### 编译报错解决

#### 当前编译环境未设置为使用 PCL 所需的 C++14 或更高版本的编译标准

![image-20241027144713738](/home/lyb/github/Typora_notes/image-20241027144713738.png)

```bash
/usr/include/pcl-1.10/pcl/pcl_config.h:7:4: error: #error PCL requires C++14 or above
    7 |   #error PCL requires C++14 or above
      |    ^~~~~

```

##### 报错原因

- 这个编译错误发生是因为 Point Cloud Library (PCL) 需要 C++14 或更高版本的编译标准，而当前的编译环境可能未设置为使用 C++14 或更高。

##### 解决方法

- 修改ORB_SLAM2_modified下的CMakeLists.txt

- 将

- ```Cmake
  CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
  CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
  if(COMPILER_SUPPORTS_CXX11)
     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
     add_definitions(-DCOMPILEDWITHC11)
     message(STATUS "Using flag -std=c++11.")
  elseif(COMPILER_SUPPORTS_CXX0X)
     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
     add_definitions(-DCOMPILEDWITHC0X)
     message(STATUS "Using flag -std=c++0x.")
  else()
     message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
  endif()
  
  ```

- 其中的11全部改为14即可

- ```cmake
  CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
  CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
  if(COMPILER_SUPPORTS_CXX14)
     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
     add_definitions(-DCOMPILEDWITHC14)
     message(STATUS "Using flag -std=c++14.")
  elseif(COMPILER_SUPPORTS_CXX0X)
     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
     add_definitions(-DCOMPILEDWITHC0X)
     message(STATUS "Using flag -std=c++0x.")
  else()
     message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
  endif()
  ```

#### `std::map` 的分配器的值类型与 `map` 的值类型不一致，导致静态断言失败

![image-20241027144958295](/home/lyb/github/Typora_notes/image-20241027144958295.png)

```bash
/usr/include/c++/9/bits/stl_map.h:122:71: error: static assertion failed: std::map must have the same value_type as its allocator
  122 |       static_assert(is_same<typename _Alloc::value_type, value_type>::value,
      |                                                                       ^~~~~
```

##### 报错原因

这个编译错误是因为在使用 `std::map` 时，分配器（allocator）的 `value_type` 和 `std::map` 的值类型不匹配，违反了 STL `map` 的类型一致性要求，导致静态断言失败。

##### 解决方法

- 修改LoopClosing.h(ORBSLAM2_with_pointcloud_map/ORB_SLAM2_modified/include)中的第49行

- 从

- ```C++
  typedef map<KeyFrame*,g2o::Sim3,std::less<KeyFrame*>,
  	Eigen::aligned_allocator<std::pair<const KeyFrame*, g2o::Sim3> > > KeyFrameAndPose;
  ```

- 改为

- ```C++
  typedef map<KeyFrame*,g2o::Sim3,std::less<KeyFrame*>,
          Eigen::aligned_allocator<std::pair<KeyFrame* const, g2o::Sim3> > > KeyFrameAndPose;
  ```

####  代码错误地引用了不存在的 `std::chrono::monotonic_clock` 类，导致相关时间点变量 `t1` 和 `t2` 未被正确声明

![image-20241027145611897](/home/lyb/github/Typora_notes/image-20241027145611897.png)

```bash
/home/lyb/ORBSLAM2_with_pointcloud_map/orbslam2_modified/ORB_SLAM2_modified/Examples/Monocular/mono_tum.cc: In function ‘int main(int, char**)’:
/home/lyb/ORBSLAM2_with_pointcloud_map/orbslam2_modified/ORB_SLAM2_modified/Examples/Monocular/mono_tum.cc:81:22: error: ‘std::chrono::monotonic_clock’ has not been declared
   81 |         std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
      |                      ^~~~~~~~~~~~~~~
/home/lyb/ORBSLAM2_with_pointcloud_map/orbslam2_modified/ORB_SLAM2_modified/Examples/Monocular/mono_tum.cc:90:22: error: ‘std::chrono::monotonic_clock’ has not been declared
   90 |         std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
      |                      ^~~~~~~~~~~~~~~
/home/lyb/ORBSLAM2_with_pointcloud_map/orbslam2_modified/ORB_SLAM2_modified/Examples/Monocular/mono_tum.cc:93:83: error: ‘t2’ was not declared in this scope; did you mean ‘tm’?
   93 | uble ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
      |                                                                         ^~
      |                                                                         tm
/home/lyb/ORBSLAM2_with_pointcloud_map/orbslam2_modified/ORB_SLAM2_modified/Examples/Monocular/mono_tum.cc:93:88: error: ‘t1’ was not declared in this scope; did you mean ‘y1’?
   93 | uble ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
      |                                                                              ^~
      |                                                                              y1

```

##### 报错原因

- 这些编译错误发生因为代码中尝试使用了不存在的 `std::chrono::monotonic_clock` 类，该类在 C++ 标准库中并未定义。此外，由于 `t1` 和 `t2` 变量的声明失败（因为它们依赖于不存在的 `monotonic_clock` 类），后续代码中尝试使用这些变量时也出现了错误，提示未声明的变量。正确的类应该是 `std::chrono::steady_clock` 或 `std::chrono::system_clock`。

##### 解决方法

- 将代码中所有使用 `std::chrono::monotonic_clock` 的地方替换为 `std::chrono::steady_clock`。
- 主要存在于以下文件中(ORBSLAM2_with_pointcloud_map/ORB_SLAM2_modified/Examples)
  - Monocular
    - mono_euroc.cc
    - mono_kitti.cc
    - mono_tum.cc
  - RGB-D
    - rgbd_tum.cc
  - Stereo
    - stereo_euroc.cc
    - stereo_kitti.cc

#### 系统中没有安装或找不到名为 `libopencv_core3.so.3.1` 的 OpenCV 3.1 库文件

![image-20241028134753664](/home/lyb/github/Typora_notes/image-20241028134753664.png)

```bash
./tools/bin_vocabulary: error while loading shared libraries: libopencv_core3.so.3.1: cannot open shared object file: No such file or directory
```

##### 报错原因

- 这个错误信息表明程序 `./tools/bin_vocabulary` 在尝试加载一个名为 `libopencv_core3.so.3.1` 的共享库文件时遇到问题，因为它无法在系统上找到这个文件。这通常是因为所需的 OpenCV 3.1 库没有被正确安装或者其安装路径没有被加入到系统的库搜索路径中。
- 这个问题发生是因为项目之前编译时使用的OpenCV版本已被更新，导致编译好的文件无法链接到原来版本的OpenCV库。即使多次运行`./build.sh`脚本，如果没有重新编译生成所有依赖文件，仍然会遇到链接错误。解决方法是确保所有文件与当前系统的库版本兼容，或者彻底重新编译整个项目以匹配新的库版本。

##### 解决方法

- 将原本的build文件夹删除，重新运行build.sh脚本进行构建编译即可。
- 主要是以下几个
  - ORBSLAM2_with_pointcloud_map/ORB_SLAM2_modified/build
  - ORBSLAM2_with_pointcloud_map/ORB_SLAM2_modified/Examples/ROS/ORB_SLAM2/build
  - ORBSLAM2_with_pointcloud_map/ORB_SLAM2_modified/Thirdparty/DBoW2/build
  - ORBSLAM2_with_pointcloud_map/ORB_SLAM2_modified/Thirdparty/g2o/build

- 编译通过，如下

![image-20241028135839793](/home/lyb/github/Typora_notes/image-20241028135839793.png)

## 运行ORB_SLAM2 RGBD

```bash
./bin/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUM1.yaml ~/rgbd_dataset_freiburg1_xyz Examples/RGB-D/associations/fr1_xyz.txt
```

- 运行成功，效果如下

![image-20241028140812128](/home/lyb/github/Typora_notes/image-20241028140812128.png)

### 运行报错解决

#### 段错误 (核心已转储)

![image-20241027151036736](/home/lyb/github/Typora_notes/image-20241027151036736.png)

```bash
Depth Threshold (Close/Far Points): 3.86618
段错误 (核心已转储)
```

##### 报错原因

- "段错误 (核心已转储)" 错误通常表明程序试图访问其内存空间中未授权或不存在的部分，这可能是由于无效的指针引用、数组越界、或其他内存管理错误导致的。

##### 解决方法

- 删除所有CMakeLists.txt文件里的`-march=native`

- 如下

- ```cmake
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")
  ```

- 修改为

- ```cmake
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3  ")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   ")
  ```

- 主要存在于以下文件中：

  - ORBSLAM2_with_pointcloud_map/ORB_SLAM2_modified/CMakeLists.txt
  - ORBSLAM2_with_pointcloud_map/ORB_SLAM2_modified/Examples/ROS/ORB_SLAM2/CMakeLists.txt
  - ORBSLAM2_with_pointcloud_map/ORB_SLAM2_modified/Thirdparty/DBoW2/CMakeLists.txt
  - ORBSLAM2_with_pointcloud_map/ORB_SLAM2_modified/Thirdparty/g2o/CMakeLists.txt

- 将原本的build文件夹删除，重新运行build.sh脚本进行构建编译即可，再次运行即可运行成功。

### 修改代码保存彩色点云地图

#### 步骤1: 在Tracking.h中添加成员变量

- 修改Tracking.h(ORBSLAM2_with_pointcloud_map/ORB_SLAM2_modified/include)中的第104行，添加新的成员变量，以保存当前帧的彩色图像。

- 从

- ```C++
  // Current Frame
  Frame mCurrentFrame;
  cv::Mat mImGray;
  cv::Mat mImDepth; 
  ```

- 改为

- ```C++
  // Current Frame
  Frame mCurrentFrame;
  cv::Mat mImRGB; // NEW
  cv::Mat mImGray;
  cv::Mat mImDepth;
  ```

#### 步骤2: 在Tracking.cc中修改代码

- 修改Tracking.cc(ORBSLAM2_with_pointcloud_map/ORB_SLAM2_modified/src/Tracking.cc)中的208行

- 从

- ```C++
  cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp)
  {
      mImGray = imRGB;
      mImDepth = imD;
  ```

- 改为

- ```C++
  cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB, const cv::Mat &imD, const double &timestamp)
  {
      mImRGB = imRGB; // NEW
      mImGray = imRGB;
      mImDepth = imD;
  ```

- 修改Tracking.cc(ORBSLAM2_with_pointcloud_map/ORB_SLAM2_modified/src/Tracking.cc)中的1142行

- 从

- ```C++
  mpPointCloudMapping->insertKeyFrame( pKF, this->mImGray, this->mImDepth );
  
  ```

- 改为

- ```C++
  mpPointCloudMapping->insertKeyFrame( pKF, this->mImRGB, this->mImDepth ); // 修改
  
  ```

#### 步骤3: 保存彩色点云地图

- 修改pointcloudmapping.cc(ORBSLAM2_with_pointcloud_map/ORB_SLAM2_modified/src/pointcloudmapping.cc)

- 添加头文件

- ```C++
  #include <pcl/io/pcd_io.h>
  ```

- 修改pointcloudmapping.cc中的void PointCloudMapping::viewer() 函数

- 从

```C++
void PointCloudMapping::viewer()
{
    pcl::visualization::CloudViewer viewer("viewer");
    while(1)
    {
        {
            unique_lock<mutex> lck_shutdown( shutDownMutex );
            if (shutDownFlag)
            {
                break;
            }
        }
        {
            unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex );
            keyFrameUpdated.wait( lck_keyframeUpdated );
        }

        // keyframe is updated
        size_t N=0;
        {
            unique_lock<mutex> lck( keyframeMutex );
            N = keyframes.size();
        }

        for ( size_t i=lastKeyframeSize; i<N ; i++ )
        {
            PointCloud::Ptr p = generatePointCloud( keyframes[i], colorImgs[i], depthImgs[i] );
            *globalMap += *p;
        }
        PointCloud::Ptr tmp(new PointCloud());
        voxel.setInputCloud( globalMap );
        voxel.filter( *tmp );
        globalMap->swap( *tmp );
        viewer.showCloud( globalMap );
        cout << "show global map, size=" << globalMap->points.size() << endl;
        lastKeyframeSize = N;
    }
}
```

- 改为

```C++
void PointCloudMapping::viewer()
{
    pcl::visualization::CloudViewer viewer("viewer");
    while(1)
    {
        {
            unique_lock<mutex> lck_shutdown( shutDownMutex );
            if (shutDownFlag)
            {
                break;
            }
        }
        {
            unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex );
            keyFrameUpdated.wait( lck_keyframeUpdated );
        }

        // keyframe is updated
        size_t N=0;
        {
            unique_lock<mutex> lck( keyframeMutex );
            N = keyframes.size();
        }

        for ( size_t i=lastKeyframeSize; i<N ; i++ )
        {
            PointCloud::Ptr p = generatePointCloud( keyframes[i], colorImgs[i], depthImgs[i] );
            *globalMap += *p;
        }
        PointCloud::Ptr tmp(new PointCloud());
        voxel.setInputCloud( globalMap );
        voxel.filter( *tmp );
        globalMap->swap( *tmp );
        viewer.showCloud( globalMap );
        cout << "show global map, size=" << globalMap->points.size() << endl;
        string save_path = "./resultPointCloudFile.pcd";//NEW
        pcl::io::savePCDFile(save_path, *globalMap);//NEW
        cout << "save pcd files to :  " << save_path << endl;//NEW

        lastKeyframeSize = N;
    }
}
```

- 修改完之后，再次编译运行，效果如下：

![image-20241028142746098](/home/lyb/github/Typora_notes/image-20241028142746098.png)

![image-20241028142817487](/home/lyb/github/Typora_notes/image-20241028142817487.png)

# 编译运行ORBSLAM2 ROS示例(Orbbec Astra+)

## 创建ros工作空间并初始化

```bash
mkdir -p ~/pointcloudmap_ws/src
cd ~/catkin_pointcloudmap_ws/src
catkin_init_workspace
cd ..
catkin_make
```

- 并将source ~/catkin_pointcloudmap_ws/devel/setup.bash写入到.bashrc，要注意写入的位置，否则可能会覆盖之前的环境变量。

- 将ORB_SLAM2_modified复制到src文件夹下

## 编译ORBSLAM2 ROS

- 确保先删除src/ORB_SLAM2_modified/Examples/ROS/ORB_SLAM2下的build目录

```
cd src/ORB_SLAM2_modified
chmod +x build_ros.sh
./build_ros.sh
```

### 编译报错解决

#### 系统中未正确安装或配置点云库（PCL），导致无法找到头文件 `pcl/common/transforms.h`

![image-20241028143701236](/home/lyb/github/Typora_notes/image-20241028143701236.png)

```
/home/lyb/pointcloudmap_ws/src/ORB_SLAM2_modified/Examples/ROS/ORB_SLAM2/../../../include/pointcloudmapping.h:25:10: fatal error: pcl/common/transforms.h: 没有那个文件或目录
   25 | #include <pcl/common/transforms.h>
      |          ^~~~~~~~~~~~~~~~~~~~~~~~~

```

##### 报错原因

- 这个错误提示表明编译过程中找不到 `pcl/common/transforms.h` 文件。这通常意味着 PCL（点云库）没有正确安装或者其安装路径没有被正确配置在项目的包含目录中。你需要确认 PCL 库已经正确安装在系统上，并且在项目的编译设置中包含了正确的头文件搜索路径。

##### 解决方法

- 修改ORB_SLAM2下的CMakeLists.txt（pointcloudmap_ws/src/ORB_SLAM2_modified/Examples/ROS/ORB_SLAM2），添加PCL库的信息
- 从

- ```cmake
  find_package(Eigen3 3.1.0 REQUIRED)
  find_package(Pangolin REQUIRED)
  
  include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/../../../
  ${PROJECT_SOURCE_DIR}/../../../include
  ${Pangolin_INCLUDE_DIRS}
  )
  ```

- 改为

- ```Cmake
  find_package(Eigen3 3.1.0 REQUIRED)
  find_package(Pangolin REQUIRED)
  find_package( PCL REQUIRED )# NEW
  
  include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/../../../
  ${PROJECT_SOURCE_DIR}/../../../include
  ${Pangolin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}# NEW
  )
  
  add_definitions( ${PCL_DEFINITIONS} )# NEW
  link_directories( ${PCL_LIBRARY_DIRS} )# NEW
  ```

#### 当前编译环境未设置为使用 PCL 所需的 C++14 或更高版本的编译标准

![image-20241027144713738](/home/lyb/github/Typora_notes/image-20241027144713738.png)

```bash
/usr/include/pcl-1.10/pcl/pcl_config.h:7:4: error: #error PCL requires C++14 or above
    7 |   #error PCL requires C++14 or above
      |    ^~~~~

```

##### 报错原因

- 这个编译错误发生是因为 Point Cloud Library (PCL) 需要 C++14 或更高版本的编译标准，而当前的编译环境可能未设置为使用 C++14 或更高。

##### 解决方法

- 修改ORB_SLAM2_modified下的CMakeLists.txt

- 将

- ```Cmake
  CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
  CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
  if(COMPILER_SUPPORTS_CXX11)
     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
     add_definitions(-DCOMPILEDWITHC11)
     message(STATUS "Using flag -std=c++11.")
  elseif(COMPILER_SUPPORTS_CXX0X)
     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
     add_definitions(-DCOMPILEDWITHC0X)
     message(STATUS "Using flag -std=c++0x.")
  else()
     message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
  endif()
  
  ```

- 其中的11全部改为14即可

- ```cmake
  CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
  CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
  if(COMPILER_SUPPORTS_CXX14)
     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
     add_definitions(-DCOMPILEDWITHC14)
     message(STATUS "Using flag -std=c++14.")
  elseif(COMPILER_SUPPORTS_CXX0X)
     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
     add_definitions(-DCOMPILEDWITHC0X)
     message(STATUS "Using flag -std=c++0x.")
  else()
     message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
  endif()
  ```

- 之后即可编译成功，如下

![image-20241028144410386](/home/lyb/github/Typora_notes/image-20241028144410386.png)

## 运行ORB_SLAM2 ROS(以Orbbec Astra+为例)

- 修改RGB-D节点订阅话题名称

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


- 运行Astra+节点

```
roslaunch orbbec_camera astra_adv.launch
```

- 运行RGB-D节点

```
rosrun ORB_SLAM2 RGBD Vocabulary/ORBvoc.txt Examples/ROS/ORB_SLAM2/Asus.yaml
```

- 运行成功，如下

![image-20241028195117579](/home/lyb/github/Typora_notes/image-20241028195117579.png)

### 运行报错解决

#### 程序尝试写入一个空的点云数据到 PCD 文件，导致抛出了 `pcl::IOException` 异常

- 深度相机深度值的单位 mm

https://blog.csdn.net/m0_62882335/article/details/140722866

https://github.com/gaoxiang12/ORBSLAM2_with_pointcloud_map/issues/40

https://blog.csdn.net/Xuesengxinyi/article/details/93469079

![image-20241028144625436](/home/lyb/github/Typora_notes/image-20241028144625436.png)

```bash
terminate called after throwing an instance of 'pcl::IOException'
  what():  : [pcl::PCDWriter::writeASCII] Input point cloud has no data!
已放弃 (核心已转储)
```

##### 报错原因

- 这个错误信息表明程序在尝试以 ASCII 格式写入一个 PCD（点云数据）文件时发生了异常，因为输入的点云数据为空。`pcl::IOException` 异常通常是因为尝试对一个没有任何点的点云进行操作，这在处理或保存点云数据时需要确保点云非空才能正常进行。

##### 解决方法

- 修改Asus.yaml(pointcloudmap_ws/src/ORB_SLAM2_modified/Examples/ROS/ORB_SLAM2)中的DepthMapFactor

- 从

- ```yaml
  DepthMapFactor: 1.0
  ```

- 改为

- ```yaml
  DepthMapFactor: 1000.0
  ```


#### 点云图没有实时绘制，viewer界面只显示一个坐标系

![image-20241028193234173](/home/lyb/github/Typora_notes/image-20241028193234173.png)

##### 报错原因

- 滤波模块有问题,直接把showCloud函数提前即可。

##### 解决方法

- 修改pointcloudmapping.cc中的void PointCloudMapping::viewer()

- 从

- ```C++
  PointCloud::Ptr tmp(new PointCloud());
  voxel.setInputCloud( globalMap );
  voxel.filter( *tmp );
  globalMap->swap( *tmp );
  viewer.showCloud( globalMap );//before
  ```

- 改为 把showCloud函数提前

- ```c++
  viewer.showCloud( globalMap );//after
  PointCloud::Ptr tmp(new PointCloud());
  voxel.setInputCloud( globalMap );
  voxel.filter( *tmp );
  globalMap->swap( *tmp );
  ```

  

