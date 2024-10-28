# zUbuntu20.04 编译运行 ORBSLAM2_with_pointcloud_map(以RGBD Orbbec Astra+为例)

https://blog.csdn.net/u013454780/article/details/132965625

## 获取源码

```bash
git clone https://github.com/gaoxiang12/ORBSLAM2_with_pointcloud_map.git
```

## 解压文件

![image-20241027134202109](/home/lyb/github/Typora_notes/image-20241027134202109.png)

- 得到如下内容

![image-20241027134232227](/home/lyb/github/Typora_notes/image-20241027134232227.png)

## 编译安装修改后的g2o

```bash
cd g2o_with_orbslam2
mkdir build
cd build
cmake ..
make
sudo make install //别忘了这一步 不然之后会有文件找不到
```

### 编译错误解决

#### 尝试将 `double` 类型的角度直接传给需要 `Eigen::Rotation2D<double>` 对象的函数，导致类型不匹配

![image-20241027140932443](/home/lyb/github/Typora_notes/image-20241027140932443.png)

```bash
/home/lyb/ORBSLAM2_with_pointcloud_map/orbslam2_modified/g2o_with_orbslam2/g2o/types/slam2d/edge_se2_pointxy_bearing.cpp:51:39: error: cannot convert ‘Eigen::Rotation2D<double>::Scalar’ {aka ‘double’} to ‘const Rotation2Dd&’ {aka ‘const Eigen::Rotation2D<double>&’}
   51 |     t.setRotation(t.rotation().angle()+_measurement);
      |                   ~~~~~~~~~~~~~~~~~~~~^~~~~~~~~~~~~
      |                                       |
      |                                       Eigen::Rotation2D<double>::Scalar {aka double}
```

##### 报错原因

- 这个编译错误是因为在 g2o 库的代码中尝试将 `double` 类型的值（在这里是通过调用 `Eigen::Rotation2D<double>::angle()` 得到的角度加上 `_measurement`）赋值给需要 `const Rotation2Dd&` （即 `const Eigen::Rotation2D<double>&`）类型的函数 `setRotation`。`Eigen::Rotation2D<double>::angle()` 返回一个 `double` 类型的值，表示旋转的角度，而 `setRotation` 函数期望的是一个 `Rotation2Dd` 类型的对象。这种类型不匹配导致了编译错误，因为不能直接将 `double` 类型转换为 `Rotation2Dd` 对象。正确的做法应该是创建一个新的 `Rotation2Dd` 对象，将角度传递给它，然后再调用 `setRotation`。

##### 解决办法

- 修改edge2_se2_pointxy_bearing.cpp(ORBSLAM2_with_pointcloud_map/orbslam2_modified/g2o_with_orbslam2/g2o/types/slam2d)中的第51行
- 从

```C++
 t.setRotation(t.rotation().angle()+_measurement);
```

- 改为

- ```C++
  t.setRotation((Eigen::Rotation2Dd)(t.rotation().angle()+_measurement));
  ```

#### 由于在 Eigen 操作中未显式转换不同的数值类型，违反了 Eigen 对于类型一致性的严格要求

![image-20241027141941369](/home/lyb/github/Typora_notes/image-20241027141941369.png)

```bash
/home/lyb/ORBSLAM2_with_pointcloud_map/orbslam2_modified/g2o_with_orbslam2/g2o/solvers/eigen/linear_solver_eigen.h:92:10:   required from here
/usr/include/eigen3/Eigen/src/Core/util/XprHelper.h:819:96: error: static assertion failed: YOU_MIXED_DIFFERENT_NUMERIC_TYPES__YOU_NEED_TO_USE_THE_CAST_METHOD_OF_MATRIXBASE_TO_CAST_NUMERIC_TYPES_EXPLICITLY
  819 | ASSERT((Eigen::internal::has_ReturnType<ScalarBinaryOpTraits<LHS, RHS,BINOP> >::value), \
      |        ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~^~~~~~

```

##### 报错原因

- 这个错误是因为在 Eigen 库的操作中混合使用了不同的数值类型，而没有显式地进行类型转换。Eigen 要求在操作涉及不同类型的数值时，必须使用其 `.cast<>()` 方法来显式转换类型，以确保操作的数学和类型正确性。

##### 解决办法

- 修改linear_solver_eigen.h(ORBSLAM2_with_pointcloud_map/orbslam2_modified/g2o_with_orbslam2/g2o/solvers/eigen)的第54行

- 从

- ```C++
  typedef Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic, SparseMatrix::Index> PermutationMatrix;
  ```

- 改为

- ```C++
  typedef Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic, int> PermutationMatrix;
  ```



#### 在使用 Eigen 库时，向期望整数的参数传递了浮点数，导致类型不匹配

![image-20241027142613052](/home/lyb/github/Typora_notes/image-20241027142613052.png)

```bash
/home/lyb/ORBSLAM2_with_pointcloud_map/orbslam2_modified/g2o_with_orbslam2/g2o/examples/tutorial_slam2d/simulator.cpp:80:39:   required from here
/usr/include/eigen3/Eigen/src/Core/PlainObjectBase.h:778:27: error: static assertion failed: FLOATING_POINT_ARGUMENT_PASSED__INTEGER_WAS_EXPECTED
  778 |       EIGEN_STATIC_ASSERT(is_integer,
      |                           ^~~~~~~~~~
```



##### 报错原因

- 这个编译错误发生是因为在使用 Eigen 库时，期望一个整数类型的参数却错误地传入了一个浮点数，违反了 Eigen 对于参数类型的要求，Eigen 静态断言失败是为了确保类型安全，防止错误的数据类型使用。

##### 解决办法

- 修改simulator.cpp(ORBSLAM2_with_pointcloud_map/orbslam2_modified/g2o_with_orbslam2/g2o/examples/tutorial_slam2d)的第80行
- 从

- ```c++
  VectorXd probLimits(MO_NUM_ELEMS);
  for (int i = 0; i < probLimits.size(); ++i)
      probLimits[i] = (i + 1) / (double) MO_NUM_ELEMS;
  ```

- 改为

- ```C++
  VectorXd probLimits;
  probLimits.resize(MO_NUM_ELEMS);
  for (int i = 0; i < probLimits.size(); ++i)
      probLimits[i] = (i + 1) / (double) MO_NUM_ELEMS;
  ```

## 编译DBoW2

![image-20241027143534483](/home/lyb/github/Typora_notes/image-20241027143534483.png)

```bash
cd orbslam2_modified/ORB_SLAM2_modified/Thirdparty/DBoW2
mkdir build
cd build
cmake ..
make
```

## 编译Pangolin模块

- 略  
- https://github.com/stevenlovegrove/Pangolin

## 获取Vocabulary

![image-20241027144034033](/home/lyb/github/Typora_notes/image-20241027144034033.png)

![image-20241027143948974](/home/lyb/github/Typora_notes/image-20241027143948974.png)

- 将ORB_SLAM2原仓库的Vocabulary文件夹复制过来，替代这里的空的Vocabulary文件夹

## 编译修改后的ORB_SLAM2

```bash
cd ORB_SLAM2_modified
mkdir build
cd build
cmake ..
make
```

### 编译报错解决

#### 未能找到匹配请求的版本 "2.4.3" 的 OpenCV 配置文件，尽管检查了多个更高版本的安装

![image-20241027144211416](/home/lyb/github/Typora_notes/image-20241027144211416.png)

```bash
CMake Error at CMakeLists.txt:33 (find_package):
  Could not find a configuration file for package "OpenCV" that is compatible
  with requested version "2.4.3".

  The following configuration files were considered but not accepted:

    /usr/local/opencv3.2.0/share/OpenCV/OpenCVConfig.cmake, version: 3.2.0
    /usr/local/opencv-4.4.0/lib/cmake/opencv4/OpenCVConfig.cmake, version: 4.4.0
    /usr/local/opencv-3.2.0/share/OpenCV/OpenCVConfig.cmake, version: 3.2.0
    /usr/lib/x86_64-linux-gnu/cmake/opencv4/OpenCVConfig.cmake, version: 4.2.0
    /lib/x86_64-linux-gnu/cmake/opencv4/OpenCVConfig.cmake, version: 4.2.0



-- Configuring incomplete, errors occurred!

```

##### 报错原因

- 这个 CMake 错误发生是因为 CMake 在配置时未能找到与指定版本（2.4.3）兼容的 OpenCV 包配置文件，尽管考虑了多个已安装的更高版本（3.2.0, 4.4.0, 和 4.2.0），但这些都不满足指定的版本需求。

##### 解决方法

- 修改ORB_SLAM2_modified下的CMakeLists.txt

- 将

- ```cmake
  find_package(OpenCV 2.4.3 REQUIRED)
  ```

- 改为自己系统中安装的OpenCV版本即可

- 例如 我这里装的是OpenCV 3.2.0

- ```cmake
  find_package(OpenCV 3.2.0 REQUIRED)
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

- 修改LoopClosing.h(ORBSLAM2_with_pointcloud_map/orbslam2_modified/ORB_SLAM2_modified/include)中的第49行

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
- 主要存在于以下文件中(ORBSLAM2_with_pointcloud_map/orbslam2_modified/ORB_SLAM2_modified/Examples)
  - Monocular
    - mono_kitti.cc
    - mono_tum.cc
  - RGB-D
    - rgbd_tum.cc
  - Stereo
    - stereo_kitti.cc

## 运行ORB_SLAM2 RGBD

```bash
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUM1.yaml ~/rgbd_dataset_freiburg1_xyz Examples/RGB-D/associations/fr1_xyz.txt
```

- 运行成功，效果如下

![image-20241027152053520](/home/lyb/github/Typora_notes/image-20241027152053520.png)

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

  - ORBSLAM2_with_pointcloud_map/orbslam2_modified/ORB_SLAM2_modified/CMakeLists.txt
  - ORBSLAM2_with_pointcloud_map/orbslam2_modified/ORB_SLAM2_modified/Examples/ROS/ORB_SLAM2/CMakeLists.txt
  - ORBSLAM2_with_pointcloud_map/orbslam2_modified/ORB_SLAM2_modified/Thirdparty/DBoW2/CMakeLists.txt

- 之后全部重新用CMake构建，Make编译一遍即可，再次运行即可运行成功。





![image-20241028125738390](/home/lyb/github/Typora_notes/image-20241028125738390.png)
