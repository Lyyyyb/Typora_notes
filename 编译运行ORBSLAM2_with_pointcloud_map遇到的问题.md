# 编译运行ORBSLAM2_with_pointcloud_map遇到的问题

![image-20241027114320911](/home/lyb/github/Typora_notes/image-20241027114320911.png)

```bash
/home/lyb/ORBSLAM2_with_pointcloud_map/ORBSLAM2_with_pointcloud_map/orbslam2_modified/g2o_with_orbslam2/g2o/types/slam2d/edge_se2_pointxy_bearing.cpp:51:39: error: cannot convert ‘Eigen::Rotation2D<double>::Scalar’ {aka ‘double’} to ‘const Rotation2Dd&’ {aka ‘const Eigen::Rotation2D<double>&’}
   51 |     t.setRotation(t.rotation().angle()+_measurement);
      |                   ~~~~~~~~~~~~~~~~~~~~^~~~~~~~~~~~~
      |                                       |
      |                                       Eigen::Rotation2D<double>::Scalar {aka double}

```

https://blog.csdn.net/qq_29710939/article/details/119961490



![image-20241027115014304](/home/lyb/github/Typora_notes/image-20241027115014304.png)

```
/home/lyb/ORBSLAM2_with_pointcloud_map/ORBSLAM2_with_pointcloud_map/orbslam2_modified/g2o_with_orbslam2/g2o/solvers/eigen/linear_solver_eigen.h:92:10:   required from here
/usr/include/eigen3/Eigen/src/Core/util/XprHelper.h:819:96: error: static assertion failed: YOU_MIXED_DIFFERENT_NUMERIC_TYPES__YOU_NEED_TO_USE_THE_CAST_METHOD_OF_MATRIXBASE_TO_CAST_NUMERIC_TYPES_EXPLICITLY
  819 | ASSERT((Eigen::internal::has_ReturnType<ScalarBinaryOpTraits<LHS, RHS,BINOP> >::value), \
      |        ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~^~~~~~

```

https://blog.csdn.net/qq_29710939/article/details/119961490

![image-20241027115509284](/home/lyb/github/Typora_notes/image-20241027115509284.png)

```
/home/lyb/ORBSLAM2_with_pointcloud_map/ORBSLAM2_with_pointcloud_map/orbslam2_modified/g2o_with_orbslam2/g2o/examples/tutorial_slam2d/simulator.cpp:80:39:   required from here
/usr/include/eigen3/Eigen/src/Core/PlainObjectBase.h:778:27: error: static assertion failed: FLOATING_POINT_ARGUMENT_PASSED__INTEGER_WAS_EXPECTED
  778 |       EIGEN_STATIC_ASSERT(is_integer,
      |                           ^~~~~~~~~~

```

https://blog.csdn.net/qq_40213457/article/details/87939629



![image-20241027120411603](/home/lyb/github/Typora_notes/image-20241027120411603.png)

```
orbslam2_modified/ORB_SLAM2_modified/src/System.cc:23:
/usr/include/pcl-1.10/pcl/pcl_config.h:7:4: error: #error PCL requires C++14 or above
    7 |   #error PCL requires C++14 or above
      |    ^~~~~

```

https://blog.csdn.net/mantou_riji/article/details/123331548



![image-20241027121327003](/home/lyb/github/Typora_notes/image-20241027121327003.png)

```
/home/lyb/ORBSLAM2_with_pointcloud_map/ORBSLAM2_with_pointcloud_map/orbslam2_modified/ORB_SLAM2_modified/src/Optimizer.cc:818:37:   required from here
/usr/include/c++/9/bits/stl_map.h:122:71: error: static assertion failed: std::map must have the same value_type as its allocator
  122 |       static_assert(is_same<typename _Alloc::value_type, value_type>::value,
      |   
```

https://blog.csdn.net/weixin_69479603/article/details/131882929



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





double free or corruption (out)
已放弃 (核心已转储)

```
lyb@lyb:~/ORBSLAM2_with_pointcloud_map/ORBSLAM2_with_pointcloud_map/orbslam2_modified/ORB_SLAM2_modified$ ./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUM1.yaml ~/rgbd_dataset_freiburg1_xyz Examples/RGB-D/associations/fr1_xyz.txt

ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza.
This program comes with ABSOLUTELY NO WARRANTY;
This is free software, and you are welcome to redistribute it
under certain conditions. See LICENSE.txt.

Input sensor was set to: RGB-D

Loading ORB Vocabulary. This could take a while...
Vocabulary loaded!


Camera Parameters: 
- fx: 517.306
- fy: 516.469
- cx: 318.643
- cy: 255.314
- k1: 0.262383
- k2: -0.953104
- k3: 1.16331
- p1: -0.005358
- p2: 0.002628
- fps: 30
- color order: RGB (ignored if grayscale)

ORB Extractor Parameters: 
- Number of Features: 1000
- Scale Levels: 8
- Scale Factor: 1.2
- Initial Fast Threshold: 20
- Minimum Fast Threshold: 7

Depth Threshold (Close/Far Points): 3.86618

-------
Start processing sequence ...
Images in the sequence: 792

New map created with 832 points
double free or corruption (out)
已放弃 (核心已转储)
```

https://www.cnblogs.com/kidtic/p/14223679.html

Depth Threshold (Close/Far Points): 3.86618
段错误 (核心已转储)

```
lyb@lyb:~/ORBSLAM2_with_pointcloud_map/ORBSLAM2_with_pointcloud_map/orbslam2_modified/ORB_SLAM2_modified$ ./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUM1.yaml ~/rgbd_dataset_freiburg1_xyz Examples/RGB-D/associations/fr1_xyz.txt

ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza.
This program comes with ABSOLUTELY NO WARRANTY;
This is free software, and you are welcome to redistribute it
under certain conditions. See LICENSE.txt.

Input sensor was set to: RGB-D

Loading ORB Vocabulary. This could take a while...
Vocabulary loaded!


Camera Parameters: 
- fx: 517.306
- fy: 516.469
- cx: 318.643
- cy: 255.314
- k1: 0.262383
- k2: -0.953104
- k3: 1.16331
- p1: -0.005358
- p2: 0.002628
- fps: 30
- color order: RGB (ignored if grayscale)

ORB Extractor Parameters: 
- Number of Features: 1000
- Scale Levels: 8
- Scale Factor: 1.2
- Initial Fast Threshold: 20
- Minimum Fast Threshold: 7

Depth Threshold (Close/Far Points): 3.86618
段错误 (核心已转储)

```

https://blog.csdn.net/u013454780/article/details/132965625





![image-20241027213128994](/home/lyb/github/Typora_notes/image-20241027213128994.png)



```bash
/home/lyb/pointcloudmap_ws/src/ORB_SLAM2_modified/Examples/ROS/ORB_SLAM2/../../../include/pointcloudmapping.h:25:10: fatal error: pcl/common/transforms.h: 没有那个文件或目录
   25 | #include <pcl/common/transforms.h>
      |          ^~~~~~~~~~~~~~~~~~~~~~~~~
compilation terminated.

```

