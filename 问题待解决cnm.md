octomap获取八叉树地图

点云 /pointcloud_mapping/Global/PointCloudOutput 有了

- 还缺一个tf转换  camera_color_frame->map 位姿
  - 用第一帧图像的摄像机位置来作为世界坐标系

- orb_slam2 中的视觉里程计？？
  - Tracking 获得相机位姿

- move_base octomap 三维栅格地图

要使得它能运行起来，我们就得构建好这些输入和输出。

必要的输入：

goal ： 期望机器人在地图中的目标位置。

tf ： 各个坐标系之间的转换关系。（具体/map frame --> /odom frame ，/odom frame --> /base_link frame）      

odom：根据机器人左右轮速度推算出的航向信息（即/odom 坐标系中机器人x,y坐标以及航向角yaw，下面会具体介绍）



？？

- map->odom->base_link->camera_link



- map->camera_link

- base_link->camera_link

- map->base_link

- map->odom

- odom->base_link

- 3. **计算`odom`到`base_link`的转换**

  假设你已经从视觉里程计获取了`odom`（或`map`）到`camera_link`的转换，并且知道`camera_link`到`base_link`的固定转换，你可以使用以下方法计算`odom`到`base_link`的转换：

  - **使用TF变换链**：在ROS中，你可以通过组合`odom`到`camera_link`和`camera_link`到`base_link`的变换来计算`odom`到`base_link`的变换。这通常通过监听TF树中的相应变换来完成。

  
