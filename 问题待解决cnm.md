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








- odom->base_link 视觉里程计 相机位姿 估算位置 实际位置
- map->odom
- map->odom->base_link
- odom->base_link 通过视觉里程计估算离目的地10m，但是通过定位算法得到实际离目的地只有9m，然后这个定位算法就会输出一段map->odom的变换来纠正这段偏差(类似平行四边形)，从而得到map->odom->base_link的完整tf树
- 为了获得map->odom->base_link->camera_link
- ORBSLAM2 已知
  - odom->camera_link（Tracking得到）
  - base_link->camera_link （直接测量得到）
- 要求
  - map->odom
    - 通过定位算法修正
  - odom->base_link
    - 可根据odom->camera_link 和 base_link->camera_link直接得到
- 例子：
  - odom->base_link 通过视觉里程计估算离目的地10m，但是通过定位算法得到实际离目的地只有9m，然后这个定位算法就会输出一段map->odom(实际先输出map->base_link 然后通过map->base_link和odom->base_link计算得到map->odom)的变换来纠正这段偏差(类似平行四边形)，从而得到map->odom->base_link->camera_link的完整tf树、
- orbslam2 中的map坐标系？？？？？

  - 第一帧图像的坐标系
  - 通常使用第一个关键帧（Keyframe）或第一对图像帧来设定地图的初始坐标系原点




![](/home/lyb/github/Typora_notes/image-20241113141027710.png)

![image-20241113141202502](/home/lyb/.config/Typora/typora-user-images/image-20241113141202502.png)

![image-20241113141519206](/home/lyb/github/Typora_notes/image-20241113141519206.png)

![image-20241113141708011](/home/lyb/github/Typora_notes/image-20241113141708011.png)

![image-20241113142151168](/home/lyb/github/Typora_notes/image-20241113142151168.png)

![image-20241113142333453](/home/lyb/github/Typora_notes/image-20241113142333453.png)

- 比如按照里程计的计算，机器人应该运动到图中的位置，但是实际上因为打滑，它真实的位置可能在其他地方，如图，中间的这点误差还会不断累积，如果放任它这么累积下去，迟早会偏差个十万八千里，怎么去修正它呢，这就要用到之前介绍的障碍物点云配准的定位算法，比如机器人的实际位置在这里，如果让激光雷达扫描障碍物，得到的雷达点云因该是这个样子，现在把这个点云，挪到里程计估算的位置上，点云和障碍物明显没有重合上，说明里程计产生了误差，如何弥补呢，这时候直接将激光雷达的点云贴合到障碍物上，同时机器人的位置也被拉了过来，只需要在里程计估算的位置上，加上这一小段位移，就能让激光雷达的扫描点云和障碍物的轮廓吻合上了，这就是障碍物点云配准算法的作用，用来修正里程计的误差。
- 这里有个问题里程计输出的是odom到base_link的tf，它已经和机器人的底盘投影中心连接上了，没法在base_link和机器人底盘之间在插入新的TF，所以一般的SLAM节点都会迁就里程计，SLAM节点最重要输出的是map到base_link的tf，既然tf末端的base_link已经被里程计占了，那么slam节点就把这段TF挪到跟端的odom之前，也就是这段map->odom的tf，跟里程计输出的tf(odom->base_link)一起连起来，就实现了map->base_link的tf。
- 像这样，先使用里程计推算机器人的位移，然后通过雷达点云贴合障碍物轮廓，修正里程计误差的方法，就是gmapping的核心算法
