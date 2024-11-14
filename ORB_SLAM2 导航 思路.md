# ORB_SLAM2 导航 思路



- map->odom->base_link->camera_link
- 由ORB_SLAM2可得：
  - odom->camera_link
- 由手动测量可得：
  - camera_link->base_link
- 由odom->camera_link和camera_link->base_link可得
  - odom->base_link
- 由定位节点(TODO)可得：
  - map->base_link 进而
    - 由map->base_link和odom->base_link可得
      - map->odom
- map
  - octomap订阅ORB_SLAM2节点发布的点云话题生成地图？
- move_base
  - tf
    - map->odom->base_link->camera_link
  - odom
  - map
  - sensor topics
    - camera

