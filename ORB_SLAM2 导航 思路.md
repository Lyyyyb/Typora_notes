# ORB_SLAM2 导航 思路



- map->odom->base_link->camera_link
- 由ORB_SLAM2可得：
  - odom->camera_link
- 由手动测量可得：
  - camera_link->base_link
- 由odom->camera_link和camera_link->base_link可得
  - odom->base_link
- map->odom 重合？？
- 回环检测 map->base_link
- 由定位节点(TODO)可得： ORB_SLAM2 自身的定位功能？？？
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





- octomap_server(/projected_map) -> map_server  二维栅格地图

- **`move_base`**
  - **功能**：`move_base` 是 ROS 中最常用的路径规划框架，它主要用于二维路径规划，但也可以扩展和定制以支持三维路径规划。
  - 扩展
    - **`move_base`** 默认使用2D栅格地图进行路径规划，但通过结合`octomap`和`3d_navigation`，可以使其支持三维环境中的路径规划。
    - **集成**：`move_base` 可以与三维地图（例如Octomap）配合使用，结合合适的规划算法（如`sbpl_lattice_planner_3d`）来执行三维路径规划。
