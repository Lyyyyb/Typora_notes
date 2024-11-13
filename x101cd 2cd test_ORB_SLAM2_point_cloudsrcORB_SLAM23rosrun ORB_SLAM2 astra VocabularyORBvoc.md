```
cd ~
cd test_ORB_SLAM2_point_cloud/src/ORB_SLAM2
rosrun ORB_SLAM2 astra Vocabulary/ORBvoc.txt Examples/ROS/ORB_SLAM2/Astra.yaml

cd ~
roslaunch pointcloud_mapping  astra.launch

cd ~
roslaunch astra_launch astra_adv.launch

```

