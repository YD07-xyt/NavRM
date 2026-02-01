# isaac_sim_pointcloud_tool

This package converts pointcloud in Ignition Gazebo to velodyne format. This is a ROS package. It subscribes to the LiDAR rostopic which is published by Ignition Gazebo. And it republish a LiDAR rostopoic in Velodyne format.

Some SLAM algorithm needs pointcloud in Velodyne format so that it can extract corner points. But Isaac ROS only send pointcloud contains XYZ information. This package helps to convert pointcloud to velodyne format.



仿真环境中，实际上 point pattern 为 velodyne 样式的机械式扫描。此外，由于仿真器中输出的 PointCloud 缺少部分 field，导致 point_lio 无法正常估计状态，故仿真器输出的点云经过 ign_sim_pointcloud_tool 处理添加 field。time