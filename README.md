# NavRM

## run
```bash
source install/setup.sh
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```
```bash
source install/setup.sh
ros2 launch patchworkpp patchworkpp.launch.py visualize:=false use_sim_time:=false cloud_topic:=/lio/cloud_world base_frame:=world
```
```bash
source install/setup.sh
ros2 launch terrain_analysis terrain_analysis.launch.py
```
```bash
source install/setup.sh
ros2 launch terrain_analysis_ext terrain_analysis_ext.launch.py
```
```bash
source install/setup.sh
ros2 launch fake_vel_transform  fake_vel_transform_launch.py 
```
```bash
source install/setup.sh
ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_launch.py
```
```bash
source install/setup.sh
ros2 launch super_lio Livox_mid360.py
```
```bash
source install/setup.sh
ros2 launch nav2 navigation.launch.py
```
```bash
source install/setup.sh
ros2 launch nav2 slam_toolbox.launch.py
```
```bash
ros2 run tf2_ros static_transform_publisher             --x 0.0 --y 0.0 --z 0 --roll 0 --pitch 0 --yaw 0             --frame-id map             --child-frame-id world
```
## env build


下载 rosdep
```bash
pip install rosdep
```
或用
```bash
wget http://fishros.com/install -O fishros && . fishros 
```
下载 rosdepc

补全依赖:
```bash
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```
或
```bash
rosdepc install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

## docker build

### 简单教程
[docker](./md_log/docker.md)

### 构建镜像 
```bash
docker build -t nav-rm-ros2 .
```

```bash
sudo docker run -it --name navrm-dev \
--env DISPLAY=$DISPLAY \
--net=host \
--privileged \
--volume /tmp/.X11-unix:/tmp/.X11-unix \
-v 主机项目绝对路径:容器项目绝对路径 \
nav-rm-ros2 /bin/bash
```

## 建图

## 保存pcd

运行./sh/map.sh
再开一个终端运行：
```bash
ros2 service call /map_save std_srvs/srv/Trigger
```


### pcd 转 pgm
运行 ./sh/pcd2pgm.sh
再开一个终端运行：
```bash
ros2 run nav2_map_server map_saver_cli -f ma
```


## run nav
