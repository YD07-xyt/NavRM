# NavRM

## 简介
本项目使用 small_point_lio 作为里程计 进行定位，建图 ，
     使用 small_gicp 进行全局重定位 ，
     TODO: 使用 grid_map 绘制2.5d图+esdf+ 骨干提取()
     TODO: 路径规划 参考pc-planner 

## env build


下载 rosdep
```bash
pip install rosdep
```

补全依赖:
```bash
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

下载 small_gicp
```bash
sudo apt install -y libeigen3-dev libomp-dev

git clone https://github.com/koide3/small_gicp.git
cd small_gicp
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j
sudo make install
```

下载串口库
```bash
sudo apt install ros-humble-serial-driver
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

```bash
chmod 777 /dev/ttyUSB0
```