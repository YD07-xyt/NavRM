# NavRM

## 简介
本项目使用 super_lio 作为里程计 进行定位，建图 ，
     使用 small_gicp 进行全局重定位 ，
     TODO: 使用 grid_map 绘制2.5d图+esdf+ 骨干提取()
     TODO: 路径规划 参考DDR-opt 

## TODO功能

### planner_controller 

mpc 的实现

### planner 
1.独立ros2 与 算法

2.独立 replan(状态机)

3.使用 grid_map 绘制2.5d图+esdf+ 骨干提取() + esdf增量更新 

### odom 

TODO:

### io

TODO: 

## 项目结构
.
├── 📂 bringup  
├── 📂 location  
│ ├── odometry    
│ ├── preprocess   
│ └── relocation  
│
├── 📂 planner/ # 规划器  
│ ├── 📂  path_searching  
│ ├── 📂  planner_controller  
│ ├── 📂  planner_map  
│ ├── 📂  trajectory_optimization  
│ ├── 📂  planner_ros2  
│ └── 📂  replanner_decision  

TODO:

## env build

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
