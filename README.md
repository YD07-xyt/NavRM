# NavRM

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
