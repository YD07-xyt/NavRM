# NavRM

## env build

ros2下载
```bash
wget http://fishros.com/install -O fishros && . fishros
```

## docker build

### 简单教程
[docker](./md_log/docker.md)

### 构建镜像 
```bash
docker build -t nav-rm-ros2 .
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