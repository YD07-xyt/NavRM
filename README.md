# NavRM

## env build

ros2下载
```bash
wget http://fishros.com/install -O fishros && . fishros
```

## docker build

### 构建镜像 
```bash
docker build -t nav-rm-ros2 .
```

### docker 镜像源配置

#### 测试镜像源是否可用
```bash
docker pull <mirror_url>/nginx:latest
```
#### 配置镜像源
```bash
sudo vim /etc/docker/daemon.json
```
#### 重启 Docker 服务
```bash
# 停止 Docker 服务                                
sudo systemctl stop docker
# 重启 Docker 服务
sudo systemctl daemon-reload
sudo systemctl start docker
```bash

#### 镜像源
  https://docker.1ms.run
  https://docker.m.daocloud.io/
  https://docker.1panel.live/
  https://hub.rat.dev/
  https://docker.1ms.run/
  https://docker.xuanyuan.me/
  https://dislabaiot.xyz/
  https://doublezonline.cloud/
  https://xdark.top/

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