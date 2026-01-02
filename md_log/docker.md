# docker 

## 下载docker
```bash
curl -fsSL https://get.docker.com | bash -s docker --mirror Aliyun
```

## docker镜像源配置

### 测试镜像源是否可用
```bash
docker pull <mirror_url>/nginx:latest
```
example:
```bash
docker pull docker.1ms.run/nginx:latest
```


### 配置镜像源
```bash
sudo vim /etc/docker/daemon.json
```

example:
```bash
{
  "registry-mirrors": 
  [
  "https://docker.1ms.run"
]
}
```


### 重启 Docker 服务
```bash
# 停止 Docker 服务                                
sudo systemctl stop docker
# 重启 Docker 服务
sudo systemctl daemon-reload
sudo systemctl start docker
```

### 镜像源
  https://docker.1ms.run
  https://docker.m.daocloud.io/
  https://docker.1panel.live/
  https://hub.rat.dev/
  https://docker.1ms.run/
  https://docker.xuanyuan.me/
  https://dislabaiot.xyz/
  https://doublezonline.cloud/
  https://xdark.top/


## dockerfile 中 git 代码

加入
```bash
RUN git config --global http.proxy  http://主机ip: clash的代理ip
```

### 主机clash中开全局代理，局域网链接 ，代理ip查看 

example:
```bash
RUN git config --global http.proxy http://192.168.1.251:7897 && \
    git config --global https.proxy http://192.168.1.251:7897
```

### 查看主机IP:
```bash
ip a  
```


## 构建镜像 
```bash
docker build -t nav-rm-ros2 <dockerfile所在的目录>
```

example:
```bash
docker build -t nav-rm-ros2 .
```

## Run 容器
```bash
sudo docker run -it --name navrm-dev 
            --env DISPLAY=$DISPLAY \
            --net=host \
            --privileged \
            --volume /tmp/.X11-unix:/tmp/.X11-unix \
            -v /home/ma/Nav/NavRM:/home/ma/NavRM \  
            -v 项目绝对路径:docker容器中的项目绝对路径
            nav-rm-ros2 /bin/bash
```

```bash
sudo docker run -it --name navrm-dev \
--env DISPLAY=$DISPLAY \
--net=host \
--privileged \
--volume /tmp/.X11-unix:/tmp/.X11-unix \
-v /home/ma/Nav/NavRM:/home/ma/NavRM \
nav-rm-ros2 /bin/bash
```

## 基本操作
删除容器：
```bash
sudo docker stop navrm-dev
sudo docker rm navrm-dev
```

启动容器：
```bash
sudo docker exec -it navrm-dev /bin/bash
```



### rviz2无法显示
主机：
```bash
echo $DISPLAY
```
输出：例如： 
```bash
：0
```
主机上在输入
```bash
xhost +
```

docker容器中：
```bash
export DISPLAY=:0
```

***其中的:0 为主机上的输出***

### docker中无法运行 mid360_driver

在 Docker 中运行时：

#### 网络模式问题：默认的 Bridge 模式经常会导致 UDP 绑定失败。
解决方法：启动 Docker 容器时，请务必添加 --net=host 参数。这让容器直接使用宿主机的网络栈，雷达驱动才能正确绑定端口。

#### 权限限制：雷达驱动需要底层网络访问权限。
解决方法：启动容器时添加 --privileged 参数。