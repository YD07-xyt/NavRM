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
            -v 项目绝对路径:docker容器中的项目绝对路径
            nav-rm-ros2 /bin/bash
```

```bash
sudo docker run -it --name navrm-dev   
            -v /home/ma/Nav/NavRM:/root/NavRM   
            nav-rm-ros2 /bin/bash
```
