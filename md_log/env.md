# 环境配置

```bash
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

下载串口
```bash
sudo apt install ros-humble-serial-driver
```

```bash
sudo apt install -y libeigen3-dev libomp-dev

git clone https://github.com/koide3/small_gicp.git
cd small_gicp
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j
sudo make install
```

## rosdepc (rosdep的国内版本)
### 下载 rosdepc
```bash
wget http://fishros.com/install -O fishros && . fishros 
```
下载 rosdepc


### 使用
```bash
rosdepc install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```