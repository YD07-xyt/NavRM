
```bash
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

```bash
sudo apt update && sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-behaviors
```

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