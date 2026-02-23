# pb rm 仿真 

### Build

```sh
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

```sh
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=release
```

### Running

启动仿真环境

```sh
ros2 launch rmu_gazebo_simulator bringup_sim.launch.py
```
> [!NOTE]
> **注意：需要点击 Gazebo 左下角橙红色的 `启动` 按钮**

#### run ign_sim_pointcloud_tool

```zsh
source install/setup.zsh
ros2 launch ign_sim_pointcloud_tool ign_sim_pointcloud_tool.launch.py
```


#### Test Commands

控制机器人移动

```sh
ros2 run rmoss_gz_base test_chassis_cmd.py --ros-args -r __ns:=/red_standard_robot1/robot_base -p v:=0.3 -p w:=0.3
#根据提示进行输入，支持平移与自旋
```

机器人云台

```sh
ros2 run rmoss_gz_base test_gimbal_cmd.py --ros-args -r __ns:=/red_standard_robot1/robot_base
#根据提示进行输入，支持绝对角度控制
```

机器人射击

```sh
ros2 run rmoss_gz_base test_shoot_cmd.py --ros-args -r __ns:=/red_standard_robot1/robot_base
#根据提示进行输入
```

#### 切换仿真世界

修改 [gz_world.yaml](./rmu_gazebo_simulator/config/gz_world.yaml) 中的 `world`。当前可选: `rmul_2024`, `rmuc_2024`, `rmul_2025`, `rmuc_2025`
