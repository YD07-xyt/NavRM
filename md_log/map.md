# map log problem

## 12.17

1. 框架： livox_ros_driver2 -> small_point_lio -> terrain_analysis ->  terrain_analysis_ext ->pointcloud_to_laserscan 

目前输出scan

2. mid360_driver不能输出点云，问题未解决 ，改换为livox_ros_driver2无影响

3. tf变换 map->odom->base_link->lidar_frame
   (1). small_point_lio的mid360.yaml文件中重力配置改为 gravity: [-4.84400, -1.03554, -8.4757] 
   
   (2). small_point_lio的launch文件自带static_base_link_to_livox_frame,并集成在bringup/map.launch.py中
   
   (3). small_point_lio的odom topic名字为Odometer，首字母大写
   
   (4). 如果topic有Odometer,无输出，怀疑是雷达驱动的问题，已解决
   
   (5). pointcloud_to_laserscan中的launch文件中参数'target_frame'写入odom / map (写入lidar_frame,会出现scan倾斜，如果lidar斜放)
   
   (6). map->odom 手动发布 或 ./tf.sh, bringup/map.launch.py中无静态发布，后面加入

4. bash脚本
    增加了各个包的启动脚本，
    main.sh中不含division_ext.sh

5. 点云切割
    (1).linefit_ground_segmentation_ros2 
    
        a.该包无法分割地面和障碍物，待解决
    
        b.输出的点云无深度信息(来源：pointcloud_to_laserscan报错)
    
        c.传入雷达驱动点云，node运行失败，而且livox_ros_driver2中xfer_format改为4 出现rviz无法兼容2
        种点云的问题(1次)，pb开源无问题， 待解决
    
    (2). terrain_analysis && terrain_analysis_ext
        
        a.移植pb包，加入launch独立启动文件，独立的yaml文件
        
        b.修改了传入的点云topic: cloud_registered  odom的topic :Odometry
        
        c.launch文件中yaml传值，硬编码，待改进

        d. terrain_analysis的 yaml文件有实车高大概0.47 对有效点云的范围经行修改 minRelZ: -1.5 maxRelZ: 0.55 ， 目前为测试下坡 ，minRelZ 设为0.0,其实 terrain_analysis yaml中considerDrop: False   # 考虑凹下去的地面,开启则将相对于地面点的高度取绝对值  目前处于True ,minRelZ未调试 , 测试效果very好 
        e. terrain_analysis坡度测试有待考虑
    
6. 点云3d->2d   pointcloud_to_laserscan 
        
        a.传入/terrain_map_ext(terrain_analysis中terrain_map传给了terrain_analysis_ext)

        b.'target_frame': 'odom'
7. bringup
    (1) map.launch.py  汇总前面的包的启动
    (2) map.launch.py 中的terrain_analysis && terrain_analysis_ext 参数传入
    ```py
             PathJoinSubstitution(
                 [
                     FindPackageShare("terrain_analysis(ext)"),
                     "config",
                     "param.yaml",
                 ]
             )
    ```

    rviz出现对应话题 显示 切割后的地面 而非 障碍物 

    ？？？？？？？？？

    硬编码可以 

    ？？？？？？？？？

    666,不🧂了

    未解决

    (3).map.launch.py 启动rviz 无法读取src/bringup/rviz/map.rviz 未解决

8. cmake 
类似：
```bash
CMake Error at CMakeLists.txt:37 (add_library):
  Target "Example" links to target "Boost::date_time" but the target was not
  found.  Perhaps a find_package() call is missing for an IMPORTED target, or
  an ALIAS target is missing?
```

解决方案：
在cmakelists中加入

```bash
#显式查找
find_package(Boost REQUIRED COMPONENTS date_time)
```
链接
这里的Example是可执行文件
```bash
target_include_directories( Example
  PUBLIC
  ${Boost_LIBRARIES}
  #其他
)
```

9.launch找不到导入模块

原因： vscode找不到ros2的解释器

在.vscode的setting.json中加入
注意ros2的版本
```bash
    "python.analysis.extraPaths": [
        "/opt/ros/humble/lib/python3.10/site-packages"
    ]
```

10.source install/setup.bash 失败

原因：zsh为配置好
     bash正常

## 12.18

1. 自瞄接口aim_driver
    (1) 自定义msg
        a.msg本身文件第1个字母must 大写 belike: Control.msg 并非 control,msg
        b.cmake编译找不到头文件"aim_driver/msg/Control.hpp"，
            原因： Control.hpp第1个字母大写了 ，应该是aim_driver/msg/control.hpp
            ？？？？？？？？？？？？？？？？
            what can I say ?
            ？？？？？？？？？？？？？？？？
        c.  "AimDriver::msg::Control" 报错
        原因： AimDriver为类的名字， 实际注册为的包名
        改为aim_driver::msg::Control
        类似：
```bash
    error: parse error in template argument list   
    send_pub_ = this->create_publisher<AimDriver::msg::Control>("/sentry_to_aim_data", 10);
```
