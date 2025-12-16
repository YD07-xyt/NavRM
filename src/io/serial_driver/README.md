# ROS2 serial

在上位机中使用ROS2，实现了与 Robomaster C板（下位机）进行串口通信，由上位机发布自定义数据给下位机


## 使用指南

1. 安装依赖 
    ```
    sudo apt install ros-humble-serial-driver
    ```

2. 编译
    ```
    colcon build --symlink-install
    ```

2. 启动串口:  

    更改 [serial_driver.yaml](config/serial_driver.yaml) 中的参数以匹配与C板通讯的串口
    
    ```
    sudo chmod 777 /dev/ttyACM0
    source install/setup.bash
    ros2 launch rm_serial_driver serial_driver.launch.py
    ```

## 发送和接收

1. Robomaster C板发送


2. Robomaster C板接收
    ```
    //ReceivedData
    typedef struct{
    uint8_t header;

    float vx; 
    float vy; 
    float vz; 

    uint16_t checksum;
    }__attribute__((packed)) ReceivedData_s;
    ```
Ubuntu上位机接收和发送包结构与C板类似

  
