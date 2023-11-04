# Carbot
基于 Ros2 Humble 的智能物流小车

![carbot](https://github.com/Henry-Sky/Carbot/blob/master/Model/img/carbot.jpg)

## 上位机:树莓派4B or 旭日x3派(地平线)
- 对接下位机库: Carbot_lib
- 改动: 对零号车型适配
- 计划: 添加Cvbot_lib库, 对调用opencv摄像头识别以及网页推流摄像头画面进行基本封装，对接收数据的python线程进行优化，随节点关闭后自动销毁

## 下位机:STM32(亚博智能的ros扩展板)
- 厂家驱动: Rosmaster (Carbot_Driver)  
- 改动: 将原保留位0号车型改为项目Carbot车型硬件  
- 计划: 向上位机发送电机累计编码器脉冲数据便于计算odom，删除姿态角解算，直接将九轴imu原始数据给上位机解算

## 中间件:Ros2 humble

### carbot_launch:
- 启动该项目所需的所有节点
- 一个基于pyqt5的实时显示订阅codeinfo信息的图形界面

### carbot_interfaces:
- 包含所需的自定义消息

### carbot_driver: 
- 订阅电机，机械臂的控制信息
- 键盘控制

### carbot_location: 
- 订阅控制信息，计算里程，建立坐标系 

### carbot_vision:
- 调用摄像头，进行二维码识别，物块识别，靶心识别等

### carbot_plan:
- 该项目的任务调度节点，控制任务的开始与结束


## 依赖(推荐)
- numpy
- opencv
- pyqt5
- ...

### 系统依赖
- ubuntu 22.04Lts
- ros2 humble
- python 3.10

### 功能包依赖
- opencv

        sudo apt install ros-humble-vision-opencv

- tf_transformations

        sudo apt install ros-humble-tf-transformations

- robot_localization:  

        sudo apt install ros-humble-robot-localization

### 开机启动 
添加  
        sudo cp carbot_start.sh /etc/init.d/  
        sudo chmod 777 carbot_start.sh  
        sudo update-rc.d carbot_start.sh defaults (优先级)

移除  
        cd /etc/init.d  
        sudo update-rc.d -f carbot_start.sh remove


## 计划
- 优化3d模型，添加屏幕支架，添加底盘支撑，调整机械臂臂长，调整摄像头位置
- 对物流场地预先建图
- 参考Navigation2进行导航
- 调取moveit对机械臂抓取进行运动控制优化