# Carbot
基于 Ros2 Humble 的智能物流小车

## 上位机:树莓派4B or 旭日x3派(地平线)
- 对接下位机库: Carbot_lib
- 改动: 对零号车型适配
- 计划: 添加Cvbot_lib库, 对调用opencv摄像头识别以及网页推流摄像头画面进行基本封装，对接收数据的python线程进行优化，随节点关闭后自动销毁

## 下位机:STM32(亚博智能的ros扩展板)
- 厂家驱动: Rosmaster (Carbot_Driver)  
- 改动: 将原保留位0号车型改为项目Carbot车型硬件  
- 计划: 向上位机发送电机累计编码器脉冲数据便于计算odom，删除姿态角解算，直接将九轴imu原始数据给上位机解算

## 中间件:Ros2 humble
### carbot_driver: 
- 订阅Twist并向下位机发送移动指令

### carbot_key_ctrl:
- 监听linux终端键盘
- 发布Twist控制消息

### carbot_odom: 
- 订阅Twist消息
- 计算里程计(获取速度后对时间积分)并发布
- 计划添加直接从编码器脉冲计算里程的方法，并对两种方法的准确度对比择优选择

### carbot_imu:
- 开启python接收下位机数据线程
- 获取陀螺仪数据并发布
- 依赖: scipy

### carbot_launch:
- 添加各包的参数配置
- 添加rviz2加载urdf文件

### carbot_camera:
- 发布摄像头画面

## 依赖(推荐)
- numpy
- scipy
- opencv
- ...

### 系统依赖
- ubuntu 22.04Lts
- ros2 humble
- python 3.10

### 功能包依赖
- opencv

        sudo apt install ros-humble-vision-opencv

- robot_localization:  

        sudo apt install ros-humble-robot-localization

- v4l2-camera:  

        sudo apt install ros-humble-v4l2-camera

- image-transport-plugins(可选):  

        sudo apt install libtheora-dev libogg-dev libboost-python-dev
        sudo apt install ros-humble-image-transport-plugins


## 计划
- 调用robot localization功能包,对里程计，陀螺仪数据融合
- 添加launch功能包，对节点参数进行系统化管理
- 优化3d模型，添加屏幕支架，添加底盘支撑，调整机械臂臂长，调整摄像头位置
- 将solidworks的3d模型转为urdf文件
- 对摄像头标定，畸变矫正
- 对物流场地预先建图
- 参考Navigation2进行导航
- 调取moveit对机械臂抓取进行运动控制优化