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

## 依赖(推荐)
### 系统依赖
- ubuntu 22.04Lts
- ros2 humble
- python 3.10

### 功能包依赖
- robot_localization:  

        sudo apt install ros-humble-robot-localization
- v4l2-camera:  

        sudo apt install ros-humble-v4l2-camera
- image_transport(可选):

        sudo apt install libtheora-dev libogg-dev libboost-python-dev
        cd [your workspace]
        git clone https://github.com/ros-perception/image_common.git --branch humble src/image_common
        git clone https://github.com/ros-perception/vision_opencv.git --branch humble src/vision_opencv
        git clone https://github.com/ros-perception/image_transport_plugins.git --branch humble src/image_transport_plugins
        colcon build