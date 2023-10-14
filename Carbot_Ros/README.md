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



