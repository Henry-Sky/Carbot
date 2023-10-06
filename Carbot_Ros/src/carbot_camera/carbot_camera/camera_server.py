import rclpy
from rclpy.node import Node
from carbot_camera.srv import Talk

class Camera_Server(Node):
    def __init__(self,name):
        super().__init__(name)
        # 创建服务节点
        self.srv = self.create_service(Talk, 'talk_test', self.reply_callback)
        
    def reply_callback(self,request,response):
        response.str2 = 'hello, I hear your voice:' + request.str1
        return response
        
        
 