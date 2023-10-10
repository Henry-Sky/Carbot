import rclpy
from rclpy.node import Node
from carbot_interfaces.srv import Test

class Server_Test(Node):
    def __init__(self,name):
        super().__init__(name)
        self.server_test = self.create_service(Test,"server_test",self.server_callback) 
        self.timers_1 = self.create_timer(0.01,self.time1_callback)
        self.timers_2 = self.create_timer(0.01,self.time2_callback)
        self.time1_num = 0.0
        self.time2_num = 0.0
        
    def time1_callback(self):
        self.time1_num += 0.1
        
    def time2_callback(self):
        self.time2_num += 0.5
        
    def server_callback(self,request,response):
        if request.get_num == True:
            response.num1 = self.time1_num
            response.num2 = self.time2_num
            return response
        else:
            response.num1 = 0.0
            response.num2 = 0.0
            return response
            
            
def main():
    rclpy.init()
    server_test = Server_Test("server_test_node")
    rclpy.spin(server_test)
    server_test.destroy_node()
    rclpy.shutdown()      