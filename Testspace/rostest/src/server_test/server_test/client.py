import rclpy
from rclpy.node import Node
from carbot_interfaces.srv import Test

class Client_Test(Node):
    def __init__(self,name):
        super().__init__(name)
        self.client_test = self.create_client(Test, 'server_test')
        self.req = Test.Request()
    
    def send_request(self):
        self.req.get_num = True
        self.future = self.client_test.call_async(self.req)
        
def main():
    rclpy.init()
    client_test = Client_Test("client_test")
    client_test.send_request()
    rclpy.spin_once(client_test)
    if client_test.future.done():
        response = client_test.future.result()
        client_test.get_logger().info("test res = %f, %f" % (response.num1,response.num2))
    client_test.destroy_node()
    rclpy.shutdown()