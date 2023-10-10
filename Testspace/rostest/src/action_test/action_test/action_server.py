import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from action_tutorials_interfaces.action import Fibonacci


class Fib_Server(Node):
    def __init__(self,name):
        super().__init__(name)
        self.act_server = ActionServer(self,Fibonacci,
                                       "fib_server",
                                       self.act_callback)
        
    def act_callback(self, goal_handle):
        self.get_logger().info("get goal:" + str(goal_handle))
        # 计算
        sequence = [0, 1]
        for i in range(1, goal_handle.request.order):
            sequence.append(sequence[i] + sequence[i-1])

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = sequence
        return result
    

def main():
    rclpy.init()
    fib_server = Fib_Server("fib_server")
    rclpy.spin(fib_server)
    
main()