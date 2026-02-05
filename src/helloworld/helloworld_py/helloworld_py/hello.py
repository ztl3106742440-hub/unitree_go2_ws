'''
    打招呼

    流程：
    1.创建发布对象
    2.创建定时器
    3.定时器中调用发布对象发布指令

'''
# import rclpy
# from rclpy.node import Node
# from unitree_api.msg import Request

# class HelloWorldPy(Node):
#     def __init__(self):
#         super().__init__('helloworld_py')
#         #1.创建发布对象
#         self.pub = self.create_publisher(Request,"/api/sport/request",10)
#         #2.创建定时器
#         self.timer = self.create_timer(1.0,self.on_timer)
#         #3.定时器中调用发布对象发布指令
#     def on_timer(self):
#         request = Request()
#         request.header.identity.api_id = 1016
#         self.pub.publish(request)
# def main():
#     rclpy.init()
#     rclpy.spin(HelloWorldPy())
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()








'''
    打招呼 - 只执行一次

    流程：
    1. 创建发布对象
    2. 等待发布者建立连接
    3. 发布一次指令
'''
import rclpy
from rclpy.node import Node
from unitree_api.msg import Request

class HelloWorldPy(Node):
    def __init__(self):
        super().__init__('helloworld_py')
        # 1. 创建发布对象
        self.pub = self.create_publisher(Request, "/api/sport/request", 10)
        self.get_logger().info("节点已启动，等待发布者建立连接...")
        
    def publish_once(self):
        """发布一次消息"""
        # 等待发布者建立连接
        import time
        time.sleep(0.5)  # 等待0.5秒确保连接建立
        
        request = Request()
        request.header.identity.api_id = 1030
        self.pub.publish(request)
        self.get_logger().info(f"已发布消息，api_id: {request.header.identity.api_id}")

def main():
    rclpy.init()
    node = HelloWorldPy()
    
    # 发布一次消息
    node.publish_once()
    
    # 等待一小会儿确保消息发送
    import time
    time.sleep(0.1)
    
    node.get_logger().info("消息已发布，正在关闭节点...")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()