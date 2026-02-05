"""
    需求:客户端向服务端发送整型数据,并接收服务器的响应结果.
    流程:
        1.判断提交的数据是否合法;
        2.ROS2初始化;
        3.创建自定义节点类对象;
        4.连接服务端;
        5.向服务端发送数据;
        6.处理响应结果;
        7.资源释放.
"""
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
import sys
from go2_tutorial_inter.srv import Cruising
from rclpy.task import Future

class Go2CruisingClient(Node):
    def __init__(self):
        super().__init__('go2_cruising_client')
        # 在这里添加初始化代码
        self.client = self.create_client(Cruising,"cruising")

    def connect_server(self):
        while not self.client.wait_for_service(1.0):
            if not rclpy.ok():
                get_logger("rclpy").error("连接被中断")
                return False
            self.get_logger().info("服务器连接中...")
        return True
    
    def send_request(self,flag) -> Future:
        # 组织数据
        req = Cruising.Request()
        req.flag = int(flag)
        # 发送请求
        return self.client.call_async(req)

def main():
    # 判断提交的数据是否合法;
    if len(sys.argv) != 2:
        get_logger("rclpy").error("请提交一个整型数据！")
        return
    # ROS2初始化;
    rclpy.init()
    # 创建自定义节点类对象;
    cru_client = Go2CruisingClient()
    # 连接服务端;
    flag = cru_client.connect_server()
    if not flag:
        return
    # 向服务端发送数据;
    future = cru_client.send_request(sys.argv[1])
    # 处理响应结果;
    rclpy.spin_until_future_complete(cru_client,future)
    if future.done():
        response : Cruising.Response = future.result()
        get_logger("rclpy").info("机器人坐标：(%.2f,%.2f)" % (response.point.x, response.point.y))
    else:
        get_logger("rclpy").info("请求失败！")
    # 资源释放.
    # rclpy.init()
    # rclpy.spin(Go2CruisingClient())
    rclpy.shutdown()

if __name__ == '__main__':
    main()