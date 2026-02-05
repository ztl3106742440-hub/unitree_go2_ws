"""
    需求：订阅里程计消息，当机器人位移超过阈值，输出当前坐标
    实现流程：
        1.订阅里程计消息
        2.在订阅方的回调函数中，计算当前机器人位置与上一个记录点的距离，
        如果大于阈值就输出坐标并更新
        3.生成第一个记录点
        当第一次订阅里程计消息时，就为记录点赋值
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

class Go2State(Node):
    def __init__(self):
        super().__init__('go2_state')
        # 第一次过去里程计消息的标记
        self.is_first = True
        # 记录点坐标对应的变量
        self.last_x = 0.0
        self.last_y = 0.0
        # 阈值参数
        self.declare_parameter("distance",0.5)

        self.odom_sub = self.create_subscription(Odometry,"odom",self.odom_cb,10)
    
    def odom_cb(self,odom : Odometry):
        # 获取当前坐标
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        
        # 设置第一个记录点
        if self.is_first:
            self.last_x = x
            self.last_y = y
            self.is_first = False
            self.get_logger().info("原点坐标:(%.2f,%.2f)" % (x,y))
            return
        
        # 计算是否超出距离
        dis_x = x - self.last_x
        dis_y = y - self.last_y

        distance = math.sqrt(dis_x**2 + dis_y**2)
        # 判断
        if distance >= self.get_parameter("distance").value:
            # 输出坐标点
            self.get_logger().info("坐标:(%.2f,%.2f)" % (x,y))
            # 记录点数据更新
            self.last_x = x
            self.last_y = y


def main():
    rclpy.init()
    rclpy.spin(Go2State())
    rclpy.shutdown()

if __name__ == '__main__':
    main()