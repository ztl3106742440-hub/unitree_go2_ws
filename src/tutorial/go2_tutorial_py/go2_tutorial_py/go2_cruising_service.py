"""
     需求：处理客户端提交的数据（0或非0）。
        如果是0,停止巡航；非0,开始巡航。
        不管提交什么数据都要相应机器人位置信息
    流程：
        1.创建服务器
        2.回调函数处理请求，分情况处理（控制机器人巡航），最后响应结果。
"""
import rclpy
from rclpy.node import Node
from go2_tutorial_inter.srv import Cruising
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from unitree_api.msg import Request
from .sport_model import ROBOT_SPORT_API_IDS
import json

class Go2CruisingService(Node):
    def __init__(self):
        super().__init__('go2_cruising_service')
        # 创建服务器
        self.service = self.create_service(Cruising,"cruising",self.cru_cb)
        # 创建空point
        self.point = Point()
        # 订阅里程计数据
        self.odom_sub = self.create_subscription(Odometry,"odom",self.odom_cb,10)
        # 设置巡航参数
        self.declare_parameter("x",0.0)
        self.declare_parameter("y",0.0)
        self.declare_parameter("z",0.5)
        # 创建速度控制的发布对象
        self.api_id = ROBOT_SPORT_API_IDS["BALANCESTAND"]
        self.req_pub = self.create_publisher(Request,"/api/sport/request",10)
        self.timer = self.create_timer(0.1,self.on_timer)

    def on_timer(self):
        req = Request()
        # 设置数据
        req.header.identity.api_id = self.api_id
        # 设置参数
        js = {
            "x": self.get_parameter("x").value,
            "y": self.get_parameter("y").value,
            "z": self.get_parameter("z").value,
        }
        req.parameter = json.dumps(js)
        self.req_pub.publish(req)

    def cru_cb(self,request : Cruising.Request, response : Cruising.Response):
        # 处理请求
        flag = request.flag

        if flag == 0:
            self.get_logger().info("结束巡航")
            self.api_id = ROBOT_SPORT_API_IDS["STOPMOVE"]
        else:
            self.get_logger().info("开始巡航")
            self.api_id = ROBOT_SPORT_API_IDS["MOVE"]

        # 产生响应
        response.point = self.point
        return response
    # 里程计回调函数
    def odom_cb(self, odom : Odometry):
        self.point = odom.pose.pose.position

def main():
    rclpy.init()
    rclpy.spin(Go2CruisingService())
    rclpy.shutdown()

if __name__ == '__main__':
    main()