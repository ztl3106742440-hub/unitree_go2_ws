"""
    导航服务端
    需求：
        1.处理客户端需求
            提交的请求为浮点数（前进距离）
            如果大于0,则前进；否则认为不合法
        2.处理客户端取消请求
            当客户端发送取消请求，则让机器人停止
        3.产生连续反馈和最终响应
            根据机器人当前坐标，结合起点坐标与前进距离，计算剩余距离并周期性反馈
            当机器人到达目标点，响应机器人坐标
    实现：
        1.创建action服务器，执行相关操作；
        2.向 go2_ctrl 注入参数控制机器人运动或停止；
        3.订阅机器人里程计获取坐标。

"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from go2_tutorial_inter.action import Nav
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from .sport_model import ROBOT_SPORT_API_IDS
from unitree_api.msg import Request
import json
from rclpy.action.server import CancelResponse,GoalResponse,ServerGoalHandle
import time
import math
from rclpy.executors import MultiThreadedExecutor

class Go2NavServer(Node):
    def __init__(self):
        super().__init__('go2_nav_server')

        # 创建空point
        self.point = Point()
        # 订阅里程计数据
        self.odom_sub = self.create_subscription(Odometry,"odom",self.odom_cb,10)
        # 设置巡航参数
        self.declare_parameter("x",0.3)
        # 创建速度控制的发布对象
        self.api_id = ROBOT_SPORT_API_IDS["BALANCESTAND"]
        self.req_pub = self.create_publisher(Request,"/api/sport/request",10)
        self.timer = self.create_timer(0.1,self.on_timer)

        # 创建 actionserver
        self.action_server = ActionServer(
            self,
            Nav,
            "nav",
            self.execute, # 主逻辑的处理函数
            goal_callback = self.goal_cb,
            cancel_callback = self.cancel_cb
        )
    def execute(self,goal_handle : ServerGoalHandle):

        feedback = Nav.Feedback()
        # 生成连续反馈
        while rclpy.ok():
            # 休眠
            time.sleep(0.5)
            # 组织数据
            # 剩余距离
            # 计算当前点 与 起点的距离差， 剩余距离 = 前进距离- 距离差
            dis_x = self.point.x - self.start_point.x
            dis_y = self.point.y - self.start_point.y
            dis = math.sqrt(math.pow(dis_x,2) + math.pow(dis_y,2))
            distance = goal_handle.request.goal - dis
            feedback.distance = distance
            # 发布
            goal_handle.publish_feedback(feedback)
            # 退出
            if distance < 0.2:
                self.api_id = ROBOT_SPORT_API_IDS["STOPMOVE"]
                break

        # 响应最终结果
        goal_handle.succeed()
        result = Nav.Result()
        result.point = self.point
        return result

        return Nav.Result()

    # 目标处理
    def goal_cb(self,goal_request: Nav.Goal):
        if goal_request.goal > 0.0:
            self.start_point = self.point
            self.get_logger().error("提交的数据合法，机器人开始运动")
            self.api_id = ROBOT_SPORT_API_IDS["MOVE"]
            return GoalResponse.ACCEPT
        else:
            self.get_logger().error("提交的数据非法")
            self.api_id = ROBOT_SPORT_API_IDS["STOPMOVE"]
            return GoalResponse.REJECT

    # 无条件取消
    def cancel_cb(self,cancle_request):
        self.api_id = ROBOT_SPORT_API_IDS["STOPMOVE"]
        return CancelResponse.ACCEPT

    # 里程计回调函数
    def odom_cb(self, odom : Odometry):
        self.point = odom.pose.pose.position

    def on_timer(self):
        req = Request()
        # 设置数据
        # self.get_logger().info("api_id = %d" % self.api_id)
        req.header.identity.api_id = self.api_id
        # 设置参数
        js = {
            "x": self.get_parameter("x").value,
            "y": 0.0,
            "z": 0.0,
        }
        req.parameter = json.dumps(js)
        self.req_pub.publish(req)

def main():
    rclpy.init()
    # rclpy.spin(Go2NavServer())
    # 使用多线程执行器运行节点
    node = Go2NavServer()
    # 创建多线程执行器
    executor = MultiThreadedExecutor()
    # 把节点添加到执行器
    executor.add_node(node)
    # 调用执行器的 spin
    executor.spin()

    rclpy.shutdown()

if __name__ == '__main__':
    main()