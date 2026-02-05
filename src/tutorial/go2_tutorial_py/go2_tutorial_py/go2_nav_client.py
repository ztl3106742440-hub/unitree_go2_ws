"""
    导航客户端
    需求：
        客户端发送数据到服务端，并发送服务端的响应结果
    流程：
        1.判断程序执行时参数个数是否合法；
        2.初始化 ROS2；
        3.创建action客户端对象；
        4.连接服务器；
        5.发送请求数据并处理响应结果。若连接失败，直接退出。
        6.调用 spin 函数，并传入节点对象指针；
        7.释放资源。
"""
import rclpy
from rclpy.node import Node
import sys
from rclpy.logging import get_logger
from rclpy.action import ActionClient
from go2_tutorial_inter.action import Nav

class Go2NavClient(Node):
    def __init__(self):
        super().__init__('go2_nav_client')
        self.client = ActionClient(self,Nav,"nav")
        self.done = False

    def connect_server(self):
        while not self.client.wait_for_server(1.0):
            self.get_logger().info("服务连接中...")
            if not rclpy.ok():
                return False
        return True
    
    def send_goal(self,goal):
        # 组织数据
        goal_msg = Nav.Goal()
        goal_msg.goal = goal
        # 发送数据，并处理响应
        future : rclpy.task.Future = self.client.send_goal_async(goal_msg,self.feedback_callback)
        future.add_done_callback(self.goal_response)

    def goal_response(self,future: rclpy.task.Future):
        goal_handle : rclpy.action.client.ClientGoalHandle = future.result()
        # self.get_logger(),info("%s" % goal_handle.__str__()) ClientGoalHandle
        if goal_handle.accepted:
            self.get_logger().info("目标请求被接收")
            future : rclpy.task.Future = goal_handle.get_result_async()
            future.add_done_callback(self.result_response)
        else:
            self.get_logger().info("目标请求被拒绝")
            # rclpy.shutdown()
            self.done = True  # 标记完成
            self._done = True

    def result_response(self,future: rclpy.task.Future):
        result : Nav.Result = future.result().result
        self.get_logger().info("机器人到达后坐标：(%.2f,%.2f)" % (result.point.x, result.point.y))
        self.done = True  # 标记完成
        self._done = True

    def feedback_callback(self,fb_msg):
        # 处理连续反馈
        fb : Nav.Feedback = fb_msg.feedback
        self.get_logger().info("距离目标还有 %.2f 米" % fb.distance)

def main():
    # 判断程序执行时参数个数是否合法；
    if len(sys.argv) != 2:
        get_logger("rclpy").error("请提交一个浮点类型的参数！")
        return
    # 初始化 ROS2
    rclpy.init()
    # 创建cation客户端对象
    go2_nav_client = Go2NavClient()
    # 连接服务端
    flag = go2_nav_client.connect_server()
    # 连接成功后，发送请求数据，并处理响应结果。连接失败，直接退出。
    if not flag:
        get_logger("rclpr").error("服务器连接失败")
        rclpy.shutdown()
        return
    
    get_logger("rclpr").info("连接成功，发送请求！")

    # 发送请求
    go2_nav_client.send_goal(float(sys.argv[1]))

    # ⭐ 关键：可控 spin，而不是 rclpy.spin()
    while rclpy.ok() and not go2_nav_client.done:
        rclpy.spin_once(go2_nav_client, timeout_sec=0.1)

    # 清理资源
    go2_nav_client.destroy_node()

    # rclpy.spin(go2_nav_client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()