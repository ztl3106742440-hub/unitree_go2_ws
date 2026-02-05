"""
    编写键盘节点控制GO2
    1.引入键盘读取度三方
    2.编写ROS2节点
    3.编写按键映射逻辑

"""


import rclpy
from rclpy.node import Node
# 三个第三方库
import termios
import sys
import tty
import threading
from unitree_api.msg import Request
import json

msg = """
This node takes keypresses from the keyboard and publishes them
as unitree_api/msg/Request messages. It works best with a US keyboard layout.
---
Moving around:

q    w    e
a    s    d
z    x    c

For Holonomic mode (strafing), hold down the shift key:
---
Q    W    E
A    s    D
Z    x    C

anything else : stop

r/t : increase/decrease max speeds by 10%
f/g : increase/decrease only linear speed by 10%
w/b : increase/decrease only angular speed by 10%

h: Greet
j: Front Jump
k: Stretch
n: Sit Down
m: Stand Up from Sitting

y: Dance 1
u: Dance 2

CTRL-C to quit
"""

# 定义常量字典
ROBOT_SPORT_API_IDS = {
    "DAMP": 1001,               # 阻尼控制
    "BALANCESTAND": 1002,       # 平衡站立
    "STOPMOVE": 1003,           # 停止运动
    "STANDUP": 1004,            # 站立
    "STANDDOWN": 1005,          # 站立下降
    "RECOVERYSTAND": 1006,      # 恢复站立
    "EULER": 1007,              # 欧拉角控制
    "MOVE": 1008,               # 移动
    "SIT": 1009,                # 坐下
    "RISESIT": 1010,            # 从坐下恢复站立
    "SNITCHGAIT": 1011,         # 切换步态
    "TRIGGER": 1012,            # 触发
    "BODYHEIGHT": 1013,         # 身体高度调整
    "FOOTRAISEHEIGHT": 1014,    # 脚部抬起高度调整
    "SPEEDLEVEL": 1015,         # 速度级别调整
    "HELLO": 1016,              # 打招呼
    "STRETCH": 1017,            # 伸展
    "TRAJECTORYFOLLOW": 1018,   # 轨迹跟随
    "CONTINUOUSGAIT": 1019,     # 连续步态
    "CONTENT": 1020,            # 内容
    "WALLOW": 1021,             # 打滚
    "DANCE1": 1022,             # 舞蹈1
    "DANCE2": 1023,             # 舞蹈2
    "GETBODYHEIGHT": 1024,      # 获取身体高度
    "GETFOOTRAISEHEIGHT": 1025, # 获取脚部抬起高度
    "GETSPEEDLEVEL": 1026,      # 获取速度级别
    "SNITCHJOYSITCK": 1027,     # 切换操纵杆
    "POSE": 1028,               # 姿态
    "SCRAPE": 1029,             # 刮擦
    "FRONTFLIP": 1030,          # 前空翻
    "FRONTJUMP": 1031,          # 前跳
    "FRONTPOUNCE": 1032         # 前扑
}

sportModel = {
    'h': ROBOT_SPORT_API_IDS["HELLO"],
    'J': ROBOT_SPORT_API_IDS["FRONTJUMP"],
    'k': ROBOT_SPORT_API_IDS["STRETCH"],
    'n': ROBOT_SPORT_API_IDS["SIT"],
    'm': ROBOT_SPORT_API_IDS["RISESIT"],
    'y': ROBOT_SPORT_API_IDS["DANCE1"],
    'u': ROBOT_SPORT_API_IDS["DANCE2"],
}

moveBindings = {
    'w': (1, 0, 0, 0), #x*1, y*0, z*0, 角速度*0
    'e': (1, 0, 0, -1),
    'a': (0, 0, 0, 1),
    'd': (0, 0, 0, -1),
    'q': (1, 0, 0, 1),
    's': (-1, 0, 0, 0),
    'c': (-1, 0, 0, 1),
    'z': (-1, 0, 0, -1),
    'E': (1, -1, 0, 0),
    'W': (1, 0, 0, 0),
    'A': (0, 1, 0, 0),
    'D': (0, -1, 0, 0),
    'Q': (1, 1, 0, 0),
    'S': (-1, 0, 0, 0),
    'C': (-1, -1, 0, 0),
    'Z': (-1, 1, 0, 0),
}

speedBindings = {
    'r': (1.1, 1.1),# 线速度 *1.1, 角速度 * 1.1
    't': (.9, .9),
    'f': (1.1, 1),
    'g': (.9, 1),
    'v': (1, 1.1),
    'b': (1, .9),
}

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_ctrl_keyboard')
        self.pub = self.create_publisher(Request,"/api/sport/request",10)
        self.declare_parameter("speed",0.2) #假设x,y上线速度一致
        self.declare_parameter("angular",0.5)

        self.speed = self.get_parameter("speed").value
        self.angular = self.get_parameter("angular").value

    # 发布速度指令
    def publish(self,apid_id,x = 0.0,y = 0.0,z = 0.0):
        req = Request()
        req.header.identity.api_id = apid_id
        js = {"x":x,"y":y,"z":z}
        req.parameter = json.dumps(js)
        self.pub.publish(req)

# 读取按键
def getkey(settings):
    # 设置读取模式
    tty.setraw(sys.stdin.fileno())
    #获取一个按键
    key = sys.stdin.read(1)
    # 恢复终端原始设置
    termios.tcsetattr(sys.stdin,termios.TCSADRAIN,settings)
    # 返回按键值
    return key


def main():
    print(msg)
    # 读取键盘
    # 1.获取键盘标准输入流终端属性并返回
    settings = termios.tcgetattr(sys.stdin)
    
    # 2.ros2 node实现，需要单独子线程处理
    rclpy.init()
    # 子线程执行
    teleopNode = TeleopNode()
    spinner = threading.Thread(target=rclpy.spin,args=(teleopNode,))
    spinner.start()
    

    # 3.循环读取按键，并映射
    # 异常处理，用finally保证资源释放
    try:
        while True:
            key = getkey(settings)           
            # if key == 'h':
            #   teleopNode.publish(1016)
            #1.ctrl+c 结束--->平衡站立
            if key == '\x03':
                #teleopNode.publish(ROBOT_SPORT_API_IDS["BALANCESTAND"])
                break
            #2.运动模式切换（api_id）
            elif key in sportModel.keys():
                teleopNode.publish(sportModel[key])
            #3.运控模式（api_id,速度消息）
            elif key in moveBindings.keys():
                x_bind = moveBindings[key][0]
                y_bind = moveBindings[key][1]
                z_bind = moveBindings[key][3]
                teleopNode.publish(ROBOT_SPORT_API_IDS["MOVE"],
                                   x = x_bind * teleopNode.speed,
                                   y = y_bind * teleopNode.speed,
                                   z = z_bind * teleopNode.angular)
            #4.速度调整（api_id,速度消息）
            elif key in speedBindings.keys():
                s_bind = speedBindings[key][0]
                a_bind = speedBindings[key][1]
                teleopNode.speed = s_bind * teleopNode.speed
                teleopNode.angular = a_bind * teleopNode.angular
                print("current speed: %.5f,angular: %.5f" %(teleopNode.speed, teleopNode.angular))
            else :
                # 停止运动
                teleopNode.publish(ROBOT_SPORT_API_IDS["BALANCESTAND"])

    finally:
        rclpy.shutdown()
    #进入平衡站立

    

if __name__ == '__main__':
    main()