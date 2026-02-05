/*
  订阅twist消息，并转换为go2所需的request消息来控制
  实现：
    1.Request发布对象
    2.twist订阅对象
    3.在回调函数中转换消息并发布

*/

#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sport_model.hpp"
#include "nlohmann/json.hpp"

using namespace std::placeholders;

class TwistBridge : public rclcpp::Node
{
public:
    TwistBridge() : Node("my_node_name")
    {
        // 构造函数：初始化发布者、订阅者、服务等
        RCLCPP_INFO(this->get_logger(), "TwistBridge创建,将geomotry_msgs/msg/twist消息转换成unitree_api/msg/request消息！");
        //  1.Request发布对象
        request_pub_ = this->create_publisher<unitree_api::msg::Request>("/api/sport/request",10);
        twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel",10,std::bind(&TwistBridge::twist_cb,this,_1));
        //  2.twist订阅对象

    }

private:
    // 成员变量（如发布者、订阅者）声明
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr request_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    void twist_cb(const geometry_msgs::msg::Twist::SharedPtr twist){
      // 3.在回调函数中转换消息并发布
      unitree_api::msg::Request request;

      // 转换
      // 获取 twist 线速度与角速度
      double x = twist->linear.x;
      double y = twist->linear.y;
      double th = twist->angular.z;
      // 默认平衡站立
      auto api_id = ROBOT_SPORT_API_ID_BALANCESTAND;
      if (x != 0 || y != 0 || th != 0)
      {
        api_id = ROBOT_SPORT_API_ID_MOVE;
        /* code */
        nlohmann::json js;
        js["x"] = x;
        js["y"] = y;
        js["z"] = th;
        request.parameter = js.dump();
        RCLCPP_INFO(this->get_logger(),"current speed: %s",request.parameter.c_str());

      }
      request.header.identity.api_id = api_id;
      request_pub_->publish(request);
    }
};

int main(int argc, char ** argv)
{
    // 初始化 ROS 2
    rclcpp::init(argc, argv);
    // 创建节点并开始自旋（处理回调）
    auto node = std::make_shared<TwistBridge>();
    rclcpp::spin(node);
    // 关闭 ROS 2
    rclcpp::shutdown();
    return 0;
}