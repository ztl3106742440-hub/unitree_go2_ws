/*
  需求：
  动态调参的方式控制运动

*/
#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"
#include "sport_model.hpp"
#include "nlohmann/json.hpp"

using namespace std::chrono_literals;

class Go2Ctrl : public rclcpp::Node
{
public:
    Go2Ctrl() : Node("go2_ctrl")
    {
        // 构造函数：初始化发布者、订阅者、服务等
        RCLCPP_INFO(this->get_logger(), "Hello from go2_ctrl");

        // 设置参数
        this->declare_parameter("sport_api_id",ROBOT_SPORT_API_ID_BALANCESTAND);
        this->declare_parameter("x",0.0);
        this->declare_parameter("y",0.0);
        this->declare_parameter("z",0.0);

        // 创建发布对象
        req_pub_ = this->create_publisher<unitree_api::msg::Request>("/api/sport/request",10);
        timer_ = this->create_wall_timer(100ms,std::bind(&Go2Ctrl::on_timer,this));
    }

private:
    // 成员变量（如发布者、订阅者）声明
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    void on_timer(){
      // 创建速度指令并发布
      unitree_api::msg::Request request;

      auto id = this->get_parameter("sport_api_id").as_int();

      // 设置参数
      request.header.identity.api_id = ROBOT_SPORT_API_ID_MOVE; //运动模式

      if (id == ROBOT_SPORT_API_ID_MOVE)
      {

        
        nlohmann::json js;
        js["x"] = this->get_parameter("x").as_double();
        js["y"] = this->get_parameter("y").as_double();
        js["z"] = this->get_parameter("z").as_double();
        
        request.parameter = js.dump();
      }

      req_pub_->publish(request);
    }
};

int main(int argc, char ** argv)
{
    // 初始化 ROS 2
    rclcpp::init(argc, argv);
    // 创建节点并开始自旋（处理回调）
    auto node = std::make_shared<Go2Ctrl>();
    rclcpp::spin(node);
    // 关闭 ROS 2
    rclcpp::shutdown();
    return 0;
}