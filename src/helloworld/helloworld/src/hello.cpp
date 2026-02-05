/*

打招呼

流程：
1.创建发布对象
2.创建定时器
3.定时器中调用发布对象发布指令

*/

#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"

using namespace std::chrono_literals;  

class HelloWorld : public rclcpp::Node
{
public:
HelloWorld() : Node("helloworld")
{
// 构造函数：初始化发布者、订阅者、服务等
RCLCPP_INFO(this->get_logger(), "Hello from my ROS 2 node!");
// 1.创建发布对象
pub_ = this->create_publisher<unitree_api::msg::Request>("/api/sport/request",10);

// 2.创建定时器

timer_ = this->create_wall_timer(1s,std::bind(&HelloWorld::on_timer,this));

// 3.定时器中调用发布对象发布指令
}

private:
// 成员变量（如发布者、订阅者）声明
rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr pub_;
rclcpp::TimerBase::SharedPtr timer_;
void on_timer(){    
// 3.定时器中调用发布对象发布指令

unitree_api::msg::Request request;

request.header.identity.api_id = 1016;//打招呼协议

pub_->publish(request);
}
}; 

int main(int argc, char ** argv)
{
// 初始化 ROS 2

rclcpp::init(argc, argv);

// 创建节点并开始自旋（处理回调）

auto node = std::make_shared<HelloWorld>();

rclcpp::spin(node);

// 关闭 ROS 2

rclcpp::shutdown();

return 0;

}