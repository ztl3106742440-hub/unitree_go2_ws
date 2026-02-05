/*
    需求:客户端向服务端发送整型数据,并接收服务器的响应结果.
    流程:
        1.判断提交的数据是否合法;
        2.ROS2初始化;
        3.创建自定义节点类对象;
        4.连接服务端;
        5.向服务端发送数据;
        6.处理响应结果;
        7.资源释放.
*/
#include "rclcpp/rclcpp.hpp"
#include "go2_tutorial_inter/srv/cruising.hpp"

using namespace std::chrono_literals;
class Go2SCruisingClient : public rclcpp::Node
{
public:
    Go2SCruisingClient() : Node("go2_cruising_client")
    {
        // 构造函数：初始化发布者、订阅者、服务等
        RCLCPP_INFO(this->get_logger(), "Hello from my ROS 2 node!");
        cru_client_ = this->create_client<go2_tutorial_inter::srv::Cruising>("cruising");
    }
    // 连接服务端
    bool connect_server(){
        while (!cru_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                return false;
            }
            RCLCPP_INFO(this->get_logger(),"服务连接中...");
        }
        return true;
    }
    // 发送请求并返回响应结果
    rclcpp::Client<go2_tutorial_inter::srv::Cruising>::FutureAndRequestId send_requrst(int32_t flag){
        auto req_ = std::make_shared<go2_tutorial_inter::srv::Cruising_Request>();
        req_->flag = flag;
        return cru_client_->async_send_request(req_);
    }

private:
    // 成员变量（如发布者、订阅者）声明
    rclcpp::Client<go2_tutorial_inter::srv::Cruising>::SharedPtr cru_client_;
};

int main(int argc, char ** argv)
{
    if (argc != 2)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"请提交一个整形数据");
        return 1;
    }
    // 初始化 ROS 2
    rclcpp::init(argc, argv);
    // 创建节点
    auto client_ = std::make_shared<Go2SCruisingClient>();
    // 连接服务器
    auto flag = client_->connect_server();
    // 连接成功后向服务器发送数据
    if (!flag)
    {
        return 1;
    }
    auto response_future = client_->send_requrst(atoi(argv[1]));
    // 处理响应结果
    if (rclcpp::spin_until_future_complete(client_,response_future) == rclcpp::FutureReturnCode::SUCCESS){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"响应成功");
        auto response_ = response_future.get();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"机器人坐标:(%.2f,%.2f)",response_->point.x,response_->point.y);
    }else {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"响应失败");
    }
    //释放资源


    // 创建节点并开始自旋（处理回调）
    // auto node = std::make_shared<Go2SCruisingClient>();
    // rclcpp::spin(node);
    // 关闭 ROS 2
    rclcpp::shutdown();
    return 0;
}