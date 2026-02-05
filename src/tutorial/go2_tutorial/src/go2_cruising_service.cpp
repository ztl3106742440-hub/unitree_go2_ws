/*
    需求：处理客户端提交的数据（0或非0）。
        如果是0,停止巡航；非0,开始巡航。
        不管提交什么数据都要相应机器人位置信息
    流程：
        1.创建服务器
        2.回调函数处理请求，分情况处理（控制机器人巡航），最后响应结果。
*/
#include "rclcpp/rclcpp.hpp"
#include "go2_tutorial_inter/srv/cruising.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sport_model.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;
class Go2SCruisingService : public rclcpp::Node
{
public:
    Go2SCruisingService() : Node("go2_cruising_service")
    {
        // 构造函数：初始化发布者、订阅者、服务等
        RCLCPP_INFO(this->get_logger(), "Hello from my ROS 2 node!");
        
        this->declare_parameter("x",0.1);
        this->declare_parameter("y",0.0);
        this->declare_parameter("z",0.5);
        // 创建远程参数客户端
        param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this,"go2_ctrl");
        // 客户端连接到服务端
        while (!param_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                return;
            }
            RCLCPP_INFO(this->get_logger(),"服务连接中......");
        }
        RCLCPP_INFO(this->get_logger(),"成功连接到参数服务器!");
        // 通过订阅里程计获取机器人信息
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom",10,std::bind(&Go2SCruisingService::odom_cb,this,_1));
        // 创建服务器
        cru_service_ = this->create_service<go2_tutorial_inter::srv::Cruising>("cruising",std::bind(&Go2SCruisingService::cru_cb,this,_1,_2));
    }

private:
    // 成员变量（如发布者、订阅者）声明
    rclcpp::AsyncParametersClient::SharedPtr param_client_;
    rclcpp::Service<go2_tutorial_inter::srv::Cruising>::SharedPtr cru_service_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    geometry_msgs::msg::Point current_point;
    // 实时获取机器人位置
    void odom_cb(const nav_msgs::msg::Odometry::SharedPtr odom){
        current_point = odom->pose.pose.position;
    }
    void cru_cb(const go2_tutorial_inter::srv::Cruising::Request::SharedPtr request,
        const go2_tutorial_inter::srv::Cruising::Response::SharedPtr response){
        
        // 处理请求
        auto flag = request->flag;
        // 向 go2_ctrl 节点注入参数,如果是0,那么api_id = stopmove, 否则 = move
        int32_t id;
        if (flag != 0)
        {
            id = ROBOT_SPORT_API_ID_MOVE;
            RCLCPP_INFO(this->get_logger(),"开始巡航......");
        }else {
            id = ROBOT_SPORT_API_ID_STOPMOVE;
            RCLCPP_INFO(this->get_logger(),"结束巡航......");
        }
        // 向巡航节点注入参数
        param_client_->set_parameters({
            // rclcpp::Parameter("x",0.1),
            // rclcpp::Parameter("y",0.0),
            // rclcpp::Parameter("z",0.5),
            this->get_parameter("x"),
            this->get_parameter("y"),
            this->get_parameter("z"),
            rclcpp::Parameter("sport_api_id",id),
        });
        // 生成响应
        response->point = current_point;
    }

};

int main(int argc, char ** argv)
{
    // 初始化 ROS 2
    rclcpp::init(argc, argv);
    // 创建节点并开始自旋（处理回调）
    auto node = std::make_shared<Go2SCruisingService>();
    rclcpp::spin(node);
    // 关闭 ROS 2
    rclcpp::shutdown();
    return 0;
}