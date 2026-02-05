/*
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
*/
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "go2_tutorial_inter/action/nav.hpp"

using namespace std::chrono_literals; 
using namespace std::placeholders;
class Go2NavClient : public rclcpp::Node
{
public:
    // 注意构造函数
    Go2NavClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : Node("go2_nav_client",options)
    {
        // 构造函数：初始化发布者、订阅者、服务等
        RCLCPP_INFO(this->get_logger(), "Hello from my ROS 2 node!");
        client_ = rclcpp_action::create_client<go2_tutorial_inter::action::Nav>(this,"nav");
    }
    bool connect_server(){
        while (!client_->wait_for_action_server(1s))
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"服务器连接中...");
            if (!rclcpp::ok())
            {
                return false;
            }
        }
        
        return true;
    }
    void send_request(float x){
        // 组织数据
        go2_tutorial_inter::action::Nav::Goal goal;
        goal.goal = x;
        // 发送 + 处理响应
        rclcpp_action::Client<go2_tutorial_inter::action::Nav>::SendGoalOptions options;
        // 目标值回调函数
        options.goal_response_callback = std::bind(&Go2NavClient::goal_response_callback,this,_1);
        // 连续反馈回调函数
        options.feedback_callback = std::bind(&Go2NavClient::feedback_callback,this,_1,_2);
        // 最终响应回调函数
        options.result_callback = std::bind(&Go2NavClient::result_callback,this,_1);
        client_->async_send_goal(goal,options);
    }
    // 析构--停止
    ~Go2NavClient(){
        client_->async_cancel_all_goals();
    }
private:
    // 成员变量（如发布者、订阅者）声明
    rclcpp_action::Client<go2_tutorial_inter::action::Nav>::SharedPtr client_;

    void goal_response_callback(std::shared_ptr<rclcpp_action::ClientGoalHandle<go2_tutorial_inter::action::Nav>> goal_handle){
        if (goal_handle)
        {
            RCLCPP_INFO(this->get_logger(),"目标请求被接收");
        }else{
            RCLCPP_ERROR(this->get_logger(),"目标值非法");
            rclcpp::shutdown();
        }
    }
    void feedback_callback(rclcpp_action::ClientGoalHandle<go2_tutorial_inter::action::Nav>::SharedPtr goal_handle,
        const std::shared_ptr<const go2_tutorial_inter::action::Nav::Feedback> feedback){
            (void)goal_handle;
            RCLCPP_INFO(this->get_logger(),"距离目标点还有%.2f米",feedback->distance);
    }
    void result_callback(const rclcpp_action::ClientGoalHandle<go2_tutorial_inter::action::Nav>::WrappedResult &result){
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(),"机器人的停止坐标：(%.2f,%.2f)",result.result->point.x,result.result->point.y);
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(this->get_logger(),"任务被取消！");
            break;
        default:
            RCLCPP_INFO(this->get_logger(),"未知异常！");
            break;
        }
        rclcpp::shutdown();
    }
};

int main(int argc, char ** argv)
{
    // 判断程序执行时参数个数是否合法；
    if (argc != 2)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"请提交一个整型的数据！");
        return 1;
    }
    // 初始化 ROS 2
    rclcpp::init(argc, argv);
    // 创建节点并开始自旋（处理回调）
    auto nav_client = std::make_shared<Go2NavClient>();
    // 连接服务端
    auto flag = nav_client->connect_server();

    // 发送请求数据并处理响应结果。若连接失败，直接退出。
    if (!flag)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"服务连接失败！");
        return 1;
    }

    nav_client->send_request(atof(argv[1]));

    rclcpp::spin(nav_client);
    // 关闭 ROS 2
    rclcpp::shutdown();
    return 0;
}