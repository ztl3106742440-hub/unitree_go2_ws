/*
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
*/
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "go2_tutorial_inter/action/nav.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sport_model.hpp"

using namespace rclcpp_action;
using namespace std::placeholders;
using namespace std::chrono_literals;
class Go2NavServer : public rclcpp::Node
{
public:
    Go2NavServer() : Node("go2_nav_server")
    {
        // 构造函数：初始化发布者、订阅者、服务等
        RCLCPP_INFO(this->get_logger(), "Hello from my ROS 2 node!");
        this->declare_parameter("x",0.1);
        this->declare_parameter("error",0.2);
        // 创建客户端参数，用于向 go2_ctrl 注入参数
        param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this,"go2_ctrl");
        // 连接参数服务
        while (!param_client_->wait_for_service(1s))
        {
            // ctrl + c 连接超时，退出
            if(!rclcpp::ok()){
                return;
            }
            RCLCPP_INFO(this->get_logger(),"参数服务连接中...");
        }
        

        // 创建里程计订阅对象
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom",10,std::bind(&Go2NavServer::odom_cb,this,_1));

        // 创建导航服务端
        nav_server_ = rclcpp_action::create_server<go2_tutorial_inter::action::Nav>(
            this,
            "nav",
            std::bind(&Go2NavServer::goal_cb,this,_1,_2),
            std::bind(&Go2NavServer::cancel_cb,this,_1),
            std::bind(&Go2NavServer::accepted_cb,this,_1)
        );
    }

private:
    // 参数客户端
    rclcpp::AsyncParametersClient::SharedPtr param_client_;
    // 成员变量（如发布者、订阅者）声明
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp_action::Server<go2_tutorial_inter::action::Nav>::SharedPtr nav_server_;
    geometry_msgs::msg::Point current_point,start_point;

    // 订阅里程计消息
    void odom_cb(const nav_msgs::msg::Odometry::SharedPtr odom_){
        // 实时获取机器人坐标
        current_point = odom_->pose.pose.position;
    }

    // /// Signature of a callback that accepts or rejects goal requests.
    // using GoalCallback = std::function<GoalResponse(
    //         const GoalUUID &, std::shared_ptr<const typename ActionT::Goal>)>;
    GoalResponse goal_cb(const GoalUUID &uuid, std::shared_ptr<const go2_tutorial_inter::action::Nav::Goal> goal){
        (void)uuid;
        // 获取前进距离 
        float goal_dis = goal->goal;
        // 判断合法性
        // 合法则控制机器人前进，否则拒绝任务
        if (goal_dis > 0.0)
        {
            RCLCPP_INFO(this->get_logger(),"前进%.2f米",goal_dis);
            start_point = current_point;
            // 让 go2 运动
            param_client_->set_parameters({
                rclcpp::Parameter("sport_api_id",ROBOT_SPORT_API_ID_MOVE),
                this->get_parameter("x")
            });

            return GoalResponse::ACCEPT_AND_EXECUTE;
        }else {
            RCLCPP_ERROR(this->get_logger(),"前进%.2f米,距离小于0,非法数据！",goal_dis);
            return GoalResponse::REJECT;
        }

    }
    // /// Signature of a callback that accepts or rejects requests to cancel a goal.
    // using CancelCallback = std::function<CancelResponse(std::shared_ptr<ServerGoalHandle<ActionT>>)>;
    // 处理客户端提交的取消请求
    CancelResponse cancel_cb(std::shared_ptr<ServerGoalHandle<go2_tutorial_inter::action::Nav>> server_goal_handle){
        (void)server_goal_handle;
        // 
        RCLCPP_INFO(this->get_logger(),"取消任务！");
        // 让机器狗停止运动
        stop_move();
        // 无条件接收取消请求
        return CancelResponse::ACCEPT;
    }
    void stop_move(){
        param_client_->set_parameters({
            rclcpp::Parameter("sport_api_id",ROBOT_SPORT_API_ID_STOPMOVE),
            rclcpp::Parameter("x",0.0),
            rclcpp::Parameter("y",0.0),
            rclcpp::Parameter("z",0.0),
        });
    }
    // /// Signature of a callback that is used to notify when the goal has been accepted.
    // using AcceptedCallback = std::function<void (std::shared_ptr<ServerGoalHandle<ActionT>>)>;
    // 主逻辑
    void accepted_cb (std::shared_ptr<ServerGoalHandle<go2_tutorial_inter::action::Nav>> server_goal_handle){
        std::thread(std::bind(&Go2NavServer::execute,this,_1),server_goal_handle).detach();
    }
    void execute(std::shared_ptr<ServerGoalHandle<go2_tutorial_inter::action::Nav>> server_goal_handle){
        // 
        auto feedback = std::make_shared<go2_tutorial_inter::action::Nav::Feedback>();
        auto result = std::make_shared<go2_tutorial_inter::action::Nav::Result>();
        // 1.生成连续反馈
        rclcpp::Rate rate(1.0);
        while (rclcpp::ok())
        {
            // 生成剩余距离 --- 获取当前坐标点和起点 --- 剩余距离 = 目标距离- 已行进距离
            auto dis_x = current_point.x - start_point.x;
            auto dis_y = current_point.y - start_point.y;
            auto dis = sqrt(pow(dis_x,2) + pow(dis_y,2));
            auto distance = server_goal_handle->get_goal()->goal - dis;
            // 发布剩余距离
            feedback->distance = distance;
            server_goal_handle->publish_feedback(feedback);
            // 循环退出条件
            // 任务被取消（客户端提交取消任务请求）
            if (server_goal_handle->is_canceling())
            {
                // 停止运动
                stop_move();
                // 响应结果
                result->point = current_point;
                server_goal_handle->canceled(result);
                return;
            }

            // 任务已完成（到达目标点附近）
            if (distance <= this->get_parameter("error").as_double())
            {
                RCLCPP_INFO(this->get_logger(),"已经到达目标点附近!");
                stop_move();
                break;
            }
            rate.sleep();
        }
        // 2.生成最终结果
        if (rclcpp::ok())
        {
            result->point = current_point;
            server_goal_handle->succeed(result);
        }

    }


};

int main(int argc, char ** argv)
{
    // 初始化 ROS 2
    rclcpp::init(argc, argv);
    // 创建节点并开始自旋（处理回调）
    auto node = std::make_shared<Go2NavServer>();
    rclcpp::spin(node);
    // 关闭 ROS 2
    rclcpp::shutdown();
    return 0;
}