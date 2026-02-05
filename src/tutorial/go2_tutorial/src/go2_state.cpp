/*
    需求：订阅里程计消息，当机器人位移超过阈值，输出当前坐标
    实现流程：
        1.订阅里程计消息
        2.在订阅方的回调函数中，计算当前机器人位置与上一个记录点的距离，
        如果大于阈值就输出坐标并更新
        3.生成第一个记录点
        当第一次订阅里程计消息时，就为记录点赋值

*/

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::placeholders;
class Go2State : public rclcpp::Node
{
public:
    Go2State(): Node("go2_state")
    {
        // 构造函数：初始化发布者、订阅者、服务等
        RCLCPP_INFO(this->get_logger(), "Hello from my ROS 2 node!");
        last_x = last_y = 0.0;
        is_first = true;
        this->declare_parameter("distance",0.5);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom",10,std::bind(&Go2State::odom_cb,this,_1));
    }

private:
    // 成员变量（如发布者、订阅者）声明
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    double last_x,last_y;
    bool is_first;
    void odom_cb(const nav_msgs::msg::Odometry::SharedPtr odom){
        // 初始化第一个记录点
        // 获取当前坐标
        double x = odom->pose.pose.position.x;
        double y = odom->pose.pose.position.y;

        // 如果是第一次订阅到消息--赋值
        if (is_first)
        {
            last_x = x;
            last_y = y;
            is_first = false;
            RCLCPP_INFO(this->get_logger(),"起点坐标:(%.2f,%.2f)",last_x,last_y);
            return;
        }

        // 计算是否超过阈值
        double distance_x = x - last_x;
        double distance_y = y - last_y;

        // 计算距离
        double distance = sqrt(pow(distance_x,2) + pow(distance_y,2));

        if (distance > this->get_parameter("distance").as_double())
        {
            // 输出坐标
            RCLCPP_INFO(this->get_logger(),"坐标:(%.2f,%.2f)",x,y);
            last_x = x;
            last_y = y;
        }
    }
};

int main(int argc, char ** argv)
{
    // 初始化 ROS 2
    rclcpp::init(argc, argv);
    // 创建节点并开始自旋（处理回调）
    auto node = std::make_shared<Go2State>();
    rclcpp::spin(node);
    // 关闭 ROS 2
    rclcpp::shutdown();
    return 0;
}