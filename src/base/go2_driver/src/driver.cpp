/*
  需求：
      1.发布里程计消息；
      2.广播里程计坐标变换；
      3.发布关节状态消息。
  分析1：
      1.了解里程计消息字段；
      2.从哪获取？机器人已经发布高层消息话题；
      3.实现，先订阅状态话题，然后转换成里程计消息，最后发布。
  分析2：
      1.需要发布机器人基坐标系与odom坐标系的相对关系；
      2.这些相对坐标系与里程计消息类似
      3.发布。
  分析3：
      1.了解关节状态信息；
      2.获取数据
      3.订阅低层状态话题，然后解析转换成关节消息，最后发布。

*/

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "unitree_go/msg/low_state.hpp"

using namespace std::placeholders;
class Driver : public rclcpp::Node
{
public:
    Driver() : Node("driver")
    {
        // 构造函数：初始化发布者、订阅者、服务等
        // 声明参数
        this->declare_parameter("odom_frame","odom");
        this->declare_parameter("base_frame","base");
        this->declare_parameter("publish_tf",true);

        // 获取参数
        odom_frame = this->get_parameter("odom_frame").as_string();
        base_frame = this->get_parameter("base_frame").as_string();
        publish_tf = this->get_parameter("publish_tf").as_bool();

        // 坐标变换广播器
        tf_bro_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);


        // RCLCPP_INFO(this->get_logger(), "Hello from my ROS 2 node!");

        RCLCPP_INFO(this->get_logger(), "Using frames: odom='%s', base='%s'", 
                odom_frame.c_str(), base_frame.c_str());

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom",10);
        mode_sub_ = this->create_subscription<unitree_go::msg::SportModeState>(
                    "/lf/sportmodestate",10,std::bind(&Driver::mode_cb,this,_1));
        //关节状态发布
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states",10);
        low_state_sub_ = this->create_subscription<unitree_go::msg::LowState>(
          "lf/lowstate",10,std::bind(&Driver::low_state_cb,this,_1)
        );
    }

private:
    // 发布关节信息
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    //订阅低层状态
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_state_sub_;
    //广播坐标变换
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_bro_;
    // 订阅go2状态
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr mode_sub_;
    // 发布里程计
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::string odom_frame,base_frame;
    bool publish_tf;
    // 订阅低层信息获取关节状态，组织消息并发布
    void low_state_cb(const unitree_go::msg::LowState::SharedPtr low_state){
      sensor_msgs::msg::JointState joint_state;

      //组织数据
      joint_state.header.stamp = this->now();
      // 描述关节名称
      joint_state.name = {
        "FL_hip_joint","FL_thigh_joint","FL_calf_joint",
        "FR_hip_joint","FR_thigh_joint","FR_calf_joint",
        "RL_hip_joint","RL_thigh_joint","RL_calf_joint",
        "RR_hip_joint","RR_thigh_joint","RR_calf_joint",
      };

      // 遍历低层状态信息的关节数据
      for (size_t i = 0; i < 12; i++)
      {
        auto motor = low_state->motor_state[i];// 获取某个关节信息
        // 获取旋转角度，添加进
        joint_state.position.push_back(motor.q);
      }

      joint_state_pub_->publish(joint_state);
    }

    // 订阅
    void mode_cb(const unitree_go::msg::SportModeState::SharedPtr mode){
      // 转换并发布里程计
      nav_msgs::msg::Odometry odom;
      //组织数据
      //头
      odom.header.stamp.sec = mode->stamp.sec;
      odom.header.stamp.nanosec = mode->stamp.nanosec;
      odom.header.frame_id = "odom_frame"; // 里程计坐标系
      odom.child_frame_id = "base_frame"; //机器人坐标系 
      //位姿
      odom.pose.pose.position.x = mode->position[0];
      odom.pose.pose.position.y = mode->position[1];
      odom.pose.pose.position.z = mode->position[2];

      odom.pose.pose.orientation.w = mode->imu_state.quaternion[0];
      odom.pose.pose.orientation.x = mode->imu_state.quaternion[1];
      odom.pose.pose.orientation.y = mode->imu_state.quaternion[2];
      odom.pose.pose.orientation.z = mode->imu_state.quaternion[3];
      //速度
      odom.twist.twist.linear.x = mode->velocity[0];
      odom.twist.twist.linear.y = mode->velocity[1];
      odom.twist.twist.linear.z = mode->velocity[2];

      odom.twist.twist.angular.z = mode->yaw_speed;

      odom_pub_->publish(odom);

      if (!publish_tf)
      {
        return;
      }

      // 发布坐标变换
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = this->now();
      transform.header.frame_id = odom_frame;
      transform.child_frame_id = base_frame;
      
      //设置偏移量
      transform.transform.translation.x = odom.pose.pose.position.x;
      transform.transform.translation.y = odom.pose.pose.position.y;
      transform.transform.translation.z = odom.pose.pose.position.z;

      //设置偏移角度
      transform.transform.rotation = odom.pose.pose.orientation;

      tf_bro_->sendTransform(transform);
    }

};

int main(int argc, char ** argv)
{
    // 初始化 ROS 2
    rclcpp::init(argc, argv);
    // 创建节点并开始自旋（处理回调）
    auto node = std::make_shared<Driver>();
    rclcpp::spin(node);
    // 关闭 ROS 2
    rclcpp::shutdown();
    return 0;
}