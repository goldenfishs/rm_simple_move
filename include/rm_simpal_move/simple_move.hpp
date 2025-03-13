#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rm_msgs/msg/goal_pose.hpp>
#include <rm_msgs/msg/data_ai.hpp>
#include <rm_msgs/msg/data_nav.hpp>
#include <atomic>
#include <thread>
#include <cmath>
#include <rclcpp_components/register_node_macro.hpp>

namespace rm_simpal_move {

struct AHRS_Eulr_t {
    float yaw;
    float rol;
    float pit;
};

struct Goal_t {
    float x;
    float y;
    float angle;
    float max_speed;
    float tolerance;
    bool rotor;
};

class RMSimpleMove : public rclcpp::Node {
public:
    RMSimpleMove(const rclcpp::NodeOptions &options);
    ~RMSimpleMove() override;

private:
    void timer_callback();
    void goal_pose_callback(const rm_msgs::msg::GoalPose::SharedPtr msg);
    int8_t AHRS_GetEulr(AHRS_Eulr_t *eulr, const geometry_msgs::msg::Quaternion &quat);
    float calc_linear_velocity(float distance, float max_speed);
    float calc_angular_velocity(float yaw, float target_yaw); 
    bool is_goal_reached(float x, float y, float tolerance);

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<rm_msgs::msg::GoalPose>::SharedPtr goal_pose_sub_;
    rclcpp::Publisher<rm_msgs::msg::DataAI>::SharedPtr data_ai_pub_;
    rclcpp::Publisher<rm_msgs::msg::DataNav>::SharedPtr data_nav_pub_;
    std::atomic<bool> running_;
    std::thread timer_thread_;
    std::atomic<bool> goal_reached_;
    AHRS_Eulr_t current_eulr_;
    Goal_t goal_pose_;
};

} // namespace rm_simpal_move

RCLCPP_COMPONENTS_REGISTER_NODE(rm_simpal_move::RMSimpleMove)