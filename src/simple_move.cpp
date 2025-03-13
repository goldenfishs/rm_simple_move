#include "rm_simpal_move/simple_move.hpp"

namespace rm_simpal_move
{

    /**
     * @brief 获取欧拉角:
     * @param eulr
     * @param quat
     * @return
     */
    int8_t RMSimpleMove::AHRS_GetEulr(AHRS_Eulr_t *eulr, const geometry_msgs::msg::Quaternion &quat)
    {
        if (!eulr)
            return -1;

        const float sinr_cosp = 2.0f * (quat.w * quat.x + quat.y * quat.z);
        const float cosr_cosp = 1.0f - 2.0f * (quat.x * quat.x + quat.y * quat.y);
        eulr->pit = atan2f(sinr_cosp, cosr_cosp);

        const float sinp = 2.0f * (quat.w * quat.y - quat.z * quat.x);
        eulr->rol = fabsf(sinp) >= 1.0f ? copysignf(M_PI / 2.0f, sinp) : asinf(sinp);

        const float siny_cosp = 2.0f * (quat.w * quat.z + quat.x * quat.y);
        const float cosy_cosp = 1.0f - 2.0f * (quat.y * quat.y + quat.z * quat.z);
        eulr->yaw = atan2f(siny_cosp, cosy_cosp);

        return 0;
    }

    /**
     * @brief RMSimpleMove 构造函数
     */
    RMSimpleMove::RMSimpleMove(const rclcpp::NodeOptions &options)
        : Node("global_position_listener", options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_), running_(true), goal_reached_(false)
    {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&RMSimpleMove::timer_callback, this));
        goal_pose_sub_ = this->create_subscription<rm_msgs::msg::GoalPose>("/goal_pose", 10, std::bind(&RMSimpleMove::goal_pose_callback, this, std::placeholders::_1));
        data_ai_pub_ = this->create_publisher<rm_msgs::msg::DataAI>("/chassis/data_ai", 10);
        data_nav_pub_ = this->create_publisher<rm_msgs::msg::DataNav>("/chassis/data_nav", 10);
        RCLCPP_INFO(this->get_logger(), "RMSimpleMove 启动！！！");
    }

    /**
     * @brief RMSimpleMove 析构函数
     */
    RMSimpleMove::~RMSimpleMove()
    {
        running_.store(false);
        if (timer_thread_.joinable())
        {
            timer_thread_.join();
        }
    }

    /**
     * @brief 定时器回调函数
     */
    
    void RMSimpleMove::timer_callback()
    {
        /*获取当前base_link相对于map的坐标变换*/
        try
        {
            auto trans = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
            AHRS_GetEulr(&current_eulr_, trans.transform.rotation);
            RCLCPP_INFO(this->get_logger(), "pitch=%f, roll=%f, yaw=%f", current_eulr_.pit, current_eulr_.rol, current_eulr_.yaw);
    
            // 发布目标点的 TF 变换
            geometry_msgs::msg::TransformStamped goal_transform;
            goal_transform.header.stamp = this->get_clock()->now();
            goal_transform.header.frame_id = "map";
            goal_transform.child_frame_id = "goal_pose";
            goal_transform.transform.translation.x = goal_pose_.x;
            goal_transform.transform.translation.y = goal_pose_.y;
            goal_transform.transform.translation.z = 0.0;
            goal_transform.transform.rotation.x = 0.0;
            goal_transform.transform.rotation.y = 0.0;
            goal_transform.transform.rotation.z = 0.0;
            goal_transform.transform.rotation.w = 1.0;
    
            tf_broadcaster_->sendTransform(goal_transform);
    
            // 计算当前位置到goal_pose和base_link的坐标变换
            geometry_msgs::msg::TransformStamped goal_in_base_link;
            goal_in_base_link = tf_buffer_.lookupTransform("base_link", "goal_pose", tf2::TimePointZero);
    
            float goal_x_in_base_link = goal_in_base_link.transform.translation.x;
            float goal_y_in_base_link = goal_in_base_link.transform.translation.y;
            // 发布 DataNav 消息
            auto data_nav_msg = rm_msgs::msg::DataNav();
            if (is_goal_reached(goal_x_in_base_link, goal_y_in_base_link, goal_pose_.tolerance))
            {
                if (!goal_reached_)
                {
                    data_nav_msg.reached = true;
                    goal_reached_ = true;
                }
            }
            else
            {
                data_nav_msg.reached = false;
                goal_reached_ = false;
            }
            // data_nav_msg.x = goal_x_in_base_link;
            // data_nav_msg.y = goal_y_in_base_link;
            //发布当前位置
            data_nav_msg.x = trans.transform.translation.x;
            data_nav_msg.y = trans.transform.translation.y;
            data_nav_msg.yaw = current_eulr_.yaw;
            data_nav_pub_->publish(data_nav_msg);
            // 发布 DataAI 消息
            auto data_ai_msg = rm_msgs::msg::DataAI();
            data_ai_msg.vx = -calc_linear_velocity(goal_y_in_base_link, goal_pose_.max_speed);
            data_ai_msg.vy = calc_linear_velocity(goal_x_in_base_link, goal_pose_.max_speed);
            data_ai_msg.yaw = -calc_angular_velocity(current_eulr_.yaw, goal_pose_.angle);
            if (goal_pose_.rotor)
            {
                data_ai_msg.notice = 0b11000000;
            }
            else
            {
                data_ai_msg.notice = 0b00000000;
            }
            data_ai_pub_->publish(data_ai_msg);
    
            RCLCPP_INFO(this->get_logger(), "Goal pose relative to base_link: x=%f, y=%f", goal_x_in_base_link, goal_y_in_base_link);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform base_link to map: %s", ex.what());
        }
    }
    

    /**
     * @brief 目标点回调函数
     * @param msg
     */
    void RMSimpleMove::goal_pose_callback(const rm_msgs::msg::GoalPose::SharedPtr msg)
    {
        goal_pose_.x = msg->x;
        goal_pose_.y = msg->y;
        goal_pose_.angle = msg->angle;
        goal_pose_.max_speed = msg->max_speed;
        goal_pose_.tolerance = msg->tolerance;
        goal_pose_.rotor = msg->rotor;
        RCLCPP_INFO(this->get_logger(), "Received goal pose: x=%f, y=%f, angle=%f", msg->x, msg->y, msg->angle);
    }

    /**
     * @brief 简单的线性速度计算 优化：可以增加PID控制让速度更加合理
     * @param distance 
     * @param max_speed 
     * @return 
     */
    float RMSimpleMove::calc_linear_velocity(float distance, float max_speed)
    {
        float k_linear = 0.5f;
        float velocity = k_linear * distance;
        if (abs(velocity) > max_speed)
        {
            velocity = copysign(max_speed, velocity);
        }
        return velocity;
    }

    /**
     * @brief 简单的角速度计算
     * @param yaw 
     * @param target_yaw 
     * @return 
     */
    float RMSimpleMove::calc_angular_velocity(float yaw, float target_yaw)
    {
        // yaw范围是[-pi, pi]，运动方向取最优方向
        float angle_diff = target_yaw - yaw;
        if (angle_diff > M_PI)
        {
            angle_diff -= 2 * M_PI;
        }
        else if (angle_diff < -M_PI)
        {
            angle_diff += 2 * M_PI;
        }
        float k_angular = 0.001f;
        return k_angular * angle_diff;
    }

    /**
     * @brief 判断是否到达目标点
     * @param x 
     * @param y 
     * @param tolerance 
     * @return 
     */
    bool RMSimpleMove::is_goal_reached(float x, float y, float tolerance)
    {
        return sqrt(x * x + y * y) < tolerance;
    }
} // namespace rm_simpal_move