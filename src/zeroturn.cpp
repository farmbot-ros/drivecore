#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/int8.hpp"
#include "farmbot_interfaces/action/control.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/trigger.hpp"
 
#include <iostream>
#include <thread>
#include <mutex>
#include <string>


float deg2rad(float deg) {
    return deg * M_PI / 180;
}

float rad2deg(float rad) {
    return rad * 180 / M_PI;
}


class Navigator : public rclcpp::Node {
    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        float max_linear_speed;
        float max_angular_speed;
        std::string name;
        std::string topic_prefix_param;

        farmbot_interfaces::msg::Segment segment;
        geometry_msgs::msg::Pose current_pose_;
        geometry_msgs::msg::Point target_pose_;
        //action_server
        using TheAction = farmbot_interfaces::action::Control;
        using GoalHandle = rclcpp_action::ServerGoalHandle<TheAction>;
        std::shared_ptr<GoalHandle> handeler_;
        rclcpp_action::Server<TheAction>::SharedPtr action_server_;
        
    public:
        Navigator(): Node("controller",
            rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true)
        ) {
            try {
                name = this->get_parameter("name").as_string(); 
                topic_prefix_param = this->get_parameter("topic_prefix").as_string();
            } catch (...) {
                name = "path_server";
                topic_prefix_param = "/fb";
            }

            try {
                max_linear_speed = this->get_parameter("max_linear_speed").as_double();
                max_angular_speed = this->get_parameter("max_angular_speed").as_double();
            } catch (...) {
                RCLCPP_WARN(this->get_logger(), "Parameters max_linear_speed and max_angular_speed not found, using default");
                max_linear_speed = 0.5;
                max_angular_speed = 0.5;
            }
            RCLCPP_INFO(this->get_logger(), "Max linear speed: %f, Max angular speed: %f", max_linear_speed, max_angular_speed);


            this->action_server_ = rclcpp_action::create_server<TheAction>(this, topic_prefix_param + "/nav/control",
                std::bind(&Navigator::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&Navigator::handle_cancel, this, std::placeholders::_1),
                std::bind(&Navigator::handle_accepted, this, std::placeholders::_1)
            );
            
            //subscribers
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(topic_prefix_param +"/loc/odom", 10, std::bind(&Navigator::odom_callback, this, std::placeholders::_1));
        }
    
    private:
        void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr& odom) {
            current_pose_ = odom->pose.pose;
        }

        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const TheAction::Goal> goal){
            goal->segment;
            RCLCPP_INFO(this->get_logger(), "Received goal request");
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle){
            RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle){
            if (handeler_ && handeler_->is_active()) {
                RCLCPP_INFO(this->get_logger(), "ABORTING PREVIOUS GOAL...");
                handeler_->abort(std::make_shared<TheAction::Result>());
            }
            handeler_ = goal_handle;
            // this needs to return quickly to avoid blocking the executor, so spin up a new thread
            std::thread{std::bind(&Navigator::execute, this, std::placeholders::_1), goal_handle}.detach();
        }

        void execute(const std::shared_ptr<GoalHandle> goal_handle){
            // RCLCPP_INFO(this->get_logger(), "Executing goal");
            const auto goal = goal_handle->get_goal();
            auto feedback = std::make_shared<TheAction::Feedback>();
            auto result = std::make_shared<TheAction::Result>();

            RCLCPP_INFO(this->get_logger(), "Going to: %f, %f, currently at: %f, %f", target_pose_.x, target_pose_.y, current_pose_.position.x, current_pose_.position.y);
            rclcpp::Rate loop_rate(10);
            while (rclcpp::ok()){
                if (goal_handle->is_canceling()) {
                    fill_result(result, 1);
                    goal_handle->canceled(result);
                    return;
                } else if (!goal_handle->is_active()){
                    fill_result(result, 1);
                    return;
                }
                std::array<double, 3> nav_params = get_nav_params(max_angular_speed, max_linear_speed);
                fill_feedback(feedback, nav_params[0], nav_params[1]);
                // RCLCPP_INFO(this->get_logger(), "Twist: %f, %f", twist.linear.x, twist.angular.z);
                fill_feedback(feedback);
                RCLCPP_INFO(this->get_logger(), "Pose: (%f, %f), Target: (%f, %f), Distance: %f", 
                    current_pose_.position.x, current_pose_.position.y, 
                    target_pose_.x, target_pose_.y, 
                    nav_params[2]
                );
                goal_handle->publish_feedback(feedback);
                loop_rate.sleep();
            }
            // Goal is done, send success message
            if (rclcpp::ok()) {
                fill_result(result);
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            }
        }

        void fill_feedback(TheAction::Feedback::SharedPtr feedback, double linear=0.0, double angular=0.0){
            geometry_msgs::msg::Twist twist;
            twist.linear.x = linear;
            twist.angular.z = angular;
            feedback->twist = twist;
        }

        void fill_result(TheAction::Result::SharedPtr result, int return_code=0){
            std_msgs::msg::Int8 return_code_msg;
            return_code_msg.data = return_code;
            result->return_code = return_code_msg;
        }

        std::array<double, 3> get_nav_params(double angle_max = 1.0,  double velocity_max = 1.0,  double velocity_scale = 0.2, bool zeroturn = true) {
            // Calculate the difference in positions
            double dx = target_pose_.x - current_pose_.position.x;
            double dy = target_pose_.y - current_pose_.position.y;
            double distance = std::sqrt(dx * dx + dy * dy);
            // Handle the case when the robot is at the target position
            const double epsilon = 1e-6;
            if (distance < epsilon) {
                return {0.0, 0.0, distance};
            }
            // Calculate the desired velocity (only for forward motion)
            double velocity = velocity_scale * distance;
            // Calculate the desired heading
            double preheading = std::atan2(dy, dx);
            // Convert current orientation from quaternion to yaw
            double qx = current_pose_.orientation.x;
            double qy = current_pose_.orientation.y;
            double qz = current_pose_.orientation.z;
            double qw = current_pose_.orientation.w;
            // Convert quaternion to Euler angles
            double siny_cosp = 2 * (qw * qz + qx * qy);
            double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
            double orientation = std::atan2(siny_cosp, cosy_cosp);
            // Log the desired and current headings
            RCLCPP_INFO(this->get_logger(), "Desired Heading: %f, Current Heading: %f", rad2deg(preheading), rad2deg(orientation));
            // Calculate the heading difference and normalize it
            double heading = std::atan2(std::sin(preheading - orientation), std::cos(preheading - orientation));
            // Handle "zeroturn" behavior
            if (zeroturn) {
                const double turn_threshold = 0.05;  // Threshold angle in radians
                if (std::abs(heading) > turn_threshold) {
                    // If the heading difference is significant, turn in place and avoid moving forward
                    return {0.0, std::clamp(heading, -angle_max, angle_max), distance};
                }
            }
            // Clamp the angular velocity and linear velocity
            double angular = std::clamp(heading, -angle_max, angle_max);
            velocity = std::clamp(velocity, -velocity_max, velocity_max);
            return {velocity, angular, distance};
        }

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<Navigator>();
    try {
        executor.add_node(node);
        executor.spin();
    } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), e.what());
    }
    rclcpp::shutdown();
    return 0;
}