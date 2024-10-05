#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/int8.hpp"
#include "farmbot_interfaces/action/control.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <iostream>
#include <thread>
#include <mutex>
#include <cmath>
#include <vector>
#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Constants for predictive control and robot parameters
constexpr double WB = 1.8;  // Wheelbase
constexpr int NX = 4;       // State vector size (x, y, v, yaw)
constexpr int NU = 2;       // Control vector size (accel, steer)
constexpr int T = 1;        // Time horizon for MPC
constexpr double DT = 0.05; // Time step for control

class Navigator : public rclcpp::Node {
private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    geometry_msgs::msg::Pose current_pose_;
    geometry_msgs::msg::Point target_pose_;
    std::shared_ptr<rclcpp_action::ServerGoalHandle<farmbot_interfaces::action::Control>> goal_handle_;
    rclcpp_action::Server<farmbot_interfaces::action::Control>::SharedPtr action_server_;

    float max_linear_speed_;
    float max_angular_speed_;
    double previous_heading_error_;

public:
    Navigator() : Node("navigator"), previous_heading_error_(0.0) {
        // Declare parameters for tuning
        this->declare_parameter("max_linear_speed", 0.5);
        this->declare_parameter("max_angular_speed", 0.5);
        max_linear_speed_ = this->get_parameter("max_linear_speed").as_double();
        max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();

        // Initialize action server
        this->action_server_ = rclcpp_action::create_server<farmbot_interfaces::action::Control>(
            this, "navigate", std::bind(&Navigator::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&Navigator::handle_cancel, this, std::placeholders::_1),
            std::bind(&Navigator::handle_accepted, this, std::placeholders::_1)
        );

        // Subscribe to odometry for updating the current robot pose
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&Navigator::odom_callback, this, std::placeholders::_1));
    }

private:
    // Odometry callback for updating the robot pose
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pose_ = msg->pose.pose;
    }

    // Action goal handler
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const farmbot_interfaces::action::Control::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Goal received");
        target_pose_ = goal->segment.destination.pose.position;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // Action cancel handler
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<farmbot_interfaces::action::Control>> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // Action accepted handler (spins a new thread for execution)
    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<farmbot_interfaces::action::Control>> goal_handle) {
        goal_handle_ = goal_handle;
        std::thread{std::bind(&Navigator::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    // Execute the goal
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<farmbot_interfaces::action::Control>> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing goal...");
        auto result = std::make_shared<farmbot_interfaces::action::Control::Result>();

        rclcpp::Rate loop_rate(10);
        while (rclcpp::ok()) {
            // Check if goal is canceled
            if (goal_handle->is_canceling()) {
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled.");
                return;
            }

            // Get navigation parameters (linear and angular velocities)
            auto nav_params = get_nav_params();

            // Fill feedback (current velocities)
            auto feedback = std::make_shared<farmbot_interfaces::action::Control::Feedback>();
            feedback->twist.linear.x = nav_params[0];
            feedback->twist.angular.z = nav_params[1];
            goal_handle->publish_feedback(feedback);

            // Check if target is reached
            if (nav_params[2] < 0.2) {
                RCLCPP_INFO(this->get_logger(), "Target reached.");
                result->return_code.data = 0;
                goal_handle->succeed(result);
                return;
            }

            loop_rate.sleep();
        }
    }

    // Calculate navigation parameters: [linear_velocity, angular_velocity, distance_to_target]
    std::array<double, 3> get_nav_params() {
        double dx = target_pose_.x - current_pose_.position.x;
        double dy = target_pose_.y - current_pose_.position.y;
        double distance = sqrt(dx * dx + dy * dy);

        // If we are very close to the target, stop the robot
        if (distance < 1e-3) {
            return {0.0, 0.0, distance};
        }

        // Calculate heading error
        double target_heading = atan2(dy, dx);
        double current_heading = get_yaw_from_quaternion(current_pose_.orientation);
        double heading_error = atan2(sin(target_heading - current_heading), cos(target_heading - current_heading));

        // PD control for heading correction
        double angular_velocity = 1.5 * heading_error + 0.1 * (heading_error - previous_heading_error_);
        previous_heading_error_ = heading_error;

        // Cap angular velocity
        angular_velocity = std::clamp(angular_velocity, -max_angular_speed_, max_angular_speed_);

        // Linear velocity proportional to distance
        double linear_velocity = 0.2 * distance;
        linear_velocity = std::clamp(linear_velocity, -max_linear_speed_, max_linear_speed_);

        return {linear_velocity, angular_velocity, distance};
    }

    // Helper function to convert quaternion to yaw angle
    double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion &q) {
        double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        return atan2(siny_cosp, cosy_cosp);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;

    auto navigator_node = std::make_shared<Navigator>();
    executor.add_node(navigator_node);

    try {
        executor.spin();
    } catch (const std::exception &e) {
        RCLCPP_ERROR(navigator_node->get_logger(), e.what());
    }

    rclcpp::shutdown();
    return 0;
}
