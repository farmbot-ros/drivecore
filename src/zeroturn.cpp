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

        double previous_heading_error_;
        
    public:
        Navigator(): Node("controller",
            rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true)
        ) {
            name = this->get_parameter_or<std::string>("name", "path_server");
            topic_prefix_param = this->get_parameter_or<std::string>("topic_prefix", "/fb");
            max_linear_speed = this->get_parameter_or<float>("max_linear_speed", 0.5);
            max_angular_speed = this->get_parameter_or<float>("max_angular_speed", 0.5);

            this->action_server_ = rclcpp_action::create_server<TheAction>(this, topic_prefix_param + "/con/zeroturn",
                std::bind(&Navigator::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&Navigator::handle_cancel, this, std::placeholders::_1),
                std::bind(&Navigator::handle_accepted, this, std::placeholders::_1)
            );

            previous_heading_error_ = 0.0;
            
            //subscribers
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(topic_prefix_param +"/loc/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                current_pose_ = msg->pose.pose;
            });
        }
    
    private:
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

            // Ensure the pose is initialized
            if (std::isnan(current_pose_.position.x)) {
                RCLCPP_ERROR(this->get_logger(), "Current pose is not initialized. Aborting goal.");
                goal_handle->abort(std::make_shared<TheAction::Result>());
                return;
            }

            std::vector<geometry_msgs::msg::Point> segments = generateSegments(current_pose_.position, goal->segment.destination.pose.position);
            for (const geometry_msgs::msg::Point& element : segments) {
                target_pose_ = element;
                RCLCPP_INFO(this->get_logger(), "Going to: %f, %f, currently at: %f, %f", target_pose_.x, target_pose_.y, current_pose_.position.x, current_pose_.position.y);
                rclcpp::Rate loop_rate(10);
                const double goal_threshold = 0.2;
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
                    fill_feedback(feedback, nav_params[0], nav_params[1], nav_params[2]);

                    // Check if within the goal threshold
                    if (std::hypot(current_pose_.position.x - target_pose_.x, current_pose_.position.y - target_pose_.y) < goal_threshold) {
                        break;
                    }
                    
                    RCLCPP_INFO(this->get_logger(), "Pose: (%f, %f), Target: (%f, %f), Distance: %f", 
                        current_pose_.position.x, current_pose_.position.y, 
                        target_pose_.x, target_pose_.y, 
                        nav_params[2]
                    );
                    goal_handle->publish_feedback(feedback);
                    loop_rate.sleep();
                }
            }
            // Goal is done, send success message
            if (rclcpp::ok()) {
                fill_result(result);
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            }
        }

        void fill_feedback(TheAction::Feedback::SharedPtr feedback, double linear=0.0, double angular=0.0, double distance=0.0){
            geometry_msgs::msg::Twist twist;
            twist.linear.x = linear;
            twist.angular.z = angular;
            feedback->twist = twist;
            feedback->distance.data = distance;
        }

        void fill_result(TheAction::Result::SharedPtr result, int return_code=0){
            std_msgs::msg::Int8 return_code_msg;
            return_code_msg.data = return_code;
            result->return_code = return_code_msg;
        }

        std::array<double, 3> get_nav_params(double angle_max = 3.14, double velocity_max = 1.0, double velocity_scale = 0.2, bool zeroturn = true, double heading_gain = 1.5, double derivative_gain = 0.1) {
            // Calculate the difference in positions
            double dx = target_pose_.x - current_pose_.position.x;
            double dy = target_pose_.y - current_pose_.position.y;
            double distance = std::sqrt(dx * dx + dy * dy);
            // Handle the case when the robot is at the target position
            const double epsilon = 1e-6;
            if (distance < epsilon) {
                return {0.0, 0.0, distance};
            }
            // Calculate the desired velocity (for forward motion)
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
            double heading_error = std::atan2(std::sin(preheading - orientation), std::cos(preheading - orientation));
            // Calculate the derivative (rate of change) of heading error
            double heading_derivative = heading_error - previous_heading_error_;
            previous_heading_error_ = heading_error;  // Update for next call
            // PD Control for angular velocity
            double angular_velocity = (heading_gain * heading_error) + (derivative_gain * heading_derivative);
            // Handle "zeroturn" behavior
            if (zeroturn) {
                const double turn_threshold = 0.02;  // Reduced threshold for more aggressive turning
                if (std::abs(heading_error) > turn_threshold) {
                    // If the heading difference is significant, turn in place and avoid moving forward
                    return {0.0, std::clamp(angular_velocity, -angle_max, angle_max), distance};
                }
            }
            // Clamp the angular velocity and linear velocity
            angular_velocity = std::clamp(angular_velocity, -angle_max, angle_max);
            velocity = std::clamp(velocity, -velocity_max, velocity_max);  // Final clamping 
            return {velocity, angular_velocity, distance};
        }




        std::vector<geometry_msgs::msg::Point> generateSegments(const geometry_msgs::msg::Point &start, const geometry_msgs::msg::Point &end, int n=1) {
            std::vector<geometry_msgs::msg::Point> segments;
            if (n <= 1) {
                segments.push_back(end);
                return segments;
            }
            // Calculate the step size for each segment in both x and y directions
            double stepX = (end.x - start.x) / n;
            double stepY = (end.y - start.y) / n;
            double stepZ = (end.z - start.z) / n;  // In case you want 3D points
            // Generate each segment
            for (int i = 1; i < n; ++i) {
                geometry_msgs::msg::Point pt;
                pt.x = start.x + i * stepX;
                pt.y = start.y + i * stepY;
                pt.z = start.z + i * stepZ;
            }
            return segments;
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
