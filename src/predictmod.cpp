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
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"

#include <iostream>
#include <thread>
#include <mutex>
#include <string>
#include <cmath>
#include <array>
#include <vector>
#include <chrono>
#include <memory>
#include <algorithm>
#include <limits>

#include <spdlog/spdlog.h>

using namespace std::chrono_literals;
namespace echo = spdlog;

double clamp(double v, double lo, double hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

double min(double a, double b) {
    return (a < b) ? a : b;
}

float deg2rad(float deg) {
    return deg * M_PI / 180.0f;
}

float rad2deg(float rad) {
    return rad * 180.0f / M_PI;
}

struct State {
    double x;
    double y;
    double yaw;
    double v;
};

struct Reference {
    double x_ref;
    double y_ref;
    double yaw_ref;
    double v_ref;
};

// A simple MPC class that tries to find optimal acceleration and steering to follow a given reference trajectory.
class ModelPredictiveController {
    public:
        ModelPredictiveController(size_t horizon, double dt)
        : horizon_(horizon), dt_(dt) {
            Q_ = {1.0, 1.0, 0.5, 0.1};
            R_ = {0.1, 0.1};
            dR_ = {0.01, 0.01};
            lf_ = 1.0;
            lr_ = 1.0;
            wheelbase_ = lf_ + lr_;
        }

        void setReference(const std::vector<Reference> &ref) {
            reference_traj_ = ref;
        }

        // Solve the MPC optimization problem by searching for best accel/steer in a discrete manner.
        // For a real application, this should be replaced by an actual optimization solver.
        std::pair<double,double> solve(const State &current_state) {
            if (reference_traj_.size() < horizon_) {
                return std::make_pair(0.0, 0.0);
            }

            double cost_best = std::numeric_limits<double>::infinity();
            std::pair<double,double> best_inputs(0.0,0.0);

            // Simple discrete search; in a real scenario, use an optimization method
            for (double steer_candidate = -0.5; steer_candidate <= 0.5; steer_candidate += 0.05) {
                for (double accel_candidate = -1.0; accel_candidate <= 1.0; accel_candidate += 0.1) {
                    double cost = simulateAndCost(current_state, steer_candidate, accel_candidate);
                    if (cost < cost_best) {
                        cost_best = cost;
                        best_inputs = std::make_pair(accel_candidate, steer_candidate);
                    }
                }
            }

            prev_accel_ = best_inputs.first;
            prev_steer_ = best_inputs.second;
            return best_inputs;
        }

    private:
        double simulateAndCost(const State &start_state, double steer, double accel) {
            State s = start_state;
            double cost = 0.0;
            for (size_t i = 0; i < horizon_; i++) {
                const Reference &r = reference_traj_[i];
                double dx = s.x - r.x_ref;
                double dy = s.y - r.y_ref;
                double dyaw = s.yaw - r.yaw_ref;
                double dv = s.v - r.v_ref;
                cost += Q_[0]*dx*dx + Q_[1]*dy*dy + Q_[2]*dyaw*dyaw + Q_[3]*dv*dv;
                if (i > 0) {
                    double du_steer = steer - prev_steer_;
                    double du_accel = accel - prev_accel_;
                    cost += dR_[0]*du_accel*du_accel + dR_[1]*du_steer*du_steer;
                }

                s = updateKinematics(s, accel, steer);
            }
            return cost;
        }

        State updateKinematics(const State &s, double a, double delta) {
            State ns = s;
            ns.x += s.v * std::cos(s.yaw) * dt_;
            ns.y += s.v * std::sin(s.yaw) * dt_;
            ns.yaw += (s.v / wheelbase_) * std::tan(delta) * dt_;
            ns.v += a * dt_;
            return ns;
        }

        size_t horizon_;
        double dt_;
        double lf_;
        double lr_;
        double wheelbase_;
        std::vector<Reference> reference_traj_;
        std::vector<double> Q_;
        std::vector<double> R_;
        std::vector<double> dR_;
        double prev_steer_ = 0.0;
        double prev_accel_ = 0.0;
    };


    class Navigator : public rclcpp::Node {
    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        float max_linear_speed;
        float max_angular_speed;
        std::string name;
        double preheading;
        double orientation;

        farmbot_interfaces::msg::Segment segment;
        geometry_msgs::msg::Pose current_pose_;
        geometry_msgs::msg::Point target_pose_;

        using TheAction = farmbot_interfaces::action::Control;
        using GoalHandle = rclcpp_action::ServerGoalHandle<TheAction>;
        std::shared_ptr<GoalHandle> handeler_;
        rclcpp_action::Server<TheAction>::SharedPtr action_server_;

        double previous_heading_error_;

        diagnostic_updater::Updater updater_;
        diagnostic_msgs::msg::DiagnosticStatus status;
        rclcpp::TimerBase::SharedPtr diagnostic_timer_;

        // MPC related members
        std::shared_ptr<ModelPredictiveController> mpc_;
        size_t horizon_;
        double dt_;

    public:
        Navigator(): Node("predictmod",
            rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true)
        ), updater_(this) {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            status.message = "Not initialized";
            name = this->get_parameter_or<std::string>("name", "path_server");
            max_linear_speed = this->get_parameter_or<float>("max_linear_speed", 0.5);
            max_angular_speed = this->get_parameter_or<float>("max_angular_speed", 0.1);
            previous_heading_error_ = 0.0;

            // Setup MPC parameters
            horizon_ = 10;
            dt_ = 0.1;
            mpc_ = std::make_shared<ModelPredictiveController>(horizon_, dt_);

            this->action_server_ = rclcpp_action::create_server<TheAction>(this, "con/predictmod",
                std::bind(&Navigator::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&Navigator::handle_cancel, this, std::placeholders::_1),
                std::bind(&Navigator::handle_accepted, this, std::placeholders::_1)
            );

            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("loc/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                current_pose_ = msg->pose.pose;
            });

            updater_.setHardwareID(static_cast<std::string>(this->get_namespace()) + "/con");
            updater_.add("Controller Status", this, &Navigator::check_system);
            diagnostic_timer_ = this->create_wall_timer(1s, std::bind(&Navigator::diagnostic_callback, this));
        }

    private:
        void diagnostic_callback() {
            updater_.force_update();
        }

        void check_system(diagnostic_updater::DiagnosticStatusWrapper &stat) {
            stat.summary(status.level, status.message);
            stat.add("Current Pose", "x: " + std::to_string(current_pose_.position.x) + " y: " + std::to_string(current_pose_.position.y));
            stat.add("Target Pose", "x: " + std::to_string(target_pose_.x) + " y: " + std::to_string(target_pose_.y));
            stat.add("Current Heading", std::to_string(rad2deg(orientation)));
            stat.add("Target Heading", std::to_string(rad2deg(preheading)));
        }

        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const TheAction::Goal> goal){
            (void)uuid;
            goal->segment;
            RCLCPP_INFO(this->get_logger(), "Received goal request");
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
            std::thread{std::bind(&Navigator::execute, this, std::placeholders::_1), goal_handle}.detach();
        }

        void execute(const std::shared_ptr<GoalHandle> goal_handle){
            const auto goal = goal_handle->get_goal();
            auto feedback = std::make_shared<TheAction::Feedback>();
            auto result = std::make_shared<TheAction::Result>();

            if (std::isnan(current_pose_.position.x)) {
                RCLCPP_ERROR(this->get_logger(), "Current pose is not initialized. Aborting goal.");
                goal_handle->abort(std::make_shared<TheAction::Result>());
                return;
            }

            std::vector<geometry_msgs::msg::Point> segments = generateSegments(current_pose_.position, goal->segment.destination.pose.position);
            for (const geometry_msgs::msg::Point& element : segments) {
                target_pose_ = element;
                rclcpp::Rate loop_rate(10);
                const double goal_threshold = 0.2;
                while (rclcpp::ok()){
                    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
                    status.message = "Moving to goal";
                    if (goal_handle->is_canceling()) {
                        fill_result(result, 1);
                        goal_handle->canceled(result);
                        return;
                    } else if (!goal_handle->is_active()){
                        fill_result(result, 1);
                        return;
                    }

                    std::array<double, 3> nav_params = run_mpc_control();
                    // RCLCPP_INFO(this->get_logger(), "Linear: %f, Angular: %f, Distance: %f", nav_params[0], nav_params[1], nav_params[2]);
                    fill_feedback(feedback, nav_params[0], nav_params[1], nav_params[2]);

                    if (std::hypot(current_pose_.position.x - target_pose_.x, current_pose_.position.y - target_pose_.y) < goal_threshold) {
                        break;
                    }

                    goal_handle->publish_feedback(feedback);
                    loop_rate.sleep();
                }
            }

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

        std::array<double, 3> run_mpc_control() {
            double dx = target_pose_.x - current_pose_.position.x;
            double dy = target_pose_.y - current_pose_.position.y;
            double distance = std::sqrt(dx * dx + dy * dy);
            if (distance < 1e-6) {
                return {0.0, 0.0, distance};
            }

            // Calculate desired heading
            preheading = std::atan2(dy, dx);
            echo::info("Preheading: {}", preheading);

            // Convert quaternion to yaw
            double qx = current_pose_.orientation.x;
            double qy = current_pose_.orientation.y;
            double qz = current_pose_.orientation.z;
            double qw = current_pose_.orientation.w;
            double siny_cosp = 2 * (qw * qz + qx * qy);
            double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
            orientation = std::atan2(siny_cosp, cosy_cosp);
            echo::info("Orientation: {}", orientation);

            // Current state
            State current_state;
            current_state.x = current_pose_.position.x;
            current_state.y = current_pose_.position.y;
            current_state.yaw = orientation;
            // Estimate current velocity from linear.x (not given here, assume 0 or small)
            // A better way would be to read robot's odom twist, here we assume near zero
            // If needed, subscribe to odometry twist to update current velocity
            double current_v = 0.0;
            current_state.v = current_v;

            // Generate a straight line reference trajectory towards target
            std::vector<Reference> ref_traj;
            double v_ref = min(max_linear_speed, 0.5 * distance); // scale speed with distance
            for (size_t i = 0; i < horizon_; i++) {
                double step = i * dt_ * v_ref;
                double x_ref = current_state.x + step * std::cos(preheading);
                double y_ref = current_state.y + step * std::sin(preheading);
                double yaw_ref = preheading;
                ref_traj.push_back({x_ref, y_ref, yaw_ref, v_ref});
            }

            mpc_->setReference(ref_traj);
            auto [accel, steer] = mpc_->solve(current_state);

            // From acceleration and steering angle, compute forward velocity and angular velocity to send as commands
            // We'll treat acceleration * dt + current_v as the commanded linear velocity
            double commanded_v = std::clamp(current_v + accel * dt_, 0.0, (double)max_linear_speed);

            // The angular velocity approximately relates to steering angle:
            // angular velocity ~ (v / wheelbase) * tan(steer), assume wheelbase ~ 2.0
            // We can just convert steer to a rough angular rate command for demonstration.
            double wheelbase = 0.1;
            double angular_velocity = (commanded_v / wheelbase) * std::tan(steer);
            angular_velocity = clamp(angular_velocity, -max_angular_speed, max_angular_speed);

            return {commanded_v, angular_velocity, distance};
        }

        std::vector<geometry_msgs::msg::Point> generateSegments(const geometry_msgs::msg::Point &start, const geometry_msgs::msg::Point &end, int n=1) {
            std::vector<geometry_msgs::msg::Point> segments;
            if (n <= 1) {
                segments.push_back(end);
                return segments;
            }
            double stepX = (end.x - start.x) / n;
            double stepY = (end.y - start.y) / n;
            double stepZ = (end.z - start.z) / n;
            for (int i = 1; i < n; ++i) {
                geometry_msgs::msg::Point pt;
                pt.x = start.x + i * stepX;
                pt.y = start.y + i * stepY;
                pt.z = start.z + i * stepZ;
                segments.push_back(pt);
            }
            segments.push_back(end);
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
