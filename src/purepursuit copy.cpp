#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <farmbot_interfaces/action/control.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <cmath>
#include <vector>
#include <array>
#include <chrono>
#include <thread>

using std::placeholders::_1;

struct Observation {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

Observation observation;
std::vector<std::pair<double, double>> path;
double vel = 0.0;

// Parameters
const double k = 0.1;   // look forward gain
const double Lfc = 2.0; // [m] look-ahead distance
const double Kp = 1.0;  // speed proportional gain
const double WB = 1.83; // [m] wheel base of the vehicle

class State {
public:
    State() {
        yaw = observation.yaw;
        v = 0.0;
        rear_x = observation.x;
        rear_y = observation.y;
    }

    double update(double a, double time_interval) {
        yaw = observation.yaw;
        v += a * time_interval;
        rear_x = observation.x;
        rear_y = observation.y;
        return v;
    }

    double calc_distance(double point_x, double point_y) {
        double dx = rear_x - point_x;
        double dy = rear_y - point_y;
        return std::hypot(dx, dy);
    }

public:
    double yaw;
    double v;
    double rear_x;
    double rear_y;
};

class TargetCourse {
public:
    TargetCourse(std::vector<double> cx, std::vector<double> cy)
        : cx(cx), cy(cy), old_nearest_point_index(-1) {}

    std::pair<int, double> search_target_index(State& state) {
        int ind;
        if (old_nearest_point_index == -1) {
            ind = search_nearest_point(state);
            old_nearest_point_index = ind;
        } else {
            ind = old_nearest_point_index;
            while (true) {
                double distance_next_index = state.calc_distance(cx[ind + 1], cy[ind + 1]);
                if (ind >= static_cast<int>(cx.size() - 2) ||
                    state.calc_distance(cx[ind], cy[ind]) < distance_next_index) {
                    break;
                }
                ++ind;
            }
            old_nearest_point_index = ind;
        }

        double Lf = k * state.v + Lfc;

        while (Lf > state.calc_distance(cx[ind], cy[ind])) {
            if (ind + 1 >= static_cast<int>(cx.size())) {
                break;
            }
            ++ind;
        }

        return {ind, Lf};
    }

private:
    int search_nearest_point(State& state) {
        std::vector<double> dx, dy;
        for (double x : cx) {
            dx.push_back(state.rear_x - x);
        }
        for (double y : cy) {
            dy.push_back(state.rear_y - y);
        }
        std::vector<double> d(cx.size());
        for (size_t i = 0; i < cx.size(); ++i) {
            d[i] = std::hypot(dx[i], dy[i]);
        }
        return std::min_element(d.begin(), d.end()) - d.begin();
    }

    std::vector<double> cx;
    std::vector<double> cy;
    int old_nearest_point_index;
};

class PurePursuit : public rclcpp::Node {
public:
    using TheAction = farmbot_interfaces::action::Control;
    using GoalHandle = rclcpp_action::ServerGoalHandle<TheAction>;

    PurePursuit() : Node("pure_pursuit") {
        action_server_ = rclcpp_action::create_server<TheAction>(
            this,
            "/pure_pursuit",
            std::bind(&PurePursuit::handle_goal, this, _1, _2),
            std::bind(&PurePursuit::handle_cancel, this, _1),
            std::bind(&PurePursuit::handle_accepted, this, _1));

        publisher_pos = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/forward_position_controller/commands", 10);
        publisher_vel = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/forward_velocity_controller/commands", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), std::bind(&PurePursuit::timer_callback, this));

        time_interval = 0.02;
        target_speed = 2.0;
        state = State();
        T = 1.1;
        L = 1.83;
        Rw = 0.3;
        target_creation = false;
    }

private:
    rclcpp_action::Server<TheAction>::SharedPtr action_server_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_pos;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_vel;
    rclcpp::TimerBase::SharedPtr timer_;

    State state;
    TargetCourse target_course{std::vector<double>(), std::vector<double>()};

    double time_interval;
    double target_speed;
    double T;
    double L;
    double Rw;
    bool target_creation;
    int target_ind;
    double wheel_vel[2];

    std::shared_ptr<GoalHandle> current_goal_handle_;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
                                            std::shared_ptr<const TheAction::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received a goal request");
        if (current_goal_handle_) {
            RCLCPP_WARN(this->get_logger(), "Goal already active, rejecting new goal");
            return rclcpp_action::GoalResponse::REJECT;
        }
        (void)uuid;
        path.clear();
        for (const auto &pose : goal->segment.path) {
            path.push_back({pose.position.x, pose.position.y});
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        current_goal_handle_ = nullptr;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
        current_goal_handle_ = goal_handle;
        std::thread{std::bind(&PurePursuit::execute, this, goal_handle)}.detach();
    }

    void execute(const std::shared_ptr<GoalHandle> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        auto feedback = std::make_shared<TheAction::Feedback>();
        auto result = std::make_shared<TheAction::Result>();

        while (rclcpp::ok() && !path.empty()) {
            if (goal_handle->is_canceling()) {
                result->return_code.data = 1;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }

            if (path.empty()) {
                result->return_code.data = 0;
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
                return;
            }

            // Continue with pursuit control
            double ai = proportional_control(target_speed, state.v);
            double alpha = pure_pursuit_steer_control(state, target_course, target_ind);
            double state_vel = state.update(ai, time_interval);

            // Update feedback
            feedback->twist.linear.x = state_vel;
            feedback->twist.angular.z = alpha;
            goal_handle->publish_feedback(feedback);

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    double pure_pursuit_steer_control(State &state, TargetCourse &trajectory, int &pind) {
        auto [ind, Lf] = trajectory.search_target_index(state);

        if (pind >= ind) {
            ind = pind;
        }

        double tx = trajectory.cx[ind];
        double ty = trajectory.cy[ind];

        double delta_x = state.rear_x - tx;
        double delta_y = state.rear_y - ty;

        double target_angle = -std::atan2(delta_y, delta_x);
        double alpha = target_angle - state.yaw;
        double delta = -alpha;

        std_msgs::msg::Float64MultiArray knuckle_pos_array;
        knuckle_pos_array.data.resize(2);
        knuckle_pos_array.data[0] = delta;
        knuckle_pos_array.data[1] = delta;
        publisher_pos->publish(knuckle_pos_array);

        return alpha;
    }

    double proportional_control(double target, double current) {
        return Kp * (target - current);
    }

    void timer_callback() {
        if (path.empty()) {
            return;
        }

        if (!target_creation) {
            std::vector<double> cx_list, cy_list;
            for (auto &p : path) {
                cx_list.push_back(p.first);
                cy_list.push_back(p.second);
            }
            target_course = TargetCourse(cx_list, cy_list);
            auto [target_ind, _] = target_course.search_target_index(state);
            this->target_ind = target_ind;
            target_creation = true;
        }

        double ai = proportional_control(target_speed, state.v);
        double alpha = pure_pursuit_steer_control(state, target_course, target_ind);
        double state_vel = state.update(ai, time_interval);

        if (target_ind < static_cast<int>(path.size()) - 2) {
            double omega = 2.0 * state_vel * std::sin(alpha) / Lfc;
            wheel_vel[0] = (state_vel - omega * T / 2) / Rw;
            wheel_vel[1] = (state_vel + omega * T / 2) / Rw;
        } else {
            wheel_vel[0] = 0.0;
            wheel_vel[1] = 0.0;
        }

        std_msgs::msg::Float64MultiArray wheel_vel_array;
        wheel_vel_array.data.resize(2);
        wheel_vel_array.data[0] = wheel_vel[0];
        wheel_vel_array.data[1] = wheel_vel[1];
        publisher_vel->publish(wheel_vel_array);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PurePursuit>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
