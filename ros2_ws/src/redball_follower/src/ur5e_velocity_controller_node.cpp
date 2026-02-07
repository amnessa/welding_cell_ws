#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <chrono>
#include <vector>
#include <cmath>
#include <algorithm>
#include <map>

using namespace std::chrono_literals;

// UR5e joint names for Isaac Sim
const std::vector<std::string> UR5E_JOINT_NAMES = {
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"
};

// UR5e IBVS End-Effector Controller with Isaac Sim integration
class UR5eEndEffectorControllerNode : public rclcpp::Node
{
public:
    UR5eEndEffectorControllerNode() : Node("ur5e_velocity_controller_node"), time_(0.0)
    {
        // Publisher for end-effector velocity commands (for external use/debugging)
        end_effector_velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/end_effector_velocity", 10);

        // Publisher for Isaac Sim joint commands (JointState type)
        joint_command_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/isaac_joint_commands", 10);

        // Subscribe to joint states from Isaac Sim
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/isaac_joint_states", 10, std::bind(&UR5eEndEffectorControllerNode::joint_state_callback, this, std::placeholders::_1));

        // Subscribe to target pixel coordinates from ball tracker
        target_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/target_pixel_coords", 10, std::bind(&UR5eEndEffectorControllerNode::target_callback, this, std::placeholders::_1));

        image_width_  = this->declare_parameter<int>("image_width", 1280);
        image_height_ = this->declare_parameter<int>("image_height", 720);
        fx_ = this->declare_parameter<double>("fx", 600.0);
        fy_ = this->declare_parameter<double>("fy", 600.0);
        depth_target_  = this->declare_parameter<double>("depth_target", 0.200);  // desired standoff distance
        k_pixel_ = this->declare_parameter<double>("k_pixel_gain", 1.8);
        k_depth_ = this->declare_parameter<double>("k_depth_gain", 1.2);
        timeout_sec_ = this->declare_parameter<double>("lost_timeout", 2.0);
        min_manipulability_ = this->declare_parameter<double>("min_manipulability", 0.02);
        w1_pixel_ = this->declare_parameter<double>("w1_pixel", 2.5);
        w3_depth_ = this->declare_parameter<double>("w3_depth", 1.5);

        control_timer_ = this->create_wall_timer(
            100ms, std::bind(&UR5eEndEffectorControllerNode::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "UR5e Velocity Controller started (Isaac Sim integration).");
    }

private:
    enum class Mode { SEARCHING, HOLDING, TRACKING };

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Store current joint positions for potential use in Jacobian calculation
        current_joint_state_ = *msg;
        have_joint_state_ = true;

        // Debug: log first reception
        static bool first_msg = true;
        if (first_msg)
        {
            RCLCPP_INFO(this->get_logger(), "First joint state received! Joints: %zu", msg->position.size());
            first_msg = false;
        }
    }

    void target_callback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        last_target_px_ = *msg;
        last_target_time_ = this->now();
        have_target_ = true;
    }

    void control_loop()
    {
        // Decide mode
        auto now = this->now();
        double time_since_target = have_target_ ? (now - last_target_time_).seconds() : 999.0;

        Mode mode;
        if (time_since_target <= timeout_sec_)
            mode = Mode::TRACKING;
        else if (time_since_target <= timeout_sec_ + 3.0)  // hold position 3s before searching
            mode = Mode::HOLDING;
        else
            mode = Mode::SEARCHING;

        // Debug: log joint state status
        static int loop_count = 0;
        if (++loop_count % 50 == 0)  // Every 5 seconds
        {
            const char* mode_str = (mode == Mode::TRACKING) ? "TRACKING" :
                                   (mode == Mode::HOLDING)  ? "HOLDING" : "SEARCHING";
            RCLCPP_INFO(this->get_logger(), "Control loop: have_joint_state=%d, mode=%s, last_target=%.1fs ago",
                have_joint_state_, mode_str, time_since_target);
        }

        geometry_msgs::msg::Twist twist;

        if (mode == Mode::SEARCHING)
        {
            generate_search_motion(twist);
        }
        else if (mode == Mode::TRACKING)
        {
            generate_tracking_twist(twist);
        }
        // HOLDING: twist stays zero (hold position)

        end_effector_velocity_pub_->publish(twist);

        // Publish joint commands
        if (have_joint_state_ && current_joint_state_.position.size() >= 6)
        {
            std::vector<double> cmd_positions = current_joint_state_.position;
            double dt = 0.1;  // 100ms control loop

            // Max joint delta per step (safety clamp) — 0.05 rad ≈ 2.9 deg per 100ms
            const double max_delta = 0.05;

            if (mode == Mode::SEARCHING)
            {
                // Search by sweeping wrist joints only (don't move base/elbow)
                cmd_positions[3] += 0.015 * std::sin(time_ * 0.3);  // wrist_1 (vertical sweep)
                cmd_positions[5] += 0.020 * std::sin(time_ * 0.5);  // wrist_3 (horizontal sweep)
            }
            else if (mode == Mode::TRACKING)
            {
                // Range-and-bearing control (more robust than raw pixel error)
                double u0 = image_width_ * 0.5;
                double v0 = image_height_ * 0.5;
                double eu = last_target_px_.x - u0;  // pixel error horizontal
                double ev = last_target_px_.y - v0;  // pixel error vertical

                // Convert pixel error to bearing (angular error in radians)
                double bearing_h = std::atan2(eu, fx_);  // horizontal bearing
                double bearing_v = std::atan2(ev, fy_);  // vertical bearing

                // Depth: only use if sensor gives valid reading
                double range = last_target_px_.z;
                bool have_depth = (range > 0.05);
                double range_error = have_depth ? (range - depth_target_) : 0.0;

                // --- Gains ---
                double bearing_gain = 0.50;  // bearing → joint delta (rad/rad)
                double range_gain   = 1.00;  // range error → elbow delta (rad/m)

                // Compute raw deltas
                double d_pan    =  bearing_gain * bearing_h * dt;
                double d_lift   = -bearing_gain * bearing_v * dt;
                double d_elbow  = have_depth ? (-range_gain * range_error * dt) : 0.0;
                double d_wrist1 = -bearing_gain * 0.3 * bearing_v * dt;

                // Clamp for safety
                d_pan    = std::clamp(d_pan,    -max_delta, max_delta);
                d_lift   = std::clamp(d_lift,   -max_delta, max_delta);
                d_elbow  = std::clamp(d_elbow,  -max_delta, max_delta);
                d_wrist1 = std::clamp(d_wrist1, -max_delta, max_delta);

                cmd_positions[0] += d_pan;
                cmd_positions[1] += d_lift;
                cmd_positions[2] += d_elbow;
                cmd_positions[3] += d_wrist1;

                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "TRACKING: R=%s(err=%+.3f) Bh=%.1fdeg Bv=%.1fdeg | pan=%+.5f lift=%+.5f elbow=%+.5f wrist1=%+.5f",
                    have_depth ? std::to_string(range).substr(0,5).c_str() : "N/A",
                    range_error,
                    bearing_h * 180.0 / M_PI, bearing_v * 180.0 / M_PI,
                    d_pan, d_lift, d_elbow, d_wrist1);
            }
            // HOLDING: cmd_positions stays at current → robot holds still

            publish_joint_command(cmd_positions);
        }
        else
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "No joint state received yet, cannot publish commands");
        }

        time_ += 0.1;
    }

    void generate_search_motion(geometry_msgs::msg::Twist & twist)
    {
        // Search uses only wrist joints (via cmd_positions above)
        // Twist stays zero — no Cartesian motion during search
        twist.linear.x = twist.linear.y = twist.linear.z = 0.0;
        twist.angular.x = twist.angular.y = twist.angular.z = 0.0;
    }

    void generate_tracking_twist(geometry_msgs::msg::Twist & twist)
    {
        // Image center
        double u0 = image_width_  * 0.5;
        double v0 = image_height_ * 0.5;

        double u = last_target_px_.x;
        double v = last_target_px_.y;
        double Z = (last_target_px_.z > 0.05) ? last_target_px_.z : 0.3;  // fallback for Twist only

        double eu = (u - u0);
        double ev = (v - v0);
        double depth_err = (Z - depth_target_);

        // Weighted IBVS style (w1, w3 just scale gains)
        double pixel_gain = k_pixel_ * w1_pixel_;
        double depth_gain = k_depth_ * w3_depth_;

        twist.linear.x = -pixel_gain * (eu / fx_) * Z;
        twist.linear.y = -pixel_gain * (ev / fy_) * Z;
        twist.linear.z =  depth_gain * depth_err;

        // No rotation yet
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = 0.0;

        // Manipulability scaling (optional external estimate fed later).
        // For now we rely on Jacobian node logging; if you wish you can add a subscription
        // to a manipulability topic, then scale here when too low.
    }

    // Publish joint position commands to Isaac Sim
    void publish_joint_command(const std::vector<double>& positions)
    {
        sensor_msgs::msg::JointState cmd;
        cmd.header.stamp = this->now();
        cmd.name = UR5E_JOINT_NAMES;
        cmd.position = positions;
        // Isaac Sim expects position commands; velocity/effort can be empty or zero
        cmd.velocity.resize(6, 0.0);
        cmd.effort.resize(6, 0.0);
        joint_command_pub_->publish(cmd);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr end_effector_velocity_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    geometry_msgs::msg::Point last_target_px_;
    sensor_msgs::msg::JointState current_joint_state_;
    rclcpp::Time last_target_time_;
    bool have_target_{false};
    bool have_joint_state_{false};

    int image_width_;
    int image_height_;
    double fx_, fy_;
    double depth_target_;
    double k_pixel_, k_depth_;
    double timeout_sec_;
    double min_manipulability_;

    double w1_pixel_{1.0}, w3_depth_{1.0};

    double time_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UR5eEndEffectorControllerNode>();
    RCLCPP_INFO(node->get_logger(), "UR5e Velocity Controller spinning...");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
