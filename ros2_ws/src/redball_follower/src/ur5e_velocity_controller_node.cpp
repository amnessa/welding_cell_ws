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
        depth_default_ = this->declare_parameter<double>("depth_default", 0.8);
        depth_target_  = this->declare_parameter<double>("depth_target", 0.8);
        k_pixel_ = this->declare_parameter<double>("k_pixel_gain", 1.8);
        k_depth_ = this->declare_parameter<double>("k_depth_gain", 1.2);
        timeout_sec_ = this->declare_parameter<double>("lost_timeout", 0.5);
        min_manipulability_ = this->declare_parameter<double>("min_manipulability", 0.02);
        w1_pixel_ = this->declare_parameter<double>("w1_pixel", 2.5);
        w3_depth_ = this->declare_parameter<double>("w3_depth", 1.5);

        control_timer_ = this->create_wall_timer(
            100ms, std::bind(&UR5eEndEffectorControllerNode::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "UR5e Velocity Controller started (Isaac Sim integration).");
    }

private:
    enum class Mode { SEARCHING, TRACKING };

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
        Mode mode = Mode::SEARCHING;
        if (have_target_ && (now - last_target_time_).seconds() <= timeout_sec_)
            mode = Mode::TRACKING;

        // Debug: log joint state status
        static int loop_count = 0;
        if (++loop_count % 50 == 0)  // Every 5 seconds
        {
            RCLCPP_INFO(this->get_logger(), "Control loop: have_joint_state=%d, mode=%s",
                have_joint_state_, mode == Mode::SEARCHING ? "SEARCHING" : "TRACKING");
        }

        geometry_msgs::msg::Twist twist;

        if (mode == Mode::SEARCHING)
        {
            generate_search_motion(twist);
        }
        else
        {
            generate_tracking_twist(twist);
        }

        end_effector_velocity_pub_->publish(twist);

        // For now, publish simple joint commands for testing
        // TODO: Implement proper Jacobian-based IK for Twist → joint velocities
        if (have_joint_state_ && current_joint_state_.position.size() >= 6)
        {
            std::vector<double> cmd_positions = current_joint_state_.position;
            double dt = 0.1;  // 100ms control loop

            if (mode == Mode::SEARCHING)
            {
                // Small search motion on wrist joints
                cmd_positions[5] += 0.02 * std::sin(time_ * 0.5);  // wrist_3
            }
            else  // TRACKING mode
            {
                // Simple proportional control based on pixel error
                // Map pixel error to joint velocity (simplified - not true IK)
                double u0 = image_width_ * 0.5;
                double v0 = image_height_ * 0.5;
                double eu = last_target_px_.x - u0;  // horizontal error
                double ev = last_target_px_.y - v0;  // vertical error

                // Scale pixel error to joint velocity (rough approximation)
                // FLIPPED SIGNS: positive eu (ball to right) -> rotate base right (positive)
                // positive ev (ball below center) -> tilt up (negative shoulder_lift)
                double gain = 0.0001;  // Small gain for safety

                cmd_positions[0] += gain * eu * dt;   // shoulder_pan (left-right) - FLIPPED
                cmd_positions[1] += -gain * ev * dt;  // shoulder_lift (up-down) - FLIPPED

                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "TRACKING: eu=%.1f, ev=%.1f, pan_delta=%.5f, lift_delta=%.5f",
                    eu, ev, gain * eu * dt, -gain * ev * dt);
            }

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
        double radius = 0.05;
        double speed = 0.4;
        twist.linear.x = radius * -std::sin(time_ * speed);
        twist.linear.y = radius *  std::cos(time_ * speed);
        twist.linear.z = 0.0;
        twist.angular.x = twist.angular.y = twist.angular.z = 0.0;
    }

    void generate_tracking_twist(geometry_msgs::msg::Twist & twist)
    {
        // Image center
        double u0 = image_width_  * 0.5;
        double v0 = image_height_ * 0.5;

        double u = last_target_px_.x;
        double v = last_target_px_.y;
        double Z = (last_target_px_.z > 0.05) ? last_target_px_.z : depth_default_;

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
    double depth_default_, depth_target_;
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
