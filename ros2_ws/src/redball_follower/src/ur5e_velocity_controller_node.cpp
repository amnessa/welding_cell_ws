#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

// UR5e IBVS End-Effector Controller
// Publishes Twist commands to /end_effector_velocity
// jacobian_calculator_node handles IK with:
//   - Damped pseudoinverse (adaptive λ based on manipulability)
//   - Nullspace manipulability gradient ascent (cost w2/μ)
//   - Nullspace posture optimization
// Cost function: J = w1*||bearing_error||² + w2*(1/μ) + w3*||range_error||²
class UR5eEndEffectorControllerNode : public rclcpp::Node
{
public:
    UR5eEndEffectorControllerNode() : Node("ur5e_velocity_controller_node"), time_(0.0)
    {
        // Publisher for end-effector velocity commands
        // → jacobian_calculator_node subscribes to this and computes proper Jacobian-based IK
        end_effector_velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/end_effector_velocity", 10);

        // Subscribe to target pixel coordinates from ball tracker
        target_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/target_pixel_coords", 10, std::bind(&UR5eEndEffectorControllerNode::target_callback, this, std::placeholders::_1));

        image_width_  = this->declare_parameter<int>("image_width", 1280);
        image_height_ = this->declare_parameter<int>("image_height", 720);
        fx_ = this->declare_parameter<double>("fx", 600.0);
        fy_ = this->declare_parameter<double>("fy", 600.0);
        depth_target_  = this->declare_parameter<double>("depth_target", 0.500);  // desired standoff distance (m)
        k_pixel_ = this->declare_parameter<double>("k_pixel_gain", 0.6);
        k_depth_ = this->declare_parameter<double>("k_depth_gain", 0.4);
        timeout_sec_ = this->declare_parameter<double>("lost_timeout", 2.0);
        min_manipulability_ = this->declare_parameter<double>("min_manipulability", 0.02);
        w1_pixel_ = this->declare_parameter<double>("w1_pixel", 1.0);
        w3_depth_ = this->declare_parameter<double>("w3_depth", 1.0);
        max_linear_vel_  = this->declare_parameter<double>("max_linear_vel", 0.15);   // m/s
        max_angular_vel_ = this->declare_parameter<double>("max_angular_vel", 0.30);  // rad/s

        control_timer_ = this->create_wall_timer(
            100ms, std::bind(&UR5eEndEffectorControllerNode::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "UR5e Velocity Controller started (Isaac Sim integration).");
    }

private:
    enum class Mode { SEARCHING, HOLDING, TRACKING };

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

        // Debug: log mode status
        static int loop_count = 0;
        if (++loop_count % 50 == 0)  // Every 5 seconds
        {
            const char* mode_str = (mode == Mode::TRACKING) ? "TRACKING" :
                                   (mode == Mode::HOLDING)  ? "HOLDING" : "SEARCHING";
            RCLCPP_INFO(this->get_logger(), "Control loop: mode=%s, last_target=%.1fs ago",
                mode_str, time_since_target);
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

        // Publish Twist command → jacobian_calculator_node handles IK with:
        //   - Damped pseudoinverse (adaptive λ based on manipulability)
        //   - Nullspace manipulability gradient ascent
        //   - Nullspace posture optimization
        // This properly implements the cost function:
        //   J = w1*||pixel_error||² + w2*(1/μ) + w3*||depth_error||²
        // where μ = sqrt(det(J*Jᵀ)) is manipulability
        end_effector_velocity_pub_->publish(twist);

        // Log bearing/range info for debugging
        if (mode == Mode::TRACKING)
        {
            double u0 = image_width_ * 0.5;
            double v0 = image_height_ * 0.5;
            double eu = last_target_px_.x - u0;
            double ev = last_target_px_.y - v0;
            double bearing_h = std::atan2(eu, fx_);
            double bearing_v = std::atan2(ev, fy_);
            double range = last_target_px_.z;
            bool have_depth = (range > 0.05);
            double range_error = have_depth ? (range - depth_target_) : 0.0;

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "TRACKING: R=%s(err=%+.3f) Bh=%.1fdeg Bv=%.1fdeg | Twist: vx=%.4f vy=%.4f vz=%.4f",
                have_depth ? std::to_string(range).substr(0,5).c_str() : "N/A",
                range_error,
                bearing_h * 180.0 / M_PI, bearing_v * 180.0 / M_PI,
                twist.linear.x, twist.linear.y, twist.linear.z);
        }

        time_ += 0.1;
    }

    void generate_search_motion(geometry_msgs::msg::Twist & twist)
    {
        // Generate a gentle search pattern using end-effector motion
        // The Jacobian node will convert this to joint velocities with manipulability optimization
        double search_amp = 0.02;  // meters amplitude
        double search_freq = 0.3;  // Hz

        // Slow sinusoidal motion in Y-Z plane (camera view plane)
        twist.linear.x = 0.0;
        twist.linear.y = search_amp * std::sin(time_ * 2.0 * M_PI * search_freq);
        twist.linear.z = search_amp * 0.5 * std::sin(time_ * 2.0 * M_PI * search_freq * 0.7);

        // Small angular motion to scan
        twist.angular.x = 0.0;
        twist.angular.y = 0.05 * std::sin(time_ * 2.0 * M_PI * search_freq * 0.5);
        twist.angular.z = 0.08 * std::sin(time_ * 2.0 * M_PI * search_freq * 0.3);
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

        // --- Velocity saturation ---
        // Clamp linear velocity magnitude to max_linear_vel_
        double lin_mag = std::sqrt(twist.linear.x * twist.linear.x +
                                   twist.linear.y * twist.linear.y +
                                   twist.linear.z * twist.linear.z);
        if (lin_mag > max_linear_vel_ && lin_mag > 1e-6) {
            double scale = max_linear_vel_ / lin_mag;
            twist.linear.x *= scale;
            twist.linear.y *= scale;
            twist.linear.z *= scale;
        }

        // Clamp angular velocity magnitude
        double ang_mag = std::sqrt(twist.angular.x * twist.angular.x +
                                   twist.angular.y * twist.angular.y +
                                   twist.angular.z * twist.angular.z);
        if (ang_mag > max_angular_vel_ && ang_mag > 1e-6) {
            double scale = max_angular_vel_ / ang_mag;
            twist.angular.x *= scale;
            twist.angular.y *= scale;
            twist.angular.z *= scale;
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr end_effector_velocity_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    geometry_msgs::msg::Point last_target_px_;
    rclcpp::Time last_target_time_;
    bool have_target_{false};

    int image_width_;
    int image_height_;
    double fx_, fy_;
    double depth_target_;
    double k_pixel_, k_depth_;
    double timeout_sec_;
    double min_manipulability_;

    double w1_pixel_{1.0}, w3_depth_{1.0};
    double max_linear_vel_{0.15};
    double max_angular_vel_{0.30};

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
