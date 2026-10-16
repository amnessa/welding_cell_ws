// aim_rate_relay: the camera aim's joint rates, streamed to the UR at its control rate.
//
// camera_aim_node.py computes joint rates at 125 Hz (Python: one step costs 2.5 ms).
// Sent straight to forward_velocity_controller, every new value is a velocity step the
// UR applies as a torque kick: at 50 Hz the wrist buzzed, one kick every 20 ms
// (notes/thesis_notes.md, 2026-10-16). This node publishes at 500 Hz, the UR's control
// rate, ramping linearly from what it was sending to each new input over one input
// period (measured), so the commanded velocity is continuous. Cost: one input period of
// delay (8 ms at 125 Hz).
//
// Safety of its own, whatever the Python node does:
//   * no input for `timeout_s` -> decelerate to zero at `stop_acc_rad_s2`, then go quiet.
//     A stream that ends on zeros (the aim node's own stop) goes quiet once it is there:
//     nothing more reaches the controller after the aim node has switched it off.
//     This also covers a hard kill of the aim node mid-turn, which otherwise left the
//     last rate on the velocity controller;
//   * every joint clamped to `max_speed_rad_s`, and its change per tick to
//     `max_acc_rad_s2` * dt;
//   * an input of the wrong size or with a non-finite value is dropped.
//
//   ros2 run admittance_control aim_rate_relay      (gestures.launch.py starts it)

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using Float64MultiArray = std_msgs::msg::Float64MultiArray;

class AimRateRelay : public rclcpp::Node {
 public:
  AimRateRelay() : Node("aim_rate_relay") {
    input_topic_ = declare_parameter<std::string>("input_topic", "/camera_aim/joint_rates");
    output_topic_ = declare_parameter<std::string>("output_topic", "/forward_velocity_controller/commands");
    rate_hz_ = declare_parameter<double>("rate_hz", 500.0);
    timeout_s_ = declare_parameter<double>("timeout_s", 0.1);
    max_speed_ = declare_parameter<double>("max_speed_rad_s", 0.6);
    max_acc_ = declare_parameter<double>("max_acc_rad_s2", 4.0);
    stop_acc_ = declare_parameter<double>("stop_acc_rad_s2", 2.0);
    n_ = static_cast<size_t>(declare_parameter<int>("n_joints", 6));

    out_.assign(n_, 0.0);
    start_.assign(n_, 0.0);
    goal_.assign(n_, 0.0);
    period_s_ = 1.0 / 125.0;

    pub_ = create_publisher<Float64MultiArray>(output_topic_, 10);
    sub_ = create_subscription<Float64MultiArray>(
        input_topic_, 10, [this](Float64MultiArray::ConstSharedPtr msg) { on_input(*msg); });
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate_hz_), [this] { tick(); });
    RCLCPP_INFO(get_logger(), "relaying %s -> %s at %.0f Hz (linear ramp over the input period; "
                "timeout %.2f s)", input_topic_.c_str(), output_topic_.c_str(), rate_hz_, timeout_s_);
  }

 private:
  double now_s() { return std::chrono::duration<double>(clock_.now().time_since_epoch()).count(); }

  void on_input(const Float64MultiArray &msg) {
    if (msg.data.size() != n_ ||
        !std::all_of(msg.data.begin(), msg.data.end(), [](double x) { return std::isfinite(x); })) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "dropped an input of size %zu (or non-finite)", msg.data.size());
      return;
    }
    const double t = now_s();
    const double gap = t - last_input_s_;
    // the input period, measured: only consecutive inputs count (not the gap after idle)
    if (active_ && gap > 0.0 && gap < 0.1) period_s_ = 0.8 * period_s_ + 0.2 * std::clamp(gap, 0.002, 0.05);
    start_ = out_;
    goal_ = msg.data;
    input_s_ = t;
    last_input_s_ = t;
    const bool moving = std::any_of(msg.data.begin(), msg.data.end(), [](double x) { return x != 0.0; });
    if (!active_ && moving && t - last_moving_s_ > 1.0) RCLCPP_INFO(get_logger(), "input: streaming");
    if (moving) last_moving_s_ = t;
    active_ = true;
    stopping_ = false;
  }

  void tick() {
    const double t = now_s();
    const double dt = last_tick_s_ > 0.0 ? std::clamp(t - last_tick_s_, 0.0005, 0.02) : 1.0 / rate_hz_;
    last_tick_s_ = t;
    if (!active_) return;                                  // idle: nothing to send

    if (!stopping_ && t - last_input_s_ > timeout_s_) {
      stopping_ = true;
      if (std::any_of(out_.begin(), out_.end(), [](double x) { return x != 0.0; }))
        RCLCPP_WARN(get_logger(), "input lost for %.2f s while moving: ramping to zero", timeout_s_);
    }
    std::vector<double> want(n_, 0.0);                     // stopping: zero
    if (!stopping_) {
      const double a = std::clamp((t - input_s_) / period_s_, 0.0, 1.0);
      for (size_t i = 0; i < n_; ++i) want[i] = start_[i] + a * (goal_[i] - start_[i]);
    }
    const double step = (stopping_ ? stop_acc_ : max_acc_) * dt;
    bool all_zero = true;
    for (size_t i = 0; i < n_; ++i) {
      out_[i] += std::clamp(want[i] - out_[i], -step, step);
      out_[i] = std::clamp(out_[i], -max_speed_, max_speed_);
      if (std::abs(out_[i]) < 1e-9) out_[i] = 0.0;
      all_zero = all_zero && out_[i] == 0.0;
    }
    Float64MultiArray msg;
    msg.data = out_;
    pub_->publish(msg);

    // the stream is over once at zero: ramped down after a timeout, or a zero goal reached
    const bool goal_zero = std::all_of(goal_.begin(), goal_.end(), [](double x) { return x == 0.0; });
    if (all_zero && (stopping_ || (goal_zero && t - input_s_ >= period_s_))) {
      active_ = false;
      stopping_ = false;
    }
  }

  std::string input_topic_, output_topic_;
  double rate_hz_, timeout_s_, max_speed_, max_acc_, stop_acc_;
  size_t n_;
  std::vector<double> out_, start_, goal_;
  double period_s_;
  double input_s_ = 0.0, last_input_s_ = -1e9, last_tick_s_ = -1.0, last_moving_s_ = -1e9;
  bool active_ = false;     // publishing: a stream, or its ramp to zero
  bool stopping_ = false;   // the input timed out: ramping to zero
  std::chrono::steady_clock clock_;
  rclcpp::Publisher<Float64MultiArray>::SharedPtr pub_;
  rclcpp::Subscription<Float64MultiArray>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AimRateRelay>());
  rclcpp::shutdown();
  return 0;
}
