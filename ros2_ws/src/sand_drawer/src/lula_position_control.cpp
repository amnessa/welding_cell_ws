#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <array>
#include <cstdio>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

class LulaPositionControlNode : public rclcpp::Node
{
public:
  LulaPositionControlNode()
  : Node("lula_position_control_node")
  {
    plane_json_path_ = this->declare_parameter<std::string>(
      "plane_json_path",
      "/workspaces/welding_cell_ws/ros2_ws/src/sand_drawer/generated_planes/sand_drawer_plane.json");
    robot_prim_path_ = this->declare_parameter<std::string>(
      "robot_prim_path", "/World/Robot_Mount_Point");
    end_effector_frame_ = this->declare_parameter<std::string>("end_effector_frame", "tool0");
    script_path_ = this->declare_parameter<std::string>(
      "script_path",
      "/workspaces/welding_cell_ws/ros2_ws/src/sand_drawer/scripts/lula_complicated_trajectory.py");
    isaac_python_cmd_ = this->declare_parameter<std::string>("isaac_python_cmd", "isaac-python");
    execute_ = this->declare_parameter<bool>("execute", true);
    headless_ = this->declare_parameter<bool>("headless", true);
    verbose_subprocess_output_ = this->declare_parameter<bool>("verbose_subprocess_output", false);
    subprocess_tail_lines_ = this->declare_parameter<int>("subprocess_tail_lines", 30);

    trigger_service_ = this->create_service<std_srvs::srv::Trigger>(
      "~/generate_trajectory",
      std::bind(
        &LulaPositionControlNode::handle_generate_trajectory, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "lula_position_control_node ready. Service: ~/generate_trajectory");
  }

private:
  static std::string shell_escape(const std::string & value)
  {
    std::string escaped = "'";
    for (char c : value) {
      if (c == '\'') {
        escaped += "'\\''";
      } else {
        escaped.push_back(c);
      }
    }
    escaped += "'";
    return escaped;
  }

  int run_command_capture(const std::string & cmd, std::string & output)
  {
    output.clear();
    std::array<char, 512> buffer{};

    FILE * pipe = popen((cmd + " 2>&1").c_str(), "r");
    if (!pipe) {
      output = "Failed to run command via popen.";
      return -1;
    }

    while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) != nullptr) {
      output += buffer.data();
    }

    int rc = pclose(pipe);
    return rc;
  }

  static bool contains(const std::string & text, const std::string & token)
  {
    return text.find(token) != std::string::npos;
  }

  std::string tail_lines(const std::string & text, int lines) const
  {
    if (lines <= 0) {
      return text;
    }

    std::vector<size_t> newline_positions;
    newline_positions.reserve(256);
    for (size_t i = 0; i < text.size(); ++i) {
      if (text[i] == '\n') {
        newline_positions.push_back(i);
      }
    }

    if (static_cast<int>(newline_positions.size()) < lines) {
      return text;
    }

    size_t start = newline_positions[newline_positions.size() - static_cast<size_t>(lines)] + 1;
    if (start >= text.size()) {
      return text;
    }
    return text.substr(start);
  }

  void handle_generate_trajectory(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;

    std::ostringstream args;
    args << shell_escape(script_path_)
         << " --plane-json " << shell_escape(plane_json_path_)
         << " --end-effector-frame " << shell_escape(end_effector_frame_)
         << " --robot-prim-path " << shell_escape(robot_prim_path_);

    if (execute_) {
      args << " --execute";
    }
    if (headless_) {
      args << " --headless";
    }

    std::ostringstream cmd;
    cmd
      << "if command -v " << shell_escape(isaac_python_cmd_) << " >/dev/null 2>&1; then "
      << shell_escape(isaac_python_cmd_) << " " << args.str()
      << "; elif [ -x /isaac-sim/python.sh ]; then /isaac-sim/python.sh " << args.str()
      << "; elif [ -x /isaac-sim/python/bin/python3 ]; then /isaac-sim/python/bin/python3 " << args.str()
      << "; else echo 'No Isaac Python launcher found (tried: " << isaac_python_cmd_
      << ", /isaac-sim/python.sh, /isaac-sim/python/bin/python3)'; exit 127; fi";

    std::string output;
    const int rc = run_command_capture(cmd.str(), output);
    const bool logical_error =
      contains(output, "No trajectory could be computed") ||
      contains(output, "Error:") ||
      contains(output, "Traceback") ||
      contains(output, "Robot articulation not found/initialized");

    const std::string summarized_output =
      verbose_subprocess_output_ ? output : tail_lines(output, subprocess_tail_lines_);

    if (rc == 0 && !logical_error) {
      response->success = true;
      response->message = "Trajectory generation succeeded.\n" + summarized_output;
      RCLCPP_INFO(this->get_logger(), "Lula trajectory generation succeeded.");
    } else {
      response->success = false;
      response->message =
        "Trajectory generation failed (exit code: " + std::to_string(rc) + ").\n" + summarized_output;
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    }
  }

  std::string plane_json_path_;
  std::string robot_prim_path_;
  std::string end_effector_frame_;
  std::string script_path_;
  std::string isaac_python_cmd_;
  bool execute_{true};
  bool headless_{true};
  bool verbose_subprocess_output_{false};
  int subprocess_tail_lines_{30};

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LulaPositionControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
