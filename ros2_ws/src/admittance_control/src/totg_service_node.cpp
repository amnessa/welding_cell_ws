/**
 * @file totg_service_node.cpp
 * @brief ROS 2 service node for Time-Optimal Trajectory Generation (TOTG).
 *
 * Exposes a service that receives raw joint-space waypoints and returns a
 * fully time-parameterized trajectory using MoveIt 2's battle-tested TOTG
 * algorithm (Kunz & Stilman, 2012).
 *
 * Uses the low-level Path + Trajectory API directly — no robot model,
 * URDF, SRDF, or MoveIt planning scene required.
 */

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <cmath>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include "admittance_control/srv/compute_totg.hpp"

class TotgServiceNode : public rclcpp::Node
{
public:
  TotgServiceNode() : Node("totg_service_node")
  {
    service_ = this->create_service<admittance_control::srv::ComputeTOTG>(
        "compute_totg",
        std::bind(&TotgServiceNode::handle_request, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "TOTG service ready on 'compute_totg'");
  }

private:
  void handle_request(
      const std::shared_ptr<admittance_control::srv::ComputeTOTG::Request> request,
      std::shared_ptr<admittance_control::srv::ComputeTOTG::Response> response)
  {
    const uint32_t n_joints = request->num_joints;
    const size_t n_values = request->waypoints_flat.size();

    // ── Validate input ──────────────────────────────────────────────
    if (n_joints == 0 || n_values == 0 || n_values % n_joints != 0) {
      response->success = false;
      response->message = "Invalid input: waypoints_flat size not divisible by num_joints";
      return;
    }

    const size_t n_waypoints = n_values / n_joints;
    if (n_waypoints < 2) {
      response->success = false;
      response->message = "Need at least 2 waypoints";
      return;
    }

    if (request->max_velocity.size() != n_joints ||
        request->max_acceleration.size() != n_joints) {
      response->success = false;
      response->message = "max_velocity/max_acceleration size must equal num_joints";
      return;
    }

    // ── Parse waypoints into Eigen vectors ──────────────────────────
    std::vector<Eigen::VectorXd> waypoints(n_waypoints);
    for (size_t i = 0; i < n_waypoints; ++i) {
      waypoints[i].resize(n_joints);
      for (uint32_t j = 0; j < n_joints; ++j) {
        waypoints[i](j) = request->waypoints_flat[i * n_joints + j];
      }
    }

    Eigen::VectorXd max_vel(n_joints), max_acc(n_joints);
    for (uint32_t j = 0; j < n_joints; ++j) {
      max_vel(j) = request->max_velocity[j];
      max_acc(j) = request->max_acceleration[j];
    }

    double path_tolerance = request->path_tolerance;
    if (path_tolerance <= 0.0) path_tolerance = 0.02;

    double resample_dt = request->resample_dt;
    if (resample_dt <= 0.0) resample_dt = 0.1;

    RCLCPP_INFO(this->get_logger(),
                "TOTG request: %zu waypoints × %u joints, "
                "path_tol=%.3f, resample_dt=%.3f",
                n_waypoints, n_joints, path_tolerance, resample_dt);

    // ── 1b. Filter near-duplicate consecutive waypoints ────────────
    {
      std::vector<Eigen::VectorXd> filtered;
      filtered.push_back(waypoints[0]);
      for (size_t i = 1; i < waypoints.size(); ++i) {
        if ((waypoints[i] - filtered.back()).norm() > 1e-6) {
          filtered.push_back(waypoints[i]);
        }
      }
      if (filtered.size() < 2) {
        // All waypoints are effectively the same point
        response->timed_positions_flat.resize(n_joints);
        response->timed_velocities_flat.resize(n_joints, 0.0);
        response->timestamps = {0.0};
        for (uint32_t j = 0; j < n_joints; ++j) {
          response->timed_positions_flat[j] = waypoints[0](j);
        }
        response->num_output_points = 1;
        response->success = true;
        response->message = "Degenerate path (all same point)";
        return;
      }
      if (filtered.size() != waypoints.size()) {
        RCLCPP_INFO(this->get_logger(),
                    "Filtered %zu near-duplicate waypoints (%zu → %zu)",
                    waypoints.size() - filtered.size(),
                    waypoints.size(), filtered.size());
        waypoints = std::move(filtered);
      }
    }

    // ── 2. Build differentiable path with corner blending ────────────
    auto path_opt = trajectory_processing::Path::create(waypoints, path_tolerance);
    if (!path_opt) {
      response->success = false;
      response->message = "Path::create failed — waypoints may be degenerate";
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    // ── 3. Compute time-optimal trajectory ──────────────────────────
    constexpr double integration_step = 0.001;
    auto traj_opt = trajectory_processing::Trajectory::create(
        *path_opt, max_vel, max_acc, integration_step);
    if (!traj_opt) {
      response->success = false;
      response->message = "Trajectory::create failed — path may be infeasible "
                          "with given velocity/acceleration limits";
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    double duration = traj_opt->getDuration();
    RCLCPP_INFO(this->get_logger(),
                "TOTG computed: duration=%.3fs", duration);

    // Guard against NaN or non-positive duration
    if (!std::isfinite(duration) || duration <= 0.0) {
      response->success = false;
      response->message = "TOTG produced invalid duration (" +
                          std::to_string(duration) + "s) — path may be too short";
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    // ── 4. Resample at requested timestep ────────────────────────────
    size_t n_out = static_cast<size_t>(duration / resample_dt) + 1;
    // Always include start and end
    std::vector<double> sample_times;
    sample_times.reserve(n_out + 1);
    for (size_t i = 0; i < n_out; ++i) {
      double t = i * resample_dt;
      if (t > duration) t = duration;
      sample_times.push_back(t);
    }
    // Ensure final point is exactly at duration
    if (sample_times.empty() || std::abs(sample_times.back() - duration) > 1e-9) {
      sample_times.push_back(duration);
    }

    const size_t n_output = sample_times.size();
    response->timed_positions_flat.resize(n_output * n_joints);
    response->timed_velocities_flat.resize(n_output * n_joints);
    response->timestamps.resize(n_output);

    for (size_t i = 0; i < n_output; ++i) {
      double t = sample_times[i];
      Eigen::VectorXd pos = traj_opt->getPosition(t);
      Eigen::VectorXd vel = traj_opt->getVelocity(t);
      response->timestamps[i] = t;
      for (uint32_t j = 0; j < n_joints; ++j) {
        response->timed_positions_flat[i * n_joints + j] = pos(j);
        response->timed_velocities_flat[i * n_joints + j] = vel(j);
      }
    }

    response->num_output_points = static_cast<uint32_t>(n_output);
    response->success = true;
    response->message = "OK: " + std::to_string(n_waypoints) + " in → " +
                        std::to_string(n_output) + " out, " +
                        std::to_string(duration) + "s";

    RCLCPP_INFO(this->get_logger(),
                "TOTG response: %zu pts, %.3fs", n_output, duration);
  }

  rclcpp::Service<admittance_control::srv::ComputeTOTG>::SharedPtr service_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TotgServiceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
