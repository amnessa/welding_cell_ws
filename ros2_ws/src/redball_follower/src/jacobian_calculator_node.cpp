#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <Eigen/Dense>
#include <memory>
#include <limits>
#include <unordered_map>
#include <algorithm>  // added for std::clamp
#include <type_traits>

// MoveIt Includes
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>

class JacobianCalculatorNode : public rclcpp::Node
{
public:
    // Remove automatically_declare_parameters_from_overrides(true)
    JacobianCalculatorNode() : Node("jacobian_calculator_node")
    {
        // Constructor minimal; full setup in init()
    }

    void init()
    {
        // Safe declare helpers
        auto ensure_param_str = [this](const std::string & name, const std::string & def) {
            if (!this->has_parameter(name)) this->declare_parameter<std::string>(name, def);
        };
        auto ensure_param_double = [this](const std::string & name, double def) {
            if (!this->has_parameter(name)) this->declare_parameter<double>(name, def);
        };
        auto ensure_param_bool = [this](const std::string & name, bool def) {
            if (!this->has_parameter(name)) this->declare_parameter<bool>(name, def);
        };

        ensure_param_str("planning_group", "ur_manipulator");
        ensure_param_str("end_effector_link", "wrist_3_link");
        ensure_param_double("min_manipulability", 0.02);
        ensure_param_str("control_mode", "position");
        ensure_param_double("posture_gain", 0.4);
        ensure_param_bool("use_nullspace_posture", true);
        ensure_param_double("slowdown_mu_threshold", 0.04);
        ensure_param_double("damping_mu_reference", 0.05);
        ensure_param_double("w2_manipulability", 1.0);
        ensure_param_double("manipulability_gain", 0.4);
        ensure_param_str("joint_state_topic", "/joint_states");
        std::string joint_state_topic = this->get_parameter("joint_state_topic").as_string();

        planning_group_name_   = this->get_parameter("planning_group").as_string();
        end_effector_link_name_= this->get_parameter("end_effector_link").as_string();
        min_manipulability_    = this->get_parameter("min_manipulability").as_double();
        control_mode_          = this->get_parameter("control_mode").as_string();
        posture_gain_          = this->get_parameter("posture_gain").as_double();
        use_nullspace_posture_ = this->get_parameter("use_nullspace_posture").as_bool();
        slowdown_mu_threshold_ = this->get_parameter("slowdown_mu_threshold").as_double();
        damping_mu_ref_        = this->get_parameter("damping_mu_reference").as_double();
        w2_manip_              = this->get_parameter("w2_manipulability").as_double();
        manip_gain_            = this->get_parameter("manipulability_gain").as_double();

        RCLCPP_INFO(this->get_logger(), "Loading robot model...");
        robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(shared_from_this(), "robot_description");
        robot_model_ = robot_model_loader_->getModel();
        if (!robot_model_) {
            RCLCPP_FATAL(this->get_logger(), "Failed to load robot model (robot_description missing?)");
            rclcpp::shutdown();
            return;
        }
        robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
        robot_state_->setToDefaultValues();
        joint_model_group_ = robot_model_->getJointModelGroup(planning_group_name_);
        if (!joint_model_group_) {
            RCLCPP_FATAL(this->get_logger(), "Planning group '%s' not found.", planning_group_name_.c_str());
            rclcpp::shutdown();
            return;
        }

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            joint_state_topic, 10,
            std::bind(&JacobianCalculatorNode::joint_state_callback, this, std::placeholders::_1));

        end_effector_velocity_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/end_effector_velocity", 10,
            std::bind(&JacobianCalculatorNode::velocity_callback, this, std::placeholders::_1));

        joint_velocity_pub_   = this->create_publisher<std_msgs::msg::Float64MultiArray>("/joint_velocities", 10);
        // The ONLY publisher to /isaac_joint_commands (sensor_msgs/JointState)
        joint_state_cmd_pub_  = this->create_publisher<sensor_msgs::msg::JointState>("/isaac_joint_commands", 10);
        // Publish manipulability for monitoring the cost function
        manipulability_pub_   = this->create_publisher<std_msgs::msg::Float64>("/manipulability", 10);

        RCLCPP_INFO(this->get_logger(), "Jacobian calculator ready. control_mode=%s; joint_state_topic=%s; w2_manip=%.2f",
                    control_mode_.c_str(), joint_state_topic.c_str(), w2_manip_);
    }

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        last_joint_state_ = *msg;  // store for position integration
        if (robot_state_)
        {
            // Filter the incoming joint state message to only include joints known to our robot model.
            // This prevents errors if the topic contains joints for other hardware, like a gripper.
            std::vector<std::string> known_joint_names;
            std::vector<double> known_joint_positions;

            for (size_t i = 0; i < msg->name.size(); ++i)
            {
                // Check if the joint from the message exists in our model
                if (robot_state_->getRobotModel()->hasJointModel(msg->name[i]))
                {
                    known_joint_names.push_back(msg->name[i]);
                    known_joint_positions.push_back(msg->position[i]);
                }
            }

            if (!known_joint_names.empty())
            {
                robot_state_->setVariablePositions(known_joint_names, known_joint_positions);
            }
        }
    }

    void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (!robot_state_ || !joint_model_group_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Robot model not ready yet.");
            return;
        }

        if (last_joint_state_.name.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "No joint state yet to base command on.");
            return;
        }

        Eigen::MatrixXd jacobian = calculate_jacobian();
        if (jacobian.rows() != 6) { RCLCPP_WARN(this->get_logger(), "Unexpected Jacobian size %ldx%ld", jacobian.rows(), jacobian.cols()); return; }

        // Compute manipulability μ = sqrt(det(J*J^T))
        double mu = 0.0;
        if (jacobian.cols() >= 6) {
            Eigen::MatrixXd JJt = jacobian * jacobian.transpose();
            double detJJt = JJt.determinant();
            if (detJJt > 0.0) {
                mu = std::sqrt(detJJt);
                last_manipulability_ = mu;
            }
        }

        // Publish manipulability for monitoring
        std_msgs::msg::Float64 mu_msg;
        mu_msg.data = mu;
        manipulability_pub_->publish(mu_msg);

        // Log manipulability status periodically
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Manipulability: μ=%.4f (threshold=%.4f, damping_ref=%.4f) | Cost w2/μ=%.4f",
            mu, slowdown_mu_threshold_, damping_mu_ref_,
            (mu > 1e-8) ? (w2_manip_ / mu) : std::numeric_limits<double>::infinity());

        // Build damped pseudoinverse: λ grows as μ drops
        double lambda = 0.0;
        if (mu < damping_mu_ref_) {
            double r = std::clamp(mu / damping_mu_ref_, 0.0, 1.0);
            lambda = (1.0 - r) * 0.08; // tune base damping
        }
        Eigen::MatrixXd Jt = jacobian.transpose();
        Eigen::MatrixXd I6 = Eigen::MatrixXd::Identity(6,6);
        Eigen::MatrixXd pinv;
        if (lambda > 1e-8) {
            Eigen::MatrixXd JJt_damped = jacobian * Jt + (lambda * lambda) * I6;
            pinv = Jt * JJt_damped.ldlt().solve(Eigen::MatrixXd::Identity(6,6));
        } else {
            pinv = jacobian.completeOrthogonalDecomposition().pseudoInverse();
        }

        Eigen::Matrix<double,6,1> v;
        v << msg->linear.x, msg->linear.y, msg->linear.z,
             msg->angular.x, msg->angular.y, msg->angular.z;

        // Primary joint velocity
        Eigen::VectorXd qdot_primary = pinv * v;

        // Slowdown if manipulability very low
        if (mu > 1e-8 && mu < slowdown_mu_threshold_) {
            double scale = mu / slowdown_mu_threshold_; // (0,1)
            qdot_primary *= scale;
        }

        // Finite-difference gradient of manipulability (grad_mu)
        Eigen::VectorXd grad_mu;
        if (w2_manip_ > 1e-6 && mu > 1e-8) {
            grad_mu = compute_manipulability_gradient(1e-4); // step
        }

        Eigen::VectorXd qdot = qdot_primary;

        // Nullspace projector
        Eigen::MatrixXd N = Eigen::MatrixXd::Identity(qdot_primary.size(), qdot_primary.size()) - pinv * jacobian;

        // Manipulability ascent term (reduces cost w2*(1/mu))
        if (w2_manip_ > 1e-6 && mu > 1e-8 && grad_mu.size() == qdot.size()) {
            // Cost term derivative for w2*(1/mu) is -w2/ mu^2 * grad_mu
            Eigen::VectorXd qdot_mu = (manip_gain_ * w2_manip_ / (mu * mu)) * grad_mu;
            qdot += N * qdot_mu;
        }

        // Optional posture term
        if (use_nullspace_posture_) {
            const std::vector<std::string>& names = joint_model_group_->getVariableNames();
            Eigen::VectorXd q(names.size());
            Eigen::VectorXd q_mid(names.size());
            for (size_t i=0;i<names.size();++i) {
                // removed getVariableIndex (not available in this MoveIt version)
                q[i] = robot_state_->getVariablePosition(names[i]);
                const moveit::core::JointModel* jm = robot_state_->getJointModel(names[i]);
                const auto& bounds = jm->getVariableBounds(names[i]);
                double low = bounds.min_position_;
                double high = bounds.max_position_;
                q_mid[i] = (std::isfinite(low) && std::isfinite(high) && high>low) ? 0.5*(low+high) : q[i];
            }
            Eigen::VectorXd posture_err = q_mid - q;
            qdot += posture_gain_ * N * posture_err;
        }

        // Publish raw velocities (diagnostic / chaining)
        std_msgs::msg::Float64MultiArray vel_arr;
        vel_arr.layout.dim.emplace_back();
        vel_arr.layout.dim[0].label="joint_velocities";
        vel_arr.layout.dim[0].size = qdot.size();
        vel_arr.layout.dim[0].stride = 1;
        vel_arr.data.assign(qdot.data(), qdot.data()+qdot.size());
        joint_velocity_pub_->publish(vel_arr);

        // Build JointState command
        sensor_msgs::msg::JointState cmd;
        cmd.header.stamp = this->now();
        const auto & names = joint_model_group_->getVariableNames();
        cmd.name = names;

        if (control_mode_ == "velocity") {
            cmd.velocity.assign(qdot.data(), qdot.data()+qdot.size());
            for (Eigen::Index i=0; i<qdot.size(); ++i)
                cmd.position.push_back(std::numeric_limits<double>::quiet_NaN());
        } else {
            // position integration
            // map current positions
            std::vector<double> current;
            current.reserve(names.size());
            // create lookup
            std::unordered_map<std::string,double> last_pos_map;
            for (size_t i=0;i<last_joint_state_.name.size();++i)
                last_pos_map[last_joint_state_.name[i]] = last_joint_state_.position[i];

            for (auto & n : names) {
                auto it = last_pos_map.find(n);
                current.push_back(it!=last_pos_map.end()? it->second : 0.0);
            }

            // simple dt assumption (or compute from timestamp)
            rclcpp::Time now = this->now();
            double dt = (last_vel_time_.nanoseconds()>0) ?
                (now - last_vel_time_).seconds() : default_dt_;
            last_vel_time_ = now;

            std::vector<double> new_pos(current.size());
            for (size_t i=0;i<current.size();++i)
                new_pos[i] = current[i] + qdot[i]*dt;

            cmd.position = new_pos;
            cmd.velocity.assign(qdot.data(), qdot.data()+qdot.size()); // optional
        }

        joint_state_cmd_pub_->publish(cmd);
    }

    Eigen::MatrixXd calculate_jacobian()
    {
        Eigen::MatrixXd jacobian;
        // The reference point for the Jacobian is the origin of the end-effector link
        Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
        robot_state_->getJacobian(
            joint_model_group_,
            robot_state_->getLinkModel(end_effector_link_name_),
            reference_point_position,
            jacobian);
        return jacobian;
    }

    Eigen::VectorXd compute_manipulability_gradient(double h)
    {
        const auto & names = joint_model_group_->getVariableNames();
        Eigen::VectorXd grad(names.size());
        double mu0 = last_manipulability_;
        if (mu0 <= 0.0) { grad.setZero(); return grad; }
        moveit::core::RobotState backup = *robot_state_;
        for (size_t i=0;i<names.size(); ++i) {
            double q_orig = robot_state_->getVariablePosition(names[i]);
            robot_state_->setVariablePosition(names[i], q_orig + h);
            robot_state_->updateLinkTransforms();
            Eigen::MatrixXd Jp = calculate_jacobian();
            double mu_p = 0.0;
            if (Jp.rows()==6) {
                Eigen::MatrixXd JJt = Jp * Jp.transpose();
                double detJJt = JJt.determinant();
                if (detJJt > 0.0) mu_p = std::sqrt(detJJt);
            }
            robot_state_->setVariablePosition(names[i], q_orig - h);
            robot_state_->updateLinkTransforms();
            Eigen::MatrixXd Jm = calculate_jacobian();
            double mu_m = 0.0;
            if (Jm.rows()==6) {
                Eigen::MatrixXd JJt = Jm * Jm.transpose();
                double detJJt = JJt.determinant();
                if (detJJt > 0.0) mu_m = std::sqrt(detJJt);
            }
            grad[i] = (mu_p - mu_m) / (2.0 * h);
            robot_state_->setVariablePosition(names[i], q_orig);
        }
        *robot_state_ = backup;
        robot_state_->updateLinkTransforms();
        return grad;
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr end_effector_velocity_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_velocity_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr manipulability_pub_;
    double min_manipulability_{0.02};
    double last_manipulability_{0.0};

    // MoveIt Members
    std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
    moveit::core::RobotModelPtr robot_model_;
    moveit::core::RobotStatePtr robot_state_;
    const moveit::core::JointModelGroup* joint_model_group_;
    std::string planning_group_name_;
    std::string end_effector_link_name_;

    std::string control_mode_;
    sensor_msgs::msg::JointState last_joint_state_;
    rclcpp::Time last_vel_time_;
    double default_dt_{0.1};

    // New members for nullspace and damping
    double posture_gain_{0.4};
    bool use_nullspace_posture_{true};
    double slowdown_mu_threshold_{0.04};
    double damping_mu_ref_{0.05};
    double w2_manip_{1.0};
    double manip_gain_{0.4};
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JacobianCalculatorNode>();
    node->init(); // Call init() here
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
