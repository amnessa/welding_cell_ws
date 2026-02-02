#!/bin/bash
set -e

echo "=========================================="
echo "Setting up Isaac Sim UR5e Digital Twin Environment"
echo "=========================================="

# Source ROS2 Humble (comes with Isaac Sim)
if [ -f /opt/ros/humble/setup.bash ]; then
    echo "Sourcing ROS2 Humble..."
    source /opt/ros/humble/setup.bash
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi

# Install additional ROS2 packages for UR5e
echo "Installing additional ROS2 packages..."
apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-humble-ur-robot-driver \
    ros-humble-ur-description \
    ros-humble-ur-moveit-config \
    ros-humble-moveit \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-controller-manager \
    ros-humble-joint-state-broadcaster \
    ros-humble-joint-trajectory-controller \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gripper-controllers \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-rviz2 \
    ros-humble-tf2-tools \
    ros-humble-rqt* \
    python3-scipy \
    python3-pip \
    mesa-utils \
    net-tools \
    iputils-ping \
    2>/dev/null || echo "Some packages may not be available, continuing..."

# Create ROS2 workspace structure
WORKSPACE_DIR="/workspaces/welding_cell_ws"
ROS2_WS="${WORKSPACE_DIR}/ros2_ws"

echo "Creating ROS2 workspace structure..."
mkdir -p ${ROS2_WS}/src

# Add useful aliases and environment variables
cat >> ~/.bashrc << 'EOF'

# ========== UR5e Digital Twin Environment ==========
export UR5E_ROBOT_IP=192.168.8.4
export ROS_DOMAIN_ID=0

# Isaac Sim aliases
alias isaac-sim='/isaac-sim/runapp.sh'
alias isaac-check='/isaac-sim/isaac-sim.compatibility_check.sh'
alias isaac-python='/isaac-sim/python.sh'

# ROS2 workspace aliases
alias cb='cd /workspaces/welding_cell_ws/ros2_ws && colcon build --symlink-install'
alias cbs='cd /workspaces/welding_cell_ws/ros2_ws && colcon build --symlink-install --packages-select'
alias sw='source /workspaces/welding_cell_ws/ros2_ws/install/setup.bash'
alias swb='source /workspaces/welding_cell_ws/ros2_ws/install/setup.bash && source ~/.bashrc'

# Source ROS2 workspace if it exists
if [ -f /workspaces/welding_cell_ws/ros2_ws/install/setup.bash ]; then
    source /workspaces/welding_cell_ws/ros2_ws/install/setup.bash
fi

# UR5e connection test
alias ur5e-ping='ping -c 3 ${UR5E_ROBOT_IP}'

echo "=========================================="
echo "UR5e Digital Twin Environment Ready!"
echo "Robot IP: ${UR5E_ROBOT_IP}"
echo "=========================================="
echo "Quick commands:"
echo "  isaac-sim    - Start Isaac Sim GUI"
echo "  isaac-check  - Run compatibility check"
echo "  cb           - Build ROS2 workspace"
echo "  sw           - Source ROS2 workspace"
echo "  ur5e-ping    - Test connection to robot"
echo "=========================================="
EOF

echo "=========================================="
echo "Setup complete!"
echo "=========================================="
