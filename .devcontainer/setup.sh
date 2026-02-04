#!/bin/bash
set -e

echo "=========================================="
echo "Setting up Isaac Sim UR5e Digital Twin Environment"
echo "=========================================="

# ==========================================
# Install essential tools (git, etc.)
# ==========================================
echo "Installing essential tools..."
apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    git \
    git-lfs \
    curl \
    wget \
    vim \
    nano \
    htop \
    mesa-utils \
    net-tools \
    iputils-ping \
    python3-pip \
    build-essential \
    2>/dev/null || echo "Some packages may not be available, continuing..."

# Mark workspace as safe for git (fixes VS Code git detection in containers)
git config --global --add safe.directory /workspaces/welding_cell_ws

# ==========================================
# Isaac Sim Internal ROS 2 Setup
# ==========================================
# IMPORTANT: Isaac Sim 5.1.0 ONLY supports Python 3.11
# ROS 2 Jazzy on Ubuntu 24.04 uses Python 3.12 (INCOMPATIBLE with Isaac Sim)
#
# Solution: Use Isaac Sim's internal ROS 2 libraries (compiled with Python 3.11)
# - DO NOT source /opt/ros/jazzy/setup.bash before running Isaac Sim
# - Isaac Sim auto-loads internal Jazzy libs on Ubuntu 24.04
# - System ROS 2 Jazzy is ONLY for external nodes (separate terminal)
#
# Reference: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.html
# Reference: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_launch.html

ISAAC_SIM_PATH="/isaac-sim"

echo "Installing ROS 2 Jazzy (recommended for Ubuntu 24.04)..."

# Add ROS 2 GPG key and repository
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Jazzy
apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-jazzy-desktop \
    ros-jazzy-ros-base \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-geometry \
    ros-jazzy-moveit \
    2>/dev/null || echo "Some ROS 2 packages may not be available, continuing..."

# Initialize rosdep if not already done
if [ ! -d /etc/ros/rosdep ]; then
    rosdep init 2>/dev/null || true
fi
rosdep update 2>/dev/null || true

# Set ROS 2 environment variables
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# ==========================================
# Clone Isaac Sim ROS Workspaces
# ==========================================
WORKSPACE_DIR="/workspaces/welding_cell_ws"
ROS2_WS="${WORKSPACE_DIR}/ros2_ws"
ISAACSIM_ROS_WS="${WORKSPACE_DIR}/IsaacSim-ros_workspaces"

echo "Creating ROS2 workspace structure..."
mkdir -p ${ROS2_WS}/src

# Clone Isaac Sim ROS Workspaces if not exists
if [ ! -d "${ISAACSIM_ROS_WS}" ]; then
    echo "Cloning Isaac Sim ROS Workspaces..."
    cd ${WORKSPACE_DIR}
    git clone https://github.com/isaac-sim/IsaacSim-ros_workspaces.git
    cd ${ISAACSIM_ROS_WS}
    git submodule update --init --recursive
else
    echo "Isaac Sim ROS Workspaces already exists, updating..."
    cd ${ISAACSIM_ROS_WS}
    git pull || true
    git submodule update --init --recursive
fi

# ==========================================
# Build ROS 2 workspace with Python 3.11 (for Isaac Sim compatibility)
# ==========================================
# Since Isaac Sim uses Python 3.11, we use the dockerfile approach
# or build the workspace inside Isaac Sim's environment

echo "Setting up Isaac Sim ROS workspace..."
if [ -d "${ISAACSIM_ROS_WS}/humble_ws" ]; then
    echo "Isaac Sim humble_ws found"
fi

# ==========================================
# Add useful aliases and environment variables
# ==========================================
cat >> ~/.bashrc << 'EOF'

# ========== UR5e Digital Twin Environment ==========
export UR5E_ROBOT_IP=192.168.8.4
export ROS_DOMAIN_ID=0

# Isaac Sim environment
export ISAAC_SIM_PATH=/isaac-sim
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# ==========================================
# IMPORTANT: Python Version Compatibility
# ==========================================
# Isaac Sim 5.1.0 = Python 3.11 ONLY
# ROS 2 Jazzy (Ubuntu 24.04) = Python 3.12
#
# DO NOT source /opt/ros/jazzy/setup.bash before running Isaac Sim!
# Isaac Sim uses its internal ROS 2 Jazzy libraries (Python 3.11)
#
# Workflow:
# - Terminal for Isaac Sim: DO NOT source system ROS 2
# - Terminal for external ROS nodes: Source system ROS 2
# ==========================================

# Function to source ROS 2 for external nodes (NOT for Isaac Sim terminal)
source_ros2_external() {
    if [ -f /opt/ros/jazzy/setup.bash ]; then
        source /opt/ros/jazzy/setup.bash
        echo "Sourced system ROS 2 Jazzy (Python 3.12) - DO NOT run Isaac Sim in this terminal"
    fi
}

# Isaac Sim aliases (run WITHOUT sourcing system ROS 2)
alias isaac-sim='${ISAAC_SIM_PATH}/runapp.sh'
alias isaac-check='${ISAAC_SIM_PATH}/isaac-sim.compatibility_check.sh'
alias isaac-python='${ISAAC_SIM_PATH}/python.sh'

# Alias to prepare terminal for Isaac Sim (ensures no conflicting ROS sourced)
alias isaac-env='unset ROS_DISTRO AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH && echo "Terminal ready for Isaac Sim (internal ROS libs)"'

# Alias to prepare terminal for external ROS nodes
alias ros2-env='source_ros2_external'

# ROS2 workspace aliases (for external nodes only)
alias cb='cd /workspaces/welding_cell_ws/ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install'
alias cbs='cd /workspaces/welding_cell_ws/ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-select'
alias sw='source /opt/ros/jazzy/setup.bash && source /workspaces/welding_cell_ws/ros2_ws/install/setup.bash 2>/dev/null || echo "Workspace not built yet"'

# Isaac Sim ROS workspace (Python 3.11 compatible - can be used with Isaac Sim)
alias sw-isaac='source /workspaces/welding_cell_ws/IsaacSim-ros_workspaces/jazzy_ws/install/local_setup.bash 2>/dev/null || echo "Isaac ROS workspace not built yet"'

# UR5e connection test
alias ur5e-ping='ping -c 3 ${UR5E_ROBOT_IP}'

# Quick ROS 2 commands (will source ROS 2 first)
alias ros2-topics='source /opt/ros/jazzy/setup.bash && ros2 topic list'
alias ros2-nodes='source /opt/ros/jazzy/setup.bash && ros2 node list'

echo "=========================================="
echo "UR5e Digital Twin Environment Ready!"
echo "Robot IP: ${UR5E_ROBOT_IP}"
echo "Isaac Sim: ${ISAAC_SIM_PATH}"
echo "=========================================="
echo ""
echo "IMPORTANT: Python Version Compatibility"
echo "  Isaac Sim uses Python 3.11 (internal ROS libs)"
echo "  System ROS 2 Jazzy uses Python 3.12"
echo "  DO NOT mix them in the same terminal!"
echo ""
echo "Isaac Sim commands (fresh terminal, no ROS sourced):"
echo "  isaac-sim     - Start Isaac Sim GUI"
echo "  isaac-check   - Run compatibility check"
echo "  isaac-python  - Isaac Sim Python interpreter"
echo "  isaac-env     - Prepare terminal for Isaac Sim"
echo ""
echo "External ROS 2 commands (separate terminal):"
echo "  ros2-env      - Source system ROS 2 Jazzy"
echo "  ros2-topics   - List ROS 2 topics"
echo "  ros2-nodes    - List ROS 2 nodes"
echo "  cb            - Build ROS2 workspace"
echo "  sw            - Source ROS2 workspace"
echo "  ur5e-ping     - Test connection to robot"
echo "=========================================="
EOF

echo "=========================================="
echo "Setup complete!"
echo "=========================================="
echo ""
echo "ROS 2 Jazzy installed (for external nodes, Python 3.12)"
echo "Isaac Sim uses internal ROS 2 Jazzy libraries (Python 3.11)"
echo ""
echo "WORKFLOW:"
echo "  Terminal 1 (Isaac Sim): Run 'isaac-sim' directly (no ROS sourced)"
echo "  Terminal 2 (ROS nodes): Run 'ros2-env' first, then ROS commands"
echo ""
echo "For more info: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.html"
echo "=========================================="
