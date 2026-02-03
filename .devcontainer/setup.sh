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
# Isaac Sim 5.1.0 on Ubuntu 24.04 uses internal ROS 2 Jazzy libraries (Python 3.12)
# We install system ROS 2 Jazzy for the ros2 CLI tools and external nodes
# Reference: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.html

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

# Source ROS 2 Jazzy (system installation)
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi

# Isaac Sim aliases
alias isaac-sim='${ISAAC_SIM_PATH}/runapp.sh'
alias isaac-check='${ISAAC_SIM_PATH}/isaac-sim.compatibility_check.sh'
alias isaac-python='${ISAAC_SIM_PATH}/python.sh'

# ROS2 workspace aliases
alias cb='cd /workspaces/welding_cell_ws/ros2_ws && colcon build --symlink-install'
alias cbs='cd /workspaces/welding_cell_ws/ros2_ws && colcon build --symlink-install --packages-select'
alias sw='source /workspaces/welding_cell_ws/ros2_ws/install/setup.bash 2>/dev/null || echo "Workspace not built yet"'

# Isaac Sim ROS workspace
alias sw-isaac='source /workspaces/welding_cell_ws/IsaacSim-ros_workspaces/jazzy_ws/install/local_setup.bash 2>/dev/null || echo "Isaac ROS workspace not built yet"'

# Source ROS2 workspace if it exists
if [ -f /workspaces/welding_cell_ws/ros2_ws/install/setup.bash ]; then
    source /workspaces/welding_cell_ws/ros2_ws/install/setup.bash
fi

# UR5e connection test
alias ur5e-ping='ping -c 3 ${UR5E_ROBOT_IP}'

# Quick ROS 2 commands
alias ros2-topics='ros2 topic list'
alias ros2-nodes='ros2 node list'

echo "=========================================="
echo "UR5e Digital Twin Environment Ready!"
echo "Robot IP: ${UR5E_ROBOT_IP}"
echo "Isaac Sim: ${ISAAC_SIM_PATH}"
echo "ROS 2 Distro: ${ROS_DISTRO}"
echo "=========================================="
echo "Quick commands:"
echo "  isaac-sim     - Start Isaac Sim GUI"
echo "  isaac-check   - Run compatibility check"
echo "  isaac-python  - Isaac Sim Python interpreter"
echo "  ros2 topic list - List ROS 2 topics"
echo "  cb            - Build ROS2 workspace"
echo "  sw            - Source ROS2 workspace"
echo "  ur5e-ping     - Test connection to robot"
echo "=========================================="
EOF

echo "=========================================="
echo "Setup complete!"
echo "=========================================="
echo ""
echo "ROS 2 Jazzy installed (Ubuntu 24.04 default)"
echo "Isaac Sim uses internal ROS 2 Jazzy libraries"
echo ""
echo "For more info: https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.html"
echo "=========================================="
