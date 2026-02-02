# UR5e Digital Twin - Welding Cell Workspace

A ROS2 Humble workspace for developing algorithms and implementing sensors for a UR5e robot using NVIDIA Isaac Sim as the simulation environment.

## Overview

This project creates a digital twin of a physical UR5e robot for:
- Algorithm development and testing
- Sensor integration and simulation
- Motion planning and control
- Welding cell automation

## Physical Robot Configuration

| Parameter | Value |
|-----------|-------|
| Robot Model | Universal Robots UR5e |
| IP Address | 192.168.8.4 |
| Connection | Ethernet |

## Prerequisites

### Host System Requirements
- Ubuntu 22.04 (recommended)
- NVIDIA GPU with driver 535+
- Docker with NVIDIA Container Toolkit
- VS Code with Remote Containers extension
- At least 32GB RAM (64GB recommended for Isaac Sim)
- 50GB+ free disk space

### Verify GPU Driver
```bash
nvidia-smi
```

## Quick Start

### 1. Prepare Host Machine
Run the preparation script on your host (not in container):
```bash
chmod +x scripts/prepare_host.sh
./scripts/prepare_host.sh
```

### 2. Open in Dev Container
1. Open this workspace in VS Code
2. Press `Ctrl+Shift+P` → "Dev Containers: Reopen in Container"
3. Wait for the container to build and setup

### 3. Verify Setup
Inside the container:
```bash
# Check Isaac Sim compatibility
isaac-check

# Test connection to physical robot
ur5e-ping

# Start Isaac Sim GUI
isaac-sim
```

## Workspace Structure

```
welding_cell_ws/
├── .devcontainer/
│   ├── devcontainer.json    # Dev container configuration
│   └── setup.sh             # Container setup script
├── ros2_ws/
│   └── src/                 # ROS2 packages
├── scripts/
│   └── prepare_host.sh      # Host preparation script
└── README.md
```

## Useful Commands

| Command | Description |
|---------|-------------|
| `isaac-sim` | Start Isaac Sim GUI |
| `isaac-check` | Run Isaac Sim compatibility check |
| `isaac-python` | Run Python with Isaac Sim environment |
| `cb` | Build ROS2 workspace |
| `cbs <pkg>` | Build specific package |
| `sw` | Source ROS2 workspace |
| `ur5e-ping` | Test connection to UR5e robot |

## Connecting to Physical UR5e

### Using ur_robot_driver
```bash
# Launch UR5e driver (after installing ur_robot_driver)
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:=192.168.8.4 \
    launch_rviz:=true
```

### Network Configuration
Ensure your host machine's network interface is configured to reach `192.168.8.4`:
- Set static IP on same subnet (e.g., 192.168.8.100)
- Or configure appropriate routing

## Isaac Sim + ROS2 Integration

Isaac Sim 5.1.0 includes ROS2 Humble support. Key features:
- ROS2 Bridge for communication
- URDF/USD robot model loading
- Sensor simulation (cameras, lidars, etc.)
- Physics-accurate robot simulation

### Loading UR5e in Isaac Sim
1. Start Isaac Sim: `isaac-sim`
2. Open Isaac Assets → Robots → Universal Robots → UR5e
3. Enable ROS2 Bridge extension
4. Configure joint publishers/subscribers

## Development Workflow

1. **Algorithm Development**: Write and test algorithms in simulation
2. **Sensor Integration**: Add virtual sensors in Isaac Sim
3. **Validation**: Compare simulation with physical robot
4. **Deployment**: Transfer validated code to physical robot

## Troubleshooting

### Isaac Sim Won't Start
- Verify GPU driver: `nvidia-smi`
- Check X11 forwarding: `xhost +local:` on host
- Ensure cache directories exist with correct permissions

### Cannot Connect to Robot
- Verify network connectivity: `ping 192.168.8.4`
- Check robot is powered on and in remote control mode
- Ensure firewall allows ROS2 traffic

### Container Build Fails
- Ensure Isaac Sim image is pulled: `docker pull nvcr.io/nvidia/isaac-sim:5.1.0`
- Check Docker has GPU access: `docker run --gpus all nvidia/cuda:12.0-base nvidia-smi`

## Resources

- [NVIDIA Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/)
- [Universal Robots ROS2 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [MoveIt2 Documentation](https://moveit.picknik.ai/main/index.html)

## License

[Add your license here]
