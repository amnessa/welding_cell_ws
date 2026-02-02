#!/bin/bash
# Run this script on the HOST machine BEFORE starting the dev container
# This prepares the Isaac Sim cache directories and X11 permissions

set -e

echo "=========================================="
echo "Preparing host for Isaac Sim Dev Container"
echo "=========================================="

# Create Isaac Sim cache directories
echo "Creating Isaac Sim cache directories..."
mkdir -p ~/docker/isaac-sim/cache/main/ov
mkdir -p ~/docker/isaac-sim/cache/main/warp
mkdir -p ~/docker/isaac-sim/cache/computecache
mkdir -p ~/docker/isaac-sim/config
mkdir -p ~/docker/isaac-sim/data/documents
mkdir -p ~/docker/isaac-sim/data/Kit
mkdir -p ~/docker/isaac-sim/logs
mkdir -p ~/docker/isaac-sim/pkg

# Set permissions (Isaac Sim uses uid 1234)
echo "Setting permissions..."
sudo chown -R 1234:1234 ~/docker/isaac-sim

# Allow X11 connections from local containers
echo "Allowing X11 connections..."
xhost +local:

# Pull the Isaac Sim image
echo "Pulling Isaac Sim Docker image..."
docker pull nvcr.io/nvidia/isaac-sim:5.1.0

# Check GPU driver
echo ""
echo "GPU Driver Information:"
nvidia-smi

echo ""
echo "=========================================="
echo "Host preparation complete!"
echo "=========================================="
echo ""
echo "Next steps:"
echo "1. Open VS Code in the workspace folder"
echo "2. Reopen in Container (Ctrl+Shift+P -> 'Reopen in Container')"
echo "3. Run 'isaac-check' to verify Isaac Sim compatibility"
echo "4. Run 'isaac-sim' to start Isaac Sim GUI"
echo "=========================================="
