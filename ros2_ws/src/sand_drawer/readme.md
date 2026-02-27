Main ros2 folder is tiago_isaac_sim so we build all required packages like ur_config, cagopa_tiago_gazebo and others

before running the ros2-humble docker use this for gui

xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f /tmp/.docker.xauth nmerge -

source /opt/ros/humble/setup.bash && colcon build
to build

also source ~/.bashrc

ros-humble-rmw-cyclonedds-cpp or
rmw_fastrtps_cpp package
ros-humble-control-msgs
sudo apt install ros-humble-moveit
apt update && apt install ros-humble-joint-state-publisher-gui
ros-humble-xacro

apt update && apt install ros-humble-rmw-cyclonedds-cpp install ros-humble-rmw-fastrtps-cpp ros-humble-control-msgs ros-humble-moveit ros-humble-joint-state-publisher-gui ros-humble-xacro ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-ur


install everytime for docker

Dont forget to source install/setup.bash after building

https://docs.isaacsim.omniverse.nvidia.com/4.5.0/ros2_tutorials/tutorial_ros2_manipulation.html

# -isaac environment setup-

## RedBall

path /World/RedBall

materials /World/Looks/OmniPBR

rigid body enabled, kinematic enabled, collision enabled

scale 0.1

## Camera

name : rsd455

camera pseudo depth

action graph here

    on playback tick - no mods
    isaac run one simulation frame - no mods
    isaac create render product - inputs:cameraprim -> set to depth camera
    ros2 context - no mods
    ros2 camera helper - frameid -> depth, topic name -> rsd455_depth, type-> depth, use system time-> check

camera color
    on playback tick - no mods
    isaac run one simulation frame - no mods
    isaac create render product - inputs:cameraprim -> set to color camera
    ros2 context - no mods
    ros2 camera helper - frameid -> rsd455, topic name -> rsd455_img, type-> rgb, use system time-> check

## Robot



---

for 2 separate docker containers for sim and ros2

this was the working bashrc modification alias

# Alias to start Isaac Sim 4.5.0 container as the current user
alias isaac-sim-4.5='docker run --name isaac-sim --entrypoint bash -it --runtime=nvidia --gpus all -e "ACCEPT_EULA=Y" --rm --network=host \
    -v ~/docker/fastdds/fastdds_discovery_server.xml:/fastdds.xml:ro \
    -e "FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds.xml" \
    --ipc=host \
    -e "ROS_DOMAIN_ID=30" \
    -e "RMW_IMPLEMENTATION=rmw_fastrtps_cpp" \
    -e "ROS_DISTRO=humble" \
    -e "LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib" \
    -e "PRIVACY_CONSENT=Y" \
    -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
    -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
    -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
    -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
    -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
    -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
    -v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
    -v ~/docker/isaac-sim/documents:/root/Documents:rw \
    nvcr.io/nvidia/isaac-sim:4.5.0'

#xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f /tmp/.docker.xauth nmerge -
alias ros2-humble='docker run -it --rm --network=host \
    -v ~/docker/fastdds/fastdds_discovery_server.xml:/fastdds.xml:ro \
    -e "FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds.xml" \
    --ipc=host \
    -e "ROS_DOMAIN_ID=30" \
    -e "RMW_IMPLEMENTATION=rmw_fastrtps_cpp" \
    -e "DISPLAY=${DISPLAY}" \
    -e "QT_X11_NO_MITSHM=1" \
    -e "XAUTHORITY=/tmp/.docker.xauth" \
    -v ~/isaac_ws_cago:/ros2_ws \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /tmp/.docker.xauth:/tmp/.docker.xauth:rw \
    -v ~/.ros:/root/.ros:rw \
    -w /ros2_ws \
    osrf/ros:humble-desktop bash'