FROM osrf/ros:jazzy-desktop-full

RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-ur \
    ros-jazzy-robotiq-description \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-controller-manager \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-joint-trajectory-controller \
    git \
    python3-rosdep \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Separater Build-Workspace (wird nicht gemountet)
#WORKDIR /root/build_ws

# Clone repository und build
#RUN mkdir -p src && \
#    git clone -b ros2 https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation.git src/ur_simulation_gz && \
#    git clone https://github.com/PickNikRobotics/ros2_robotiq_gripper.git src/ros2_robotiq_gripper && \
#    git clone -b ros2 https://github.com/tylerjw/serial.git src/serial && \
#    apt-get update && \
#    rosdep update && \
#    rosdep install --from-paths src --ignore-src -r -y && \
#    . /opt/ros/jazzy/setup.sh && \
#    colcon build --symlink-install

# Arbeits-Workspace (wird gemountet)
WORKDIR /root/ros2_ws

# Setup bashrc mit conditional sourcing für eigene Pakete
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    #echo "source /root/build_ws/install/setup.bash" >> /root/.bashrc && \
    echo "[ -f /root/ros2_ws/install/setup.bash ] && source /root/ros2_ws/install/setup.bash" >> /root/.bashrc