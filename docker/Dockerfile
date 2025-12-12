# AMR Perception Container
# Base: ROS 2 Jazzy auf Ubuntu 24.04

FROM ros:jazzy-ros-base

# Nicht-interaktive Installation
ENV DEBIAN_FRONTEND=noninteractive

# System-Updates und Basis-Tools
RUN apt-get update && apt-get install -y \
    # Build-Tools
    build-essential \
    cmake \
    python3-pip \
    python3-colcon-common-extensions \
    # ROS 2 Tools
    ros-jazzy-rviz2 \
    ros-jazzy-rqt* \
    ros-jazzy-teleop-twist-keyboard \
    # Navigation & SLAM
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-robot-localization \
    # Sensor-Treiber
    ros-jazzy-rplidar-ros \
    ros-jazzy-camera-ros \
    # TF & Transforms
    ros-jazzy-tf2-tools \
    ros-jazzy-tf-transformations \
    # Visualisierung
    ros-jazzy-joint-state-publisher \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-xacro \
    # Utilities
    vim \
    htop \
    net-tools \
    iputils-ping \
    usbutils \
    && rm -rf /var/lib/apt/lists/*

# Workspace erstellen
WORKDIR /ros_ws
RUN mkdir -p src config maps launch

# Entrypoint
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]
