FROM ros:humble-perception

ENV WS_DIR="/ros2_ws"
WORKDIR ${WS_DIR}

SHELL ["/bin/bash", "-c"]

ARG DEBIAN_FRONTEND=noninteractive

# Install basic build tools and utilities
RUN apt-get update \
 && apt-get install -y \
    build-essential \
    cmake \
    git-all \
    software-properties-common \
    vim \
    jstest-gtk \
    sudo \
    kitty-terminfo \
    zsh \
    curl \
    wget \
 && rm -rf /var/lib/apt/lists/*

# Install RealSense packages
# Add Intel RealSense repository
RUN apt-get update \
 && apt-get install -y software-properties-common \
 && apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
 && add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u \
 && apt-get install -y \
    ros-${ROS_DISTRO}-realsense2-camera \
    ros-${ROS_DISTRO}-realsense2-description \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-image-transport-plugins \
    ros-${ROS_DISTRO}-diagnostic-updater \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-tf-transformations \
    librealsense2-utils \
    librealsense2-dev \
 && rm -rf /var/lib/apt/lists/*

# Install RViz2
RUN apt-get update \
 && apt-get install -y \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
 && rm -rf /var/lib/apt/lists/*


# Install RPLidar ROS2 packages
RUN apt-get update \
 && apt-get install -y \
    ros-${ROS_DISTRO}-rplidar-ros \
 && rm -rf /var/lib/apt/lists/*

# Install SLAM and mapping packages for lidar and VSLAM
RUN apt-get update \
 && apt-get install -y \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-cartographer \
    ros-${ROS_DISTRO}-cartographer-ros \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-rtabmap-ros \
    ros-${ROS_DISTRO}-rtabmap-slam \
    ros-${ROS_DISTRO}-rtabmap-odom \
    ros-${ROS_DISTRO}-rtabmap-viz \
    ros-${ROS_DISTRO}-rtabmap \
 && rm -rf /var/lib/apt/lists/*

# Install Python dependencies and zbar for QR code detection
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    libzbar0 \
    zbar-tools \
 && rm -rf /var/lib/apt/lists/*

# Install Python packages for the competition
RUN python3 -m pip install --no-cache-dir \
    pyserial \
    "numpy<2" \
    opencv-contrib-python \
    pyzbar \
    pillow \
    transforms3d

# Add udev rules for RealSense (optional, but good practice if running privileged)
RUN mkdir -p /etc/udev/rules.d \
 && wget -O /etc/udev/rules.d/99-realsense-libusb.rules https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules

# # Clone and build RPLidar SDK (optional, for additional driver support)
# RUN mkdir -p ${WS_DIR}/src \
#  && cd ${WS_DIR}/src \
#  && git clone https://github.com/Slamtec/rplidar_ros.git -b ros2 \
#  && cd ${WS_DIR} \
#  && source /opt/ros/${ROS_DISTRO}/setup.bash \
#  && colcon build --symlink-install \
#  && rm -rf build log


# Create a non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# Set up sudo for non-root user
RUN echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
  && chmod 0440 /etc/sudoers.d/$USERNAME

# Switch to non-root user
USER $USERNAME

# Set zsh as default shell for the user
RUN sudo chsh -s /usr/bin/zsh $USERNAME

# Source ROS2 in bashrc and zshrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/$USERNAME/.bashrc \
  && echo "export ROS_DOMAIN_ID=0" >> /home/$USERNAME/.bashrc \
  && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/$USERNAME/.bashrc \
  && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/$USERNAME/.zshrc \
  && echo "export ROS_DOMAIN_ID=0" >> /home/$USERNAME/.zshrc \
  && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/$USERNAME/.zshrc

ARG DEBIAN_FRONTEND=dialog
