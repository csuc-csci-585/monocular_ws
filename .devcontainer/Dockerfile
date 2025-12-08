FROM osrf/ros:humble-desktop

# Add ubuntu user with same UID and GID as your host system, if it doesn't already exist
# Since Ubuntu 24.04, a non-root user is created by default with the name vscode and UID=1000
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions \
      python3-argcomplete \
      build-essential \
      git \
      nano \
      wget \
      # Python dependencies for DepthAnything V2
      python3-pip \
      python3-torch \
      python3-torchvision \
      python3-opencv \
      python3-numpy \
      python3-matplotlib \
      # Add dependencies for rtabmap, depth processing, camera drivers
      ros-${ROS_DISTRO}-rtabmap-ros \
      ros-${ROS_DISTRO}-rtabmap-launch \
      ros-${ROS_DISTRO}-image-transport \
      ros-${ROS_DISTRO}-image-transport-plugins \
      ros-${ROS_DISTRO}-compressed-image-transport \
      ros-${ROS_DISTRO}-vision-opencv \
      ros-${ROS_DISTRO}-cv-bridge \
      # TurtleBot3 dependencies - install all Gazebo packages as per e-manual
      ros-${ROS_DISTRO}-gazebo-* \
      ros-${ROS_DISTRO}-cartographer \
      ros-${ROS_DISTRO}-cartographer-ros \
      ros-${ROS_DISTRO}-navigation2 \
      ros-${ROS_DISTRO}-nav2-bringup \
      ros-${ROS_DISTRO}-turtlebot3-msgs \
      ros-${ROS_DISTRO}-dynamixel-sdk \
      # RViz2 visualization
      ros-${ROS_DISTRO}-rviz2 \
      ros-${ROS_DISTRO}-rviz-common \
      ros-${ROS_DISTRO}-rviz-default-plugins \
      # Essential RQT tools for ROS2 Humble
      ros-${ROS_DISTRO}-rqt \
      ros-${ROS_DISTRO}-rqt-common-plugins \
      ros-${ROS_DISTRO}-rqt-graph \
      ros-${ROS_DISTRO}-rqt-image-view \
      ros-${ROS_DISTRO}-rqt-plot \
      ros-${ROS_DISTRO}-rqt-console \
      ros-${ROS_DISTRO}-rqt-reconfigure \
      && rm -rf /var/lib/apt/lists/*

# Install additional Python dependencies for DepthAnything V2
RUN pip3 install transformers timm einops

# Create a non-root user that matches host UID/GID
RUN groupadd --gid $USER_GID ros && \
    useradd --uid $USER_UID --gid $USER_GID -m --shell /bin/bash ros && \
    usermod -aG sudo ros && \
    echo 'ros ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# Switch to ros user and set up workspace
USER ros

# Create a separate directory for upstream repos
WORKDIR /home/ros/upstream_repos
RUN git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git && \
    git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git && \
    git clone https://github.com/grupo-avispa/depth_anything_v2_ros2.git

# Create workspace src directory and symlink upstream repos
RUN mkdir -p /home/ros/monocular_ws/src && \
    ln -s /home/ros/upstream_repos/* /home/ros/monocular_ws/src/

# Build the workspace (all upstream dependencies)
WORKDIR /home/ros/monocular_ws
RUN bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Setup bashrc for ros user
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc && \
    echo "source /home/ros/monocular_ws/install/setup.bash" >> ~/.bashrc && \
    echo "alias cb='colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'" >> ~/.bashrc && \
    echo "alias cbs='colcon build --symlink-install --packages-select'" >> ~/.bashrc && \
    echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc && \
    echo "# export TURTLEBOT3_MODEL=burger" >> ~/.bashrc

# Reset to interactive mode for runtime
ENV DEBIAN_FRONTEND=

WORKDIR /home/ros

CMD ["bash"]