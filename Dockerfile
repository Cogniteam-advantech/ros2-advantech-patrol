FROM dustynv/l4t-pytorch:r36.2.0

# Add ROS2 repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble Desktop and tools
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-argcomplete \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Source ROS2 in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Set environment variables
ENV ROS_DISTRO=humble
ENV AMENT_PREFIX_PATH=/opt/ros/humble
ENV COLCON_PREFIX_PATH=/opt/ros/humble
ENV LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH
ENV PATH=/opt/ros/humble/bin:$PATH
ENV PYTHONPATH=/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages:$PYTHONPATH
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2



SHELL ["/bin/bash", "-c"]

# RUN LINE BELOW TO REMOVE debconf ERRORS (MUST RUN BEFORE ANY apt-get CALLS)

RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections
RUN apt-get update && apt-get upgrade -y && apt-get -y install python3-pip && apt-get install -y --no-install-recommends apt-utils

# Install dependencies

RUN apt-get install build-essential git cmake libasio-dev -y

RUN apt install ros-humble-tf2-geometry-msgs -y

# Cloning this repository into your workspace
RUN mkdir -p ~/tracer_driver_ws/src
WORKDIR /root/tracer_driver_ws/src
RUN git clone https://github.com/westonrobot/ugv_sdk.git
RUN git clone https://github.com/agilexrobotics/tracer_ros2.git


RUN apt install -y can-utils iproute2
RUN apt update && apt install -y can-utils
RUN apt install -y can-utils iproute2 net-tools



# Create setup script that will be run when container starts
RUN echo '#!/bin/bash\n\
# Load kernel module\n\
modprobe gs_usb 2>/dev/null || echo "gs_usb module not available or already loaded"\n\
# Setup CAN interface\n\
ip link set can0 down 2>/dev/null || true\n\
ip link set can0 type can bitrate 500000 2>/dev/null || echo "Failed to configure can0 - interface may not exist"\n\
ip link set can0 up 2>/dev/null || echo "Failed to bring up can0 - check hardware connection"\n\
echo "CAN setup completed"\n\
exec "$@"' > /setup_can.sh && chmod +x /setup_can.sh




# nav2
# Install required ROS 2 packages
RUN apt-get update && \
    apt-get install -y \
    ros-humble-nav-msgs \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-rviz2 \
    ros-humble-vision-opencv \
    ros-humble-cv-bridge \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-tf-transformations \
    python3-pip


RUN  pip3 install pyyaml
