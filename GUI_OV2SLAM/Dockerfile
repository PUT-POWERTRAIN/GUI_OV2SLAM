# System architecture: amd64 or arm64v8
ARG ARCHITECTURE=amd64
ARG ROS_DISTRO=humble
ARG BASE_IMAGE=${ARCHITECTURE}/ros:${ROS_DISTRO}-ros-core
FROM $BASE_IMAGE

# Podstawowe narzędzia i ROS 2 repo
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y curl \
    && sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list' \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Narzędzia budowania
RUN apt update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    python3-colcon-common-extensions build-essential cmake pkg-config git \
    && rm -rf /var/lib/apt/lists/*

# Biblioteki graficzne dla ImGui (OpenGL, GLFW, X11)
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
        libgl1-mesa-dev \
        libglu1-mesa-dev \
        libglfw3-dev \
        libglew-dev \
        libx11-dev \
        libxrandr-dev \
        libxinerama-dev \
        libxcursor-dev \
        libxi-dev \
        libfreetype6-dev \
        x11-apps \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws/src/imgui_app/Thirdparty

##############
### ImGui  ###
##############
RUN git clone https://github.com/ocornut/imgui.git
WORKDIR /ws/src/imgui_app/Thirdparty/imgui
RUN git checkout docking

##################
### ImPlot     ###
##################
WORKDIR /ws/src/imgui_app/Thirdparty
RUN git clone https://github.com/epezent/implot.git

###################
### GLM (math)  ###
###################
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
        libglm-dev \
    && rm -rf /var/lib/apt/lists/*

######################
### ROS 2 Packages ###
######################
RUN apt update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
        ros-${ROS_DISTRO}-rclcpp \
        ros-${ROS_DISTRO}-std-msgs \
        ros-${ROS_DISTRO}-sensor-msgs \
        ros-${ROS_DISTRO}-geometry-msgs \
        ros-${ROS_DISTRO}-nav-msgs \
        ros-${ROS_DISTRO}-tf2 \
        ros-${ROS_DISTRO}-tf2-ros \
        ros-${ROS_DISTRO}-tf2-geometry-msgs \
        ros-${ROS_DISTRO}-cv-bridge \
    && rm -rf /var/lib/apt/lists/*
###################
### OpenCV      ###
###################
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
        libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

############################
### ImGui Application    ###
############################
ADD . /ws/src/imgui_app

WORKDIR /ws
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --symlink-install

# Ustaw zmienne środowiskowe dla X11
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1

# Source ROS setup
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source /ws/install/setup.bash" >> ~/.bashrc

WORKDIR /ws

CMD ["/bin/bash"]
