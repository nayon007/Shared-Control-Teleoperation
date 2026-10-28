# ROS 2 Humble (Ubuntu 22.04) + Gazebo Classic 11 + ros2_control for the
# SCE 571 shared-control project. Built and started via ../compose.yaml.
FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-gazebo-ros-pkgs \
      ros-humble-gazebo-ros2-control \
      ros-humble-ros2-control \
      ros-humble-ros2-controllers \
      ros-humble-xacro \
      ros-humble-joy \
      ros-humble-kdl-parser \
      ros-humble-tf2-eigen \
      ros-humble-joint-state-publisher-gui \
      ros-humble-robot-state-publisher \
      ros-humble-rosbag2-storage-mcap \
      python3-colcon-common-extensions \
      python3-pip \
      python3-pytest \
      python3-numpy \
      python3-matplotlib \
      libhidapi-dev \
      git nano less bash-completion \
    && rm -rf /var/lib/apt/lists/*

# DualSense rumble + adaptive trigger over hidraw
RUN pip3 install --no-cache-dir pydualsense

# Non-root user with the host's UID/GID so files in the mounted workspace
# stay owned by you on the host.
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} \
    && usermod -aG dialout,plugdev,video ${USERNAME} \
    && apt-get update && apt-get install -y --no-install-recommends sudo \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && rm -rf /var/lib/apt/lists/*

USER ${USERNAME}
WORKDIR /home/${USERNAME}/SCE_571_ws/ros2_ws

RUN rosdep update --rosdistro humble \
    && { echo 'source /opt/ros/humble/setup.bash'; \
         echo '[ -f ~/SCE_571_ws/ros2_ws/install/setup.bash ] && source ~/SCE_571_ws/ros2_ws/install/setup.bash'; \
         echo 'source /usr/share/gazebo/setup.sh 2>/dev/null'; \
         echo 'export RCUTILS_COLORIZED_OUTPUT=1'; } >> ~/.bashrc

CMD ["bash"]
