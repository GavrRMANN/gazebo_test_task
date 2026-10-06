FROM osrf/ros:humble-desktop-jammy

ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]


RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    rm -f /etc/apt/apt.conf.d/docker-clean && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        git \
        curl \
        nano \
        mesa-utils \
        python3-colcon-common-extensions \
        xvfb \
        x11vnc \
        fluxbox \
        novnc \
        websockify

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    rm -f /etc/apt/apt.conf.d/docker-clean && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-humble-gazebo-ros-pkgs \
        ros-humble-xacro

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-simple-commander \
    ros-humble-slam-toolbox \
    ros-humble-robot-localization \
    ros-humble-tf2-tools \
    && rm -rf /var/lib/apt/lists/*

ENV DISPLAY=:1
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV ROS_DOMAIN_ID=42

WORKDIR /workspace

COPY docker/entrypoint.sh /entrypoint.sh

RUN chmod +x /entrypoint.sh

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo 'if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi' >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]

CMD ["bash"]