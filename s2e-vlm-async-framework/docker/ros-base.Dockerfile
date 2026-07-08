FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    ffmpeg \
    python3-colcon-common-extensions \
    python3-numpy \
    python3-opencv \
    python3-pip \
    python3-pytest \
    ros-jazzy-cv-bridge \
    ros-jazzy-tf2-ros \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY src ./src

RUN source /opt/ros/jazzy/setup.bash \
  && colcon build --symlink-install

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
