FROM nvidia/cuda:12.8.1-cudnn-runtime-ubuntu24.04

ENV DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg \
    lsb-release \
    python3-pip \
    software-properties-common \
  && add-apt-repository universe \
  && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list \
  && apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    python3-colcon-common-extensions \
    python3-numpy \
    python3-opencv \
    ros-jazzy-ros-base \
  && python3 -m pip install --break-system-packages --no-cache-dir \
    torch \
    torchvision \
    torchaudio \
    --index-url https://download.pytorch.org/whl/cu128 \
  && python3 -m pip install --break-system-packages --no-cache-dir \
    onnx \
    onnxruntime-gpu==1.23.2 \
    pyyaml \
    safetensors \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY src ./src

RUN source /opt/ros/jazzy/setup.bash \
  && colcon build --symlink-install

ENTRYPOINT ["/bin/bash", "-lc"]
CMD ["source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && bash"]
