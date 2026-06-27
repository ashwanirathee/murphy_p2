FROM ros:jazzy-perception

ENV DEBIAN_FRONTEND=noninteractive

# System + ROS dependencies
RUN apt update && apt install -y \
    python3-pip \
    python3-opencv \
    python3-requests \
    python3-numpy \
    python3-yaml \
    espeak \
    alsa-utils \
    sox \
    ros-jazzy-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*

# Python packages used by perception / VLM helpers
RUN python3 -m pip install --break-system-packages --no-cache-dir \
    requests \
    matplotlib \
    onnxruntime \
    ultralytics --no-deps

# PyTorch CPU wheels
RUN python3 -m pip install --break-system-packages --no-cache-dir \
    torch torchvision \
    --index-url https://download.pytorch.org/whl/cpu

WORKDIR /home/ubuntu/r1

CMD ["/bin/bash"]
