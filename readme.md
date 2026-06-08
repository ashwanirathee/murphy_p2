## Murphy P2: Physical AI Platform

This repository contains the code for Murphy P2, a Physical AI platform for running experiments related to robot perception, reasoning, and control. The system is built around a Raspberry Pi 5 and uses ROS 2 as the software framework to modularize the different components of the robot.

### Setup Instructions:
```
mkdir -p /home/murphy/Documents/murphy_p2/src
cd /home/murphy/Documents/murphy_p2/src

ros2 pkg create murphy_p2 \
  --build-type ament_python \
  --dependencies rclpy sensor_msgs std_msgs cv_bridge

cd /home/ubuntu/murphy_p2/src
cd /home/ubuntu/murphy_p2/src/murphy_p2/murphy_p2

sudo chown -R murphy:murphy /home/murphy/Documents/murphy_p2 # fix ownership of the files in the host machine

rm -rf build install log

colcon build
source install/setup.bash

docker run -it --rm \
  --name murphy_ros \
  --user $(id -u):$(id -g) \
  --add-host=host.docker.internal:host-gateway \
  --group-add video \
  --device /dev/video0 \
  --device /dev/video1 \
  --device /dev/video2 \
  --device /dev/video3 \
  -p 8765:8765 \
  -v /home/murphy/Documents/murphy_p2:/home/ubuntu/murphy_p2 \
  ros:jazzy-perception

cd ~/murphy_p2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch murphy_p2 bringup.launch.py \
  event_min_interval_sec:=5.0 \
  event_max_silence_sec:=5.0 \
  camera_uids:="[0, 2]" \
  camera_labels:='["left", "right"]' \
  enable_slam:=true \
  enable_ear:=false \
  enable_audio:=false \
  enable_vlm:=false
  

docker exec -u 0 -it murphy_ros bash
apt update
apt install -y ros-jazzy-foxglove-bridge
exit

docker exec -it murphy_ros bash

ros2 topic hz /camera/uid_0/image_raw
ros2 topic hz /camera/uid_2/image_raw

cd ~/murphy_p2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml

ws://192.168.68.59:8765
```

### Nodes:

##### Cameras Node:
```
cd ~/murphy_p2
source ~/murphy_p2/install/setup.bash
ros2 run murphy_p2 cameras_node --ros-args \
  -p camera_uids:="[0, 2]" \
  -p camera_labels:="[left, right]"
```

##### Visual Processor Node:
```
cd ~/murphy_p2
source ~/murphy_p2/install/setup.bash
ros2 run murphy_p2 visual_processor_node
```

##### Brain Node:
```
docker exec -it murphy_ros bash
cd ~/murphy_p2
source ~/murphy_p2/install/setup.bash
ros2 run murphy_p2 brain_node
```

##### Audio Node and its bluetooth bridge:
```
docker exec -it murphy_ros bash
cd ~/murphy_p2
source ~/murphy_p2/install/setup.bash
ros2 run murphy_p2 audio_node

# audio control
pactl list short sinks
pactl set-sink-volume bluez_output.41_42_12_84_8B_60.1 40%

# audio bridge controlling the bluetooth speaker from the host machine
chmod +x /home/murphy/Documents/murphy_p2/src/speaker_bridge.sh
apt update
apt install -y espeak alsa-utils
sudo apt install -y sox

docker exec -it murphy_ros bash
cd ~/murphy_p2
source ~/murphy_p2/install/setup.bash
ros2 topic pub --once /audio/heard_text std_msgs/msg/String "{data: 'how many cans are there and are of which brand? what about bottles?'}"

# run the speaker bridge in the host machine to forward audio from ROS to the bluetooth speaker
/home/murphy/Documents/murphy_p2/src/speaker_bridge.sh
```

##### Ear Node:

```
docker exec -it murphy_ros bash
cd ~/murphy_p2
source ~/murphy_p2/install/setup.bash
ros2 run murphy_p2 ear_node
```

##### Action Node:

```
docker exec -it murphy_ros bash
cd ~/murphy_p2
source ~/murphy_p2/install/setup.bash
ros2 run murphy_p2 action_node
```

##### VLM Node:

```
ollama run moondream # on the host

```

##### Remaining:
```
- OTA updates
- simulation + physical hardware
- kinematics, dynamics, control theory, and state estimation in simulation + sensor fusion across IMU, lidar, cameras, and force/torque sensors
- performance optimization, multithreading, memory management, and real-time systems; Python for tooling and scripting. devbuggin
- embedded or low-level systems and familiarity with communication protocols such as CAN, EtherCAT, Ethernet, and serial
-  MuJoCo, Gazebo, or NVIDIA Isaac Sim all work
- SLAM, path planning, or perception pipelines
- reinforcement learning or imitation learning workflows and deploying learned policies to hardware
- GPU/MPS

VLA models:

- OpenVLA — best practical/open baseline.
- π0 / openpi — best “physical AI frontier” direction.
- RT-2 — best conceptual foundation.
- Octo / Diffusion Policy — important robot-policy baselines.
- ChatVLA-2 — interesting for reasoning-heavy embodied agents.
- Gemini Robotics Model

Probability/Statistic/Linear Algebra/ML/DL/Vision/DSA

- perception, reconstruction, diffusion, world models and pre training vision models.
- BEV, Sparse Transformer architectures and Vision-Language Models (VLMs)
- object detection, segmentation, point cloud processing
- transformer architectures, attention mechanisms, and modern generative modeling (diffusion, flow matching) 
- representation learning from real world.
- Experience streaming video at scale — RTSP, GStreamer, multi-cam sync, edge-to-cloud pipelines.

Embedded Firmware
Language: C++, Python
ML Frameworks: PyTorch
Device: Jetson, Pi, Qualcomm
Inference Engines: TensorRT, ONNX on GPU/FPGA achieving >= 500Hz for microcontrollers (STM32, NXP, TI). VHDL/Verilog within RTOS environments (FreeRTOS, Zephyr).
Classification: 
Object Detection: 2D/3D, YOLO, RT-DETR, CNNs, OWLv2, BoxerNet. Yolo
Object Tracking: 
Segmentation:
Depth Estimation:
Pose Estimation:
ROS2
Docker
CI/CD
Sensor Fusion/Calibration. Data Synchronization: RGB, thermal, LiDAR/ToF, IMU, encoder.  robust, real‑time Regions of Interest (ROIs).
State Estimation: Kalman filters, factor graphs
Techniques:  extrinsic calibration, SLAM, visual odometry, point cloud registration
Develop dashboards and telemetry for drift analysis, hardware health monitoring, performance metrics, and automated retraining triggers.

low-level image-processing techniques, such as deconvolution, low SNR detection, and motion-isolation techniques.

global‑shutter and rolling‑shutter cameras, thermal imagers, LiDAR/ToF modules, IMUs) over GigE Vision, USB3 Vision, CAN, SPI, and I²C protocols; 

VLMs for auto-labeling or offline perception tasks
metric-semantic mapping, visual relocalization, and SLAM

```

### References:
- ROS 2 documentation: https://docs.ros.org/