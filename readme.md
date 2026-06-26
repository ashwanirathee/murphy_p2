## Murphy P2 Robot

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
  yolo_camera_uid:=2 \
  enable_slam:=false \
  enable_ear:=false \
  enable_audio:=false \
  enable_vlm:=false \
  enable_2dobd:=true \
  enable_3dobd:=false \
  yolo_camera_uid:=0 
  

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

docker exec -it murphy_ros bash
cd ~/murphy_p2
source install/setup.bash
python3 -m pip install --upgrade pip
python3 -m pip install ultralytics opencv-python-headless


docker exec -u 0 -it murphy_ros bash
sudo apt update
sudo apt install -y python3-pip
apt update
apt install -y python3-pip python3-opencv
cd /home/ubuntu/murphy_p2
source install/setup.bash

python3 -m pip install --break-system-packages ultralytics --no-deps
python3 -m pip install --break-system-packages --no-cache-dir matplotlib \
  torch torchvision \
  --index-url https://download.pytorch.org/whl/cpu
python3 -m pip install --break-system-packages matplotlib requests onnxruntime
python3 - <<'PY'
from ultralytics import YOLO

model = YOLO("/home/ubuntu/murphy_p2/yolov12n-face.onnx")
results = model.predict(
    "/home/ubuntu/murphy_p2/latest_frame.jpg",
    imgsz=320,
    device="cpu",
    verbose=True,
)

for r in results:
    print(r.boxes)
PY

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

### References:
- ROS 2 documentation: https://docs.ros.org/
- https://github.com/apple/ml-cubifyanything
- https://arxiv.org/abs/2005.14165
- https://arxiv.org/abs/2203.02155
- https://arxiv.org/pdf/2412.16720
- https://github.com/karpathy/autoresearch
- https://github.com/facebookresearch/dinov3
