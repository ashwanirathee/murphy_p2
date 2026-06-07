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
  -v /home/murphy/Documents/murphy_p2:/home/ubuntu/murphy_p2 \
  ros:jazzy-perception

source install/setup.bash
ros2 launch murphy_p2 bringup.launch.py \
  event_min_interval_sec:=5.0 \
  event_max_silence_sec:=5.0 \
  camera_uids:="[0, 2]" \
  camera_labels:='["left", "right"]'
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