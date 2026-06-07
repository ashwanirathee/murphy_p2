
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
  --group-add video \
  --device /dev/video0 \
  --device /dev/video1 \
  --device /dev/video2 \
  --device /dev/video3 \
  -v /home/murphy/Documents/murphy_p2:/home/ubuntu/murphy_p2 \
  ros:jazzy-perception
```

Nodes:
```
cd ~/murphy_p2
source ~/murphy_p2/install/setup.bash
ros2 run murphy_p2 cameras_node --ros-args \
  -p camera_uids:="[0, 2]" \
  -p camera_labels:="[left, right]"
```

```
cd ~/murphy_p2
source ~/murphy_p2/install/setup.bash
ros2 run murphy_p2 visual_processor_node
```

```
docker exec -it murphy_ros bash
cd ~/murphy_p2
source ~/murphy_p2/install/setup.bash
ros2 run murphy_p2 brain_node
```

```
ros2 topic list
```