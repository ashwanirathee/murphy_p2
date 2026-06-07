#!/bin/bash

docker exec -i murphy_ros bash -lc \
"cd /home/ubuntu/murphy_p2 && source install/setup.bash && ros2 topic echo /audio/speech_text --field data" \
| while read line; do
    if [ -n "$line" ] && [ "$line" != "---" ]; then
        echo "[SPEAK] $line"
        espeak-ng -v en-us  -s 130 -p 35 -a 120 "$line"
    fi
done