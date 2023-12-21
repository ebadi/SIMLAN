echo "--- removing build files"
rm -rf ./build ./install ./log


export INFOBOT_MODEL=infobot
colcon build --symlink-install --cmake-args "-Wno-dev"
source install/setup.bash

ros2 launch infobot_navigation2 navigation2.launch.py use_sim_time:=True map:=maps/emptywarehouse.yaml