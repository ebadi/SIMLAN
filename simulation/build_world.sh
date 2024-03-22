echo "--- killing processes"
pkill -9 -f gzserver
pkill -9 -f gzclient
pkill -9 -f gazebo
pkill -9 -f rviz
pkill -9 -f humble

echo "--- removing build files"
rm -rf ./build ./install ./log

colcon build --symlink-install --cmake-args "-Wno-dev"
source install/setup.bash
export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11
#ros2 launch infobot_gazebo_environment infobot_warehouse.launch.py
ros2 launch infobot_gazebo_environment infobot_factory.launch.py