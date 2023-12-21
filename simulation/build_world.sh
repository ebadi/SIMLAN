echo "--- killing processes"
pkill -9 -f gzserver
pkill -9 -f gzclient
pkill -9 -f gazebo
pkill -9 -f rviz
pkill -9 -f humble
pkill -9 -f ros

echo "--- removing build files"
rm -rf ./build ./install ./log

colcon build --symlink-install --cmake-args "-Wno-dev"
source install/setup.bash
ros2 launch infobot_gazebo_environment infobot_warehouse.launch.py
