source install/setup.bash
LAST_ROSBAG_DIR=$(ls -td rosbag* | head -1)
ros2 bag info $LAST_ROSBAG_DIR
ros2 bag play $LAST_ROSBAG_DIR