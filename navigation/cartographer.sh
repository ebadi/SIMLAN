export INFOBOT_MODEL=infobot
source install/setup.bash
ros2 launch infobot_cartographer cartographer.launch.py use_sim_time:=True
ros2 run infobot_teleop teleop_keyboard
ros2 run nav2_map_server map_saver_cli -f maps/slam

ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
