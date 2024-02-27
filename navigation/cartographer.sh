export INFOBOT_MODEL=infobot
source install/setup.bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
# rviz -d cart.rviz
# ros2 run infobot_teleop teleop_keyboard
# ros2 run nav2_map_server map_saver_cli -f maps/volvo
