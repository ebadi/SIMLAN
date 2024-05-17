#! /bin/bash

export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11
export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:/usr/share/gazebo-11/models
source install/setup.bash
source /opt/dependencies_ws/install/setup.bash
if [ $# -eq 0 ]
  then
    echo "No arguments supplied. ** Try passing 'sim' as an argument ** "
fi

clean () {
    echo "--- killing processes ---"
    pkill -9 -f gzserver
    pkill -9 -f gzclient
    pkill -9 -f gazebo
    pkill -9 -f rviz
    pkill -9 -f humble

    echo "--- removing build files"
    rm -rf ./build ./install ./log
    rm -rf rosbag* ; rm -rf processing/images_data/*
}

build () {
    clean
    colcon build --merge-install --symlink-install --cmake-args " -Wno-dev "
    echo "successful build"
}


if [[ "$*" == *"clean"* ]]
then
    clean
elif [[ "$*" == *"build"* ]]
then
    build
elif [[ "$*" == *"sim"* ]]
then
    # environment
    # SIM_ENV=CICD or DEV
    # agents
    (sleep 8 && ros2 launch infobot_agent infobot.launch.py)&
    echo "> Infobot is queed to be spawned"
    (sleep 8 && ros2 launch dyno_jackal_bringup sim.launch.py)&
    echo "> Jackal is queed to be spawned"
    (sleep 8 && ros2 launch static_agent_launcher static-agent.launch.py)&
    echo "> Static agents are queed to be spawned"
    echo "> starting Gazebo"
    # This has to be blocking so that k8s can restart when it crashes
    ros2 launch infobot_gazebo_environment infobot_factory.launch.py


elif [[ "$*" == *"rviz"* ]]
then
    rviz2 -d ./processing/rviz_config.rviz
elif [[ "$*" == *"jackal_teleop"* ]]
then
    ros2 launch dyno_jackal_bringup keyboard_steering.launch.py
elif [[ "$*" == *"infobot_teleop"* ]]
then
    ros2 run infobot_teleop teleop_keyboard
elif [[ "$*" == *"ros_record"* ]]
then
    ros2 bag record /cmd_vel
elif [[ "$*" == *"ros_replay"* ]]
then
    # replay last recording
    LAST_ROSBAG_DIR=$(ls -td rosbag* | head -1)
    ros2 bag info $LAST_ROSBAG_DIR
    ros2 bag play $LAST_ROSBAG_DIR
elif [[ "$*" == *"slam"* ]]
then
    ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
    # ros2 run nav2_map_server map_saver_cli -f maps/mapname
elif [[ "$*" == *"commander"* ]]
then
    ros2 run nav2_commander commander
elif [[ "$*" == *"nav"* ]]
then
    ros2 launch infobot_navigation2 navigation2.launch.py use_sim_time:=True map:=maps/emptywarehouse.yaml
elif [[ "$*" == *"cam_record"* ]]
then
    # python3 ./camera_subscriber.py
    # python3 ./camera_subscriber.py  --action save
    # python3 ./camera_subscriber.py  --action removebg  --algo KNN
    # algo: MOG2, KNN
    cd ./processing ; python3 camera_subscriber.py --action save --camera $2
elif [[ "$*" == *"screenshot"* ]]
then
    cd ./processing ; python3 camera_subscriber.py --action screenshot --camera $2 --shottime 4
elif [[ "$*" == *"test"* ]]
then
    # Running python unittest, https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Python.html
    # https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Python.html
    colcon test --packages-select ros2_test --pytest-args --verbose
elif [[ "$*" == *"collect_data"* ]]
then
    ros2 launch data_collection collect_data.launch.py && 
    sleep 5 &&
    ros2 run data_collection move_object 
fi
