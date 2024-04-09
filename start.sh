#! /bin/bash

export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11
./build.sh
source install/setup.bash
source /opt/dependencies_ws/install/setup.bash

ros2 launch infobot_gazebo_environment infobot_factory.launch.py &

 sleep 7 

if [[ "$*" == *"all"* ]]
then
    ros2 launch infobot_agent infobot.launch.py &
    ros2 launch dyno_jackal_bringup sim.launch.py &
    ros2 launch static_agent_launcher static-agent.launch.py &
else
    echo "No agents is activated"
fi
