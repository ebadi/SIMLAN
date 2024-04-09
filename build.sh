echo "--- killing processes ---"
pkill -9 -f gzserver
pkill -9 -f gzclient
pkill -9 -f gazebo
pkill -9 -f rviz
pkill -9 -f humble

colcon build --merge-install --symlink-install --cmake-args "-Wno-dev"  --cmake-args " -DCMAKE_BUILD_TYPE=RelWithDebInfo"
