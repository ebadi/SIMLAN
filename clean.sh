echo "--- killing processes ---"
pkill -9 -f gzserver
pkill -9 -f gzclient
pkill -9 -f gazebo
pkill -9 -f rviz
pkill -9 -f humble

echo "--- removing build files"
rm -rf ./build ./install ./log

