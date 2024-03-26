echo "--- removing build files"
rm -rf ./build ./install ./log

colcon build --symlink-install --cmake-args "-Wno-dev"
