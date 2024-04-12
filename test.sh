#! /bin/bash

# Running python unittest, https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Python.html
# https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Python.html

./start.sh clean
colcon test --packages-select ros2_test --pytest-args --verbose
