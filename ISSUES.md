## gazebo_ros2_control

```
[gzserver-1] [ERROR] [1727951819.671190014] [jackal.gazebo_ros2_control]: controller manager doesn't have an update_rate parameter
```

No solution

## OpenAL

```
[gzserver-1] [Err] [OpenAL.cc:84] Unable to open audio device[default]
```

Related to support for audio inside docker container. It will not be resolved

## Dark simulation

Make sure that nvidia driver is installed correctly

## Missing nvidia docker runtime

Solution: https://stackoverflow.com/questions/59008295/add-nvidia-runtime-to-docker-runtimes
