# InfoBot Navigation Within a Simulation Warehouse

This is the repository for research project activities at Infotiv Technology Development.

Find technical and more detailed documentation and specifications in the following links:

- [Simulation](simulation/README.md)
- [Building Gazebo models (Blender/Phobos)](simulation/raw_models/README.md)
- [Objects specifications](simulation/raw_models/objects/README.md)
- [Warehouse specification](simulation/raw_models/warehouse/README.md)

![Warehouse in Gazebo and ROS](resources/warehouse.png)

[video demo 1](resources/demo1.mp4)
,
[video demo 2](resources/demo2.mp4)


Click on image below to see the Volvo layout demo:

[![Delivery 2, Volvo layout](https://img.youtube.com/vi/f8ULCZFEM5Q/0.jpg)](https://www.youtube.com/watch?v=f8ULCZFEM5Q)

## Installation in production environment

These steps are used to install ROS (Humble) and Gazebo (11.10.2) on
  - Ubuntu 22.04
  - Windows 10 (after installing "ubuntu 22.04.3 LTS" WSL2 from Microsoft Store).

### ROS, RViz, Gazebo

We follow [the installation instruction](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) to install ROS, RViz, Gazebo and nav2. [.devcontainer/Dockerfile](.devcontainer/Dockerfile) can alternatively be used.

```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings


sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop ros-dev-tools ros-humble-slam-toolbox ros-humble-twist-mux
sudo apt install ros-humble-nav2*
```

After successful installation you need to set up the ROS environment, which can be done by running `source /opt/ros/humble/setup.bash`. Doing this every time you launch a new terminal instance can be annoying, so we recommend adding it to your bash config so that the environment is sourced automatically on every new instance you open. Do this by running the following command in your terminal:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

On a *new* terminal make sure the following command prints out `humble` to ensure that ROS environment is set up correctly.

```bash
$ printenv ROS_DISTRO
humble
```

For a quick installation of Gazebo, use the following command command:

```bash
sudo apt install ros-humble-gazebo*
```

After successfully installation and setup you should be able to see this:

```bash
$ gazebo -v
Gazebo multi-robot simulator, version 11.10.2
```

### Development environment
To improve collaboration in development environment we use vscode and docker as explained in [this instruction](https://www.allisonthackston.com/articles/docker-development.html) using these [docker files](https://github.com/athackst/dockerfiles).

To build and open the container in vscode: `Ctrl + Shift + P` and select `Dev containers: Rebuild and Reopen in container`

If you have any issue with the dockerfile, update `.devcontainer/Dockerfile`
```
-FROM nvidia/cuda:11.8.0-runtime-ubuntu22.04 AS base
+FROM ubuntu:22.04 AS base
```

#### Nvidia GPU

To build the docker files with nvidia support, update `.devcontainer/devcontainer.json`  from `"service": "smile_simulation"` to `"service": "smile_simulation_nvidia"`

To test the docker files : `docker compose up --build`

#### Python Packages 
For communicating with ROS2 we use `rclpy` package. Make sure that you source your environment if `rclpy` is missing. For other packages use pip to install the right package as below:

```bash
pip install -r requirements.txt
```
### to build and run the simulation

```bash
./build.sh
```

2. Spawn Aruco and Camera on the scene:

```bash
./start all
```

The jackal can then be controlled with the computer keyboard by running
```bash
ros2 launch dyno_jackal_bringup keyboard_steering.launch.py
```


### Additional information

Please see [LICENSE](LINCESE),  [CREDITS.md](CREDITS.md) and [CHANGELOG.md](CHANGELOG.md) for more information.