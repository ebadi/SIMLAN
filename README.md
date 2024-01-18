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

## Installation

These steps are used to install ROS (Humble) and Gazebo (11.10.2) on
  - Ubuntu 22.04
  - Windows 10 (after installing "ubuntu 22.04.3 LTS" WSL2 from Microsoft Store).

### ROS, RViz

We follow [the installation instruction](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) to install ROS, RViz, Gazebo and nav2s.

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

Open a *new* terminal make sure the following command prints out `humble` to ensure that ROS environment is set up correctly.

```bash
$ printenv ROS_DISTRO
humble
```

### Gazebo

For a quick installation of Gazebo, use the following command command:

```bash
sudo apt install ros-humble-gazebo*
```

After successfully installation and setup you should be able to see this:

```bash
$ gazebo -v
Gazebo multi-robot simulator, version 11.10.2
```

### Additional information

Please see [LICENSE](LINCESE),  [CREDITS.md](CREDITS.md) and [CHANGELOG.md](CHANGELOG.md) for more information.