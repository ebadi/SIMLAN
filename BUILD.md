
To improve collaboration in development environment we use vscode and docker as explained in [this instruction](https://www.allisonthackston.com/articles/docker-development.html) using these [docker files](https://github.com/athackst/dockerfiles).

To build the docker files with nvidia support, update `.devcontainer/devcontainer.json`  from `"service": "smile_simulation"` to `"service": "smile_simulation_nvidia"`. 

### ROS, RViz, Gazebo

We follow installation procedure used in [.devcontainer/Dockerfile](.devcontainer/Dockerfile) to install dependencies in the production environment. 
Alternatively one can follow [the installation instruction](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) to install minimal packages to get ROS, RViz, Gazebo and nav2 working. 

These steps are used to install ROS (Humble) and Gazebo (11.10.2) on
  - Ubuntu 22.04
  - Windows 10 (after installing "ubuntu 22.04.3 LTS" WSL2 from Microsoft Store).
  

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

### Python Packages 
For communicating with ROS2 we use `rclpy` package. Make sure that you source your environment if `rclpy` is missing. For other packages use pip to install the right package as below:

```bash
pip install -r requirements.txt
```