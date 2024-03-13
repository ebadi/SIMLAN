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

## Docker container
Need a brushup on docker, see this tutorial.

### Install and start
Another option for installing is to run the environment in a Docker container, to avoid installing everything on your own computer. Check [docker install guide](https://docs.docker.com/engine/install/) to install docker on your machine.

Developing in a docker container can easily be done by using VS Code, and installing the _Remote Development extension_.

Once all this is done, open the repository in VS Code. Hit ctr + shift + p and select _Dev Containers: Rebuild in container_. This will build the container and open VS Code inside of the container, thanks to the [docker-compose](docker-compose.yaml)-file. Your terminal inside VS Code is then running inside the Docker container. 

### Developing inside the container
Below are some features than can assist you in developing inside the container.

#### Building ros2 packages
There is also a [tasks.json](.vscode/tasks.json) file where some build rules are stated. Building can be done by ctrl + shift + b. 

#### Dependencies and other repos
This development environment has a few nice features. There are two files: [requirements](requirements.txt) and [ros_dependencies](ros_dependencies.repos), where python3 (pip install) and Github repos can be listed to be installed and cloned into the container.

The repos installed in the [ros_dependencies](ros_dependencies.repos) will end up in _/opt/dependencies_ws/src/_ in the container. They can be copied into the working directory if changes to these repos are needed. For example copying it into the simulation folder.
```bash
cp -r /opt/dependencies_ws/src/<dir_name> simulation/
```
The package in the working directory will be prioritized over the one in _/opt/dependencies_ws/_, and will be build together with the rest of your ros2 packages. Note: if these packages are big, the building time can increase.

To avoid uploading the package to git adjust your [gitignore](.gitignore) accordingly. Once you are done making changes to the imported repo, you need to make sure others get the changes as well. Easiest is often to fork the repo needed and commit and push your changes to it. Then change the _url_ in the [ros_dependencies](ros_dependencies.repos)-file to import from your fork instead. Then remove the folder from your workspace and rebuild the container and make sure it works.

Think about if the change is really needed before going through this somewhat lengthy process. Launch and config files can often be made inside a a bringup package instead of places inside the imported repo.

## Additional information

Please see [LICENSE](LINCESE),  [CREDITS.md](CREDITS.md) and [CHANGELOG.md](CHANGELOG.md) for more information.