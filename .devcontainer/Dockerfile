
FROM althack/ros2:humble-cuda-gazebo-nvidia

ENV ROS_DISTRO=humble
ENV AMENT_PREFIX_PATH=/opt/ros/humble
ENV COLCON_PREFIX_PATH=/opt/ros/humble
ENV LD_LIBRARY_PATH=/opt/ros/humble/lib
ENV PATH=/opt/ros/humble/bin:$PATH
ENV PYTHONPATH=/opt/ros/humble/lib/python3.8/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2

ENV DEBIAN_FRONTEND=dialog
# Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute
ENV QT_X11_NO_MITSHM=1

# ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
  # opengl stuff
  libgl1-mesa-glx \
  libgl1-mesa-dri \
  x11-apps \
  mesa-utils \
  libglvnd0 \
  libgl1 \
  libglx0 \
  libegl1 \
  libxext6 \
  libx11-6 \
  # other stuff
  bash-completion \
  build-essential \
  cmake \
  gdb \
  git \
  tig \
  # pylint3 \
  python3-argcomplete \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  python3-vcstool \
  vim \
  wget \
  gdbserver \
  htop \
  xterm \
  # Install ros distro testing packages
  ros-humble-ament-lint \
  ros-humble-launch-testing \
  ros-humble-launch-testing-ament-cmake \
  ros-humble-launch-testing-ros \
  python3-autopep8 \
  && rm -rf /var/lib/apt/lists/* \
  && rosdep init || echo "rosdep already initialized"
  
ARG USERNAME=ros
ARG USER_UID=1453
ARG USER_GID=$USER_UID

# # Create a non-root user
# RUN groupadd --gid $USER_GID $USERNAME \
#   && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
#   # [Optional] Add sudo support for the non-root user
#   && apt-get update \
#   && apt-get install -y sudo \
#   && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
#   && chmod 0440 /etc/sudoers.d/$USERNAME \
#   # Cleanup
#   && rm -rf /var/lib/apt/lists/* \
#   && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc \
#   && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc

ARG USERNAME=ros
ARG WORKSPACE=/home/${USERNAME}

ENV DEPENDENCIES_WS /opt/dependencies_ws
RUN mkdir -p $DEPENDENCIES_WS/src

ENV HOME=/home/${USERNAME}
ENV WS=${WORKSPACE}

ARG CMAKE_BUILD_TYPE=RelWithDebInfo

# Install palanteer
RUN apt-get update && apt-get install -y \ 
    libdw-dev libunwind-dev     libgl1-mesa-glx libgl1-mesa-dri mesa-utils     && rm -rf /var/likb/apt/lists/*

RUN cd /opt && git clone https://github.com/dfeneyrou/palanteer && \
    cd palanteer && \
    mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && \
    make install

# Install supl dependencies
RUN apt-get update && \
    apt-get install -y -qq --no-install-recommends \
    gcc cmake libssl-dev make ninja-build ranger \
    && rm -rf /var/lib/apt/lists/*

# Build ROS 2 source dependencies
WORKDIR $DEPENDENCIES_WS
COPY ros_dependencies.repos ros_dependencies.repos
RUN vcs import src < ros_dependencies.repos
RUN apt-get update && \
    rosdep update && \
    rosdep install -q -y --from-paths src --ignore-src && \
    rm -rf /var/likb/apt/lists/*

RUN . /opt/ros/humble/setup.sh && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE

# Install PIP requirements
COPY requirements.txt /tmp/
RUN python3 -m pip install -r /tmp/requirements.txt

ENV DEBIAN_FRONTEND=dialog

# Create source folder
WORKDIR $WS

# Install ros dependencies
COPY --chown=$USERNAME . src

RUN apt-get update && \
    . $DEPENDENCIES_WS/install/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y && \
    rm -rf /var/lib/apt/lists/*

USER $USERNAME

RUN sudo chown -R ros .ros

# Build ros
RUN /bin/bash -c 'source $DEPENDENCIES_WS/install/setup.bash && cd /home/ros && colcon build --merge-install --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE'

RUN echo "if [ -f /home/ros/install/setup.bash ]; then source /home/ros/install/setup.bash; fi" >> /home/ros/.bashrc
RUN echo "if [ -f /usr/share/gazebo/setup.sh ]; then . /usr/share/gazebo/setup.sh; fi" >> /home/ros/.bashrc
RUN echo "export RCUTILS_COLORIZED_OUTPUT=1" >> /home/ros/.bashrc

