FROM nvidia/opengl:1.2-glvnd-runtime-ubuntu22.04

USER root
ENV user=px4space
ENV home=/home/$user
WORKDIR $home
ENV ROS_DISTRO=humble

RUN apt-get update -y && apt-get upgrade -y &&  apt-get install -y bash

## Install ROS2
RUN apt-get update -y && apt-get install -y curl tzdata \
    && ln -fs /usr/share/zoneinfo/Etc/UTC /etc/localtime \
    && dpkg-reconfigure -f noninteractive tzdata \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update -y \
    && apt-get upgrade -y \
    && apt-get install ros-humble-desktop -y

RUN echo "source /opt/ros/humble/setup.bash" >> $home/.bashrc
RUN ["/bin/bash", "-c", "source $home/.bashrc"]

## Install Gazebo
RUN apt update && apt install -y python3-pip lsb-release gnupg curl git \
    && pip3 install vcstool \
    && pip3 install -U colcon-common-extensions \
    && pip3 show vcstool | grep Location \
    && pip3 show colcon-common-extensions | grep Location \
    && sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros2-latest.list' \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && apt-get update -y \
    && apt-get install -y python3-vcstool python3-colcon-common-extensions \
    && mkdir -p $home/Gazebo/src \
    && cd $home/Gazebo/src \
    && curl -O https://raw.githubusercontent.com/gazebo-tooling/gazebodistro/master/collection-garden.yaml \
    && sed -i "s|gz-sim7|pr-spacecraft-thrusters|" collection-garden.yaml \
    && sed -i 's|https://github.com/gazebosim/gz-sim|https://github.com/DISCOWER/gz-sim.git|' collection-garden.yaml \
    && vcs import < collection-garden.yaml \
    && curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update \
    && apt-get install -y \
    $(sort -u $(find . -iname 'packages-'$(lsb_release -cs)'.apt' -o -iname 'packages.apt' | grep -v '/\.git/') | sed '/gz\|sdf/d' | tr '\n' ' ') && \
    cd $home/Gazebo && \
    colcon build --merge-install && \
    echo "source $home/Gazebo/install/setup.bash" >> $home/.bashrc


RUN apt-get update -y && apt-get install -y git python3-rosdep && \
    rosdep fix-permissions && rosdep init && rosdep update && \
    apt-get install -y ros-${ROS_DISTRO}-ament-cmake && \
    apt-get install -y python3-colcon-common-extensions && \
    mkdir -p $HOME/microros_ws/ && cd $HOME/microros_ws/ && \
    git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y && \
    apt-get install -y python3-pip && \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build && \
    . install/setup.sh && \
    ros2 run micro_ros_setup create_firmware_ws.sh host && \
    ros2 run micro_ros_setup build_firmware.sh && \
    . install/setup.sh && \
    ros2 run micro_ros_setup create_agent_ws.sh && \
    ros2 run micro_ros_setup build_agent.sh && \
    . install/local_setup.sh && \
    echo "source $HOME/microros_ws/install/local_setup.bash" >> $HOME/.bashrc

## QGround Control
# Install qt version 6.6.3
RUN pip install aqtinstall && \
    aqt install-qt --outputdir $home/qt linux desktop 6.6.3 gcc_64 -m qtcharts qtconnectivity qtlocation qtmultimedia qtpositioning qtsensors qtserialport qtspeech qtshadertools qt5compat qtquick3d

ENV PATH=$home/qt/6.6.3/gcc_64/bin:$PATH
ENV LD_LIBRARY_PATH=$home/qt/6.6.3/gcc_64/lib:$LD_LIBRARY_PATH
ENV QT_PLUGIN_PATH=$home/qt/6.6.3/gcc_64/lib/pkgconfig:$PKG_CONFIG_PATH

RUN mkdir -p $home/QGroundControl && \
    cd $home/QGroundControl && \
    git clone --recursive --depth=1 https://github.com/DISCOWER/qgroundcontrol.git && \
    bash ./qgroundcontrol/tools/setup/install-dependencies-debian.sh && \
    cd qgroundcontrol && \
    cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug && \
    cmake --build build --config Debug && \
    pwd && \
    cp ./build/QGroundControl $home/QGroundControlApp

## PX4 messages compile
RUN mkdir -p $home/PX4/ros2_ws/src && \
    cd $home/PX4/ros2_ws/src && \
    git clone --recursive --depth=1 https://github.com/PX4/px4_msgs.git && \
    cd $home/PX4/ros2_ws && \
    . /opt/ros/humble/setup.sh && colcon build --symlink-install && \
    echo "source $home/PX4/ros2_ws/install/setup.bash" >> $home/.bashrc

RUN apt-get update && apt-get install -y sudo wget

## PX4 related stuff
RUN mkdir -p $home/PX4 && \
    cd $home/PX4 && \
    git clone --recursive https://github.com/SpaceBotsISR/PX4-Space-Systems.git && \
    ./PX4-Space-Systems/Tools/setup/ubuntu.sh -y

ENV XDG_RUNTIME_DIR=/tmp/runtime-root

RUN apt-get update && apt-get install -y \
  mesa-utils \
  libgl1-mesa-dri \
  nvidia-utils-460

# Install necessary Mesa libraries
RUN apt-get update && apt-get install -y \
  mesa-utils \
  libegl1-mesa \
  libgles2-mesa \
  libglx-mesa0 \
  libglx0 \
  nvidia-utils-460 \
  libnvidia-gl-470  # Or the matching version of your driver

# Set runtime directory environment variable
ENV XDG_RUNTIME_DIR=/tmp/runtime-root


# Create a user and group with specific IDs
# Create a user and group with specific IDs
RUN groupadd -r px4space && useradd -r -g px4space px4space

# Set the user to own the home directory (optional)
RUN chown -R px4space:px4space /home/px4space

# Switch to non-root user
USER px4space
