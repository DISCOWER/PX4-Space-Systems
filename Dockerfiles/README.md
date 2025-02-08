# PX4 Space System docker container

## Overview
This docker file is used as a way to containerize the simulation environment of PX4 space systems. The simulation run in Ubuntu 22.04 container and uses the new gazebo garden as a simulation engine.

If it is intended to use with ROS (inside the container) you should use ROS2 humble.

## Software packed inside

Inside the container you have packed Gazebo Garden, ros2 humble, micro ros agent, px4 msgs, qt versioon 6.6.3 and QGroundControl (with custom plugin for gzç-sim7).

## Compiling

Compiling the docker is rather simple you must ensure that you have [docker installed](https://docs.docker.com/engine/install/), after this simply build the container, as you can see in the command bellow you can specify the link and the branch for the PX4 git, this allow for more modularity when using in fork of the main work. **Be aware that when changing the branch or link you will need to compile the full container, and this is a lengthy process**

```
docker build -t px4 --build-arg PX4_SOFTWARE_REPO=https://github.com/SpaceBotsISR/PX4-Space-Systems.git --build-arg PX4_SOFTWARE_BRANCH=pr-space_cobot .
```

In the computer where this was developed the build process toke around 20 minutes, and needed 16GB of RAM and 16GB of swap space and during the gazebo compilation process the full 16 cores were occupied with build the docker.

After compiling you now need to create the container, this can e done with 
```
docker run -it --gpus all \\n  --network=host --ipc=host \\n  -e DISPLAY=$DISPLAY \\n  -e NVIDIA_VISIBLE_DEVICES=all \\n  -e NVIDIA_DRIVER_CAPABILITIES=all \\n  -e QT_X11_NO_MITSHM=1 \\n  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \\n  -v /dev/dri:/dev/dri \\n  -e XDG_RUNTIME_DIR=/tmp/runtime-root \\n  --runtime=nvidia \\n  -v /home/andret/ISR:/home/px4space/ISR \\n  --name px4_cont px4
```

Most of the flags are used are to pass a gpu to docker and e able to use it for running the gazebo simulation, if you dont have a gpu, or is not a nvidia gpu i recomend removing the flags, since the base container is from nvidia in this docker. An extra folder is also passed to the docker, remove this if you dont want to have the need to it.

## Using the container

The current test done to the simulation enviomnent envolve
 - Gazebo and QGroundControl running inside the docker container
 - Gazebo, QGroundControl and ros2 nodes running inside the docker
 - Gazebo, micro-ros and ros2 nodes inside this docker, with QGroundControl in the host machine and more ros2 node in an external container 

When you open the docker this is the result of the ls command 
```
Documents  ISR	QGroundControl	   aqtinstall.log  qt
Gazebo	   PX4	QGroundControlApp  microros_ws	   ros2_ws
```

The QGroundControlApp is the execulable, and the simulation toll are accesible through PX4/PX4-Space-Systems