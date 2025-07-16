Installation instructions for the DISCOWER sitl docker image!

# Pre-requisites
## First we install the nvidia container toolkit for gpu usage with docker
https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html

## Then we obtain the default kasm docker image from here: https://hub.docker.com/r/kasmweb/ubuntu-jammy-desktop/tags
docker pull kasmweb/core-jammy-desktop:1.14.0-rolling




# Build the docker
`docker build -t ff_ros2_DISCOWER -f docker_file_kasm_ubuntu_jammy_DISCOWER .`

now change the id that you get in 'docker image list' (IMAGE ID) in the run_kasm_ros.sh file (line 4)

## Run the setup file
./run_kasm_ros.sh

## GUI support
open `https://127.0.0.1:10334` in your browser

## Stop and Start the docker image
You shouldn't have to run the `.sh` script again
`docker stop ff_ros2_DISCOWER_container`
`docker start ff_ros2_DISCOWER_container`
and go to your browser




# In the docker instance

## Run the PX4 sim

`cd ~/discower_gits/PX4-Space-Systems`
`make px4_sitl gz_spacecraft_2d`

## Run micro-ROS 

`ros2 run micro_ros_agent micro_ros_agent upd4 --port 8888`

Now you should see the the /fmu/ topics being published!

 
