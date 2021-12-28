# Notes

Dockers are similiar to virtual machines, but lighter weight. This will allow for your computer to be running the newest version of linux and still run ROS melodic in an Ubuntu 18.04 enviroment.
Another benefit is you can create an image per project that has all of the reqirements for that project installed and give it to someone else to run.

Guide:

* Follow install instructions here: https://docs.docker.com/engine/install/ubuntu/
* If you are running a Nvidia GPU and you want gui support follow these instructions to install nvidia_docker2: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html
  * The first step has you installing docker skip that part but do run: 
    ```console
    sudo systemctl --now enable docker
    ```
 * To get a base image of ubuntu 18.04 with ros melodic full desktop installed: (There may already be a based image for the project you're on)
    *  Run everytime booting up computer: (This allows docker enviroments to display images) 
       ```console 
       xhost +local:docker &> /dev/null
       ```
    * To create the docker container run the following:
      ```console
      DOCKER_COMMON_ARGS="--gpus all --env=NVIDIA_VISIBLE_DEVICES=all --env=NVIDIA_DRIVER_CAPABILITIES=all --env=DISPLAY --env=QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:rw"
      ```
      (This will open a bash window in the container, also it will install and setup the ROS workspace for the infrastructure project)
      ```console
      sudo docker run -it --net=host --privileged $DOCKER_COMMON_ARGS --name <name_of_container> grimmlins/infrastructure:base_image
      ```
     * In a new terminal run:
       ```console
       sudo docker exec -it <name_of_container> bash
       ```

## How to get into docker container after the computer is turned on:
* If you want guis(run in a host os terminal not the terminal for the container):
  ```console
  xhost +local:docker &> /dev/null
  ```
* Start the container: 
  ```console
  sudo docker start <name_of_container>
  ```
* Access the bash of the container:
  ```console
  sudo docker exec -it <name_of_container> bash
  ```
  
## Add user to docker group(removes the need for sudo when doing docker commands)
* https://docs.docker.com/engine/install/linux-postinstall/


## Useful commands:
* See running containers: 
  ```console
  sudo docker ps
  ```
* See all containers on computer: 
  ```console
  sudo docker container list -a
  ```
* Stop the container once you are done:
  ```console
  sudo docker stop <name_of_container>
  ```

## Useful Tips:
* When working with a usb device the easiest method is to give the container --privileged access when doing the docker run command. (This method does have security risks) Ensure to plug in the device before executing the "docker start <container name>".
  * If the container is already running use the stop command plug in usb device then execute the start command.

## References for this Guide:

* https://github.com/koenlek/docker_ros_nvidia
* https://docs.docker.com/engine/install/ubuntu/
* https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html
