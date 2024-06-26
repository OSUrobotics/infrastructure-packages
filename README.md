roscore
rosparam load /root/infrastructure_ws/src/infrastructure-packages/infrastructure_behaviors/infrastructure_flexbe_behaviors/cfg/gen3_moveit_params.yaml
roslaunch infrastructure_flexbe_behaviors start_gen3_test_gpd.launch 
rosrun dataset_test xyz_poses.py 

On pi:
roslaunch infrastructure_raspi start_raspi_nodes.launch

# infrastructure-packages with Docker Containers
#### All the Needed packages for the testing Infrastructure (Updated Raspberry Pi Version)

## Prequisite:
- Install docker and enable GUI support by following instructions in [docker_setup.md](https://github.com/OSUrobotics/infrastructure-packages/blob/new_file_structure/docker_setup.md). Follow all instructions listed, except it is not necessary to create a base image of ubuntu 18.0.4.

## Setup:

- Notes: 
    - if you added the user to the docker group, it is not necessary to run the following commands with sudo
    - If the docker container is already made on the machine, start the container and skip to step 3.

1. Build an image with the infrastructure Dockerfile
    ```console
    sudo docker build --no-cache -t infrastructure-packages:grimmlins https://github.com/OSUrobotics/Grimmlins_Dockerfiles.git#main:dockerfiles/infrastructure_base
    ```
2. Create and start the docker container from the image
   ```console
   DOCKER_COMMON_ARGS="--gpus all --env=NVIDIA_VISIBLE_DEVICES=all --env=NVIDIA_DRIVER_CAPABILITIES=all --env=DISPLAY --env=QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:rw"
   ```
   ```console,
   sudo docker run -it -d --net=host --privileged $DOCKER_COMMON_ARGS --name infra_env infrastructure-packages:grimmlins
   ```
2. If the above doesn't work, try:
    ```console,
    sudo docker run -it -d --net=host --privileged $DOCKER_COMMON_ARGS --env=DISPLAY --env=NVIDIA_VISIBLE_DEVICES=all --env=NVIDIA_DRIVER_CAPABILITIES=all --env=QT_X11_NO_MITSHM=1 --runtime=nvidia -v /tmp/.X11-unix:/tmp/.X11-unix --name infra_env_arm_v0 infrastructure-packages:grimmlins
    ```
3. This will have started a detached container called "infra_env". You can check if it is running with:
    ```console
    sudo docker ps
    ```
   If the docker container gets stopped for any reason, you can start it again with:
    ```console
    sudo docker start infra_env
    ```
4. Attach the container to a desired terminal: (or use your favorite docker extension, like [visual studio codes'](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker) to access the container)
    ```console
    sudo docker attach infra_env
    ```
5. **Change Branches in submodules** to the desired branches. 
    To check which branch each submodule is in, use this command in the respective submodule:
    ```console
    git branch
    ```
    The _infrastructure-arms_ directory contains branches for the arm controller node as well as any packages for controlling the robotic arm. See [the readme](https://github.com/OSUrobotics/infrastructure-arms/blob/main/README.md) on how to use this repository.
    Currenty available branches for _infrastructure-arms_:
    - main (can be used for testing)
    - Kinova_j2s7s300 (custom packages for Kinova arm)
    - Example (sets the branch to the testing _main_ branch):
        ```console
        cd ~/infrastructure_ws/src/infrastructure-packages/infrastructure-arms
        git checkout main
        ```
    The _infrastructure-raspi_ directory contains the packages and source code for interfacing and controlling each apparatus. Within the docker container, the branch for this directory should just be in _main_. The _infrastructure_msgs_ submodule should also be loaded. If any of these requirements are not met, run the corresponding command below: 
    - Switch _infrastructure-raspi_ to _main_ branch:
        ```console
        cd ~/infrastructure_ws/src/infrastructure-packages/infrastructure-raspi
        git checkout main
        ```
    - Load the _infrastructure_msgs_ submodule:
        ```console
        cd ~/infrastructure_ws/src/infrastructure-packages/infrastructure-raspi
        git submodule update --init
        ```
 6. After all changes have been made, rebuild and source workspace in container:
    ```console
    cd ~/infrastructure_ws
    catkin_make
    source devel/setup.bash
    ```
    
    
## How to use:
### Interfacing with an arm
Place all path planning and moveit commands in the "start_arm_sequence" action server located in arm_control. (see [example_arm_controller.py](https://github.com/OSUrobotics/infrastructure-arms/blob/main/arm_control/src/example_arm_controller.py))

Connect USB from the arm into the host machine BEFORE launching the "infra_env" container. If container is already started, you must stop it and then restart it again after the USB is connected for the arm.

All supported arms have setup documentation in the [infrastructure-arms](https://github.com/OSUrobotics/infrastructure-arms/tree/main) repository within their respective branches.

### Connecting to an apparatus
To be able to use one of the apparatuses, you must set up ROS communication between the main machine (within docker container) and the raspberry PI for the desired apparatus. 

1. SSH into the PI for the desired apparatus
    - Make sure PI is connected to same network as the PC via ethernet or wifi
    - testbed:
        ```console
        ssh ubuntu@testbed-osu.local
        ```
    - door:
        ```console
        ssh ubuntu@door-osu.local
        ```
    - drawer:
        ```console
        ssh ubuntu@drawer-osu.local
        ```
2. Make sure you are using the correct workspace and branches on PI (git checkout in infrastructure_raspi), then:
        ```console
        cd ~/infrastructure_system
        catkin build
        source devel/setup.bash
        ```
3. On the master machine (Container):
    ```console
    export ROS_HOSTNAME=<PC-HOSTNAME>
    export ROS_MASTER_URI=http://<HOSTNAME>:11311
    ```
4. On the listener machine (PI):
    ```console
    export ROS_HOSTNAME=<PI-HOSTNAME>
    ROS_MASTER_URI=http://<HOSTNAME>:11311 # same as master machine
    ```

5. Make sure on both devices to edit the hostname file located at /etc/hosts

6. (optional) Check that communication is working:
    - Start a roscore on the container:
        ```console
        roscore
        ```
    - In another terminal in the container, repeat step 3, then run:
        ```console
        rosrun rospy_tutorials listener.py
        ```
    - On the PI:
        ```console
        rosrun rospy_tutorials talker.py
        ```
    - You should see messages appear on both the PI and container
    - Then try running listener.py on the PI and talker.py in the container. You should observe similar behavior
    - If you do not see messages appear both on the PI and in the container for BOTH tests:
        - Make sure that roscore is running on the container before trying to list rostopics on the PI
        - Check that the PI and PC have the same ROS_MASTER_URI
        - To check that the environment variable ROS_MASTER_URI is set properly in the current terminal:
            ```console
            printenv ROS_MASTER_URI
            ```
    - Kill the roscore once finished (Ctrl-C)
    
### Launching

If not done already, make sure gui support is enabled for docker containers on the master machine:
```console
xhost +local:docker &> /dev/null # run in a host terminal, not container!
```
__Note:__ this needs to be done for every new session on the PC. If you don't want to do this evertime, then copy this command into the .bashrc file for your host machine located in the home directory.

For the door/drawer and basic run/testing, launch the example test file:
```console
roslaunch infrastructure_flexbe_behaviors start_test.launch 
```
Optional launch parameters (none are needed for testbed):
```
collect_data:=true (This activates a rosbag that records all topics with the suffix "_infsensor", stored in data collection package. Defaults to false)

name:=<string> (name given to rosbags, csv files, and videos)
  
video:=true (activates recording for a camera connected to the main PC. Defaults to false)
```
On the PI:
    ```console
    roslaunch infrastructure_raspi start_raspi_nodes.launch
    ```

For launching the Grasp Reset w/ the Kinova Gen 3 and GPD:
```console
roslaunch armada_flexbe_utilities gen3_pick_and_place.launch 
```
For additionally including the interface:
```console
roslaunch RemasteredInterface main_page.launch
```


### FlexBE
After running the launch file, FlexBe will pop up:

#### General Test
For Grasp Reset with Gen 3 and GPD run: Load Behavior->Grasp_Reset_GPD_Behavior_Kinova_Gen_3
For Door/Drawer run: Load Behavior->Door_Drawer_Behavior

Updated "Session Info" with path to csv file with trial info. 

##### Proper formatting of CSV file:
```
testbed
object_index, angle, num_trials
... (repeat for each test)
```

OR

```
drawer/door
resistance_value, num_trials
... (repeat for each test)



### You're ready to go!
