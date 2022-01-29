# infrastructure-packages with Docker Containers
#### All the Needed packages for the testing Infrastructure (Updated Raspberry Pi Version)

## Prequisite:
- Be able to read and understand instructions
- Install docker and enable GUI support by following instructions in [docker_setup.md](https://github.com/OSUrobotics/infrastructure-packages/blob/new_file_structure/docker_setup.md).
- Install catkin tools: https://catkin-tools.readthedocs.io/en/latest/installing.html

## Setup:

1. Build an image with the infrastructure Dockerfile
    ```console
    docker build -t infrastructure-packages:grimmlins https://github.com/OSUrobotics/lab_Dockerfiles.git#master:infrastructure_base
    ```
2. Create and start the docker container from the image
   ```console
   DOCKER_COMMON_ARGS="--gpus all --env=NVIDIA_VISIBLE_DEVICES=all --env=NVIDIA_DRIVER_CAPABILITIES=all --env=DISPLAY --env=QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:rw"
   sudo docker run -it -d --net=host --privileged $DOCKER_COMMON_ARGS --name infra_env infrastructure-packages:grimmlins
   ```
3. This will have started a detached container called "infra_env". You can check if it is running with:
    ```console
    docker ps
    ```
    Attach the container to a desired terminal: (or use your favorite docker extension, like [visual studio codes'](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker) to access the container)
    ```console
    docker attach infra_env
    ```
4. Inside of the container, source the ROS workspace:
    ```console
    cd ~/infrastructure_ws/
    source devel/setup.bash
    ```

## Package Overview (outdated)

### data_collection

#### Current State:
The main purpose of this package is to handle data collection for video recording and some rosbag behavior if it needs to. Uses OpenCV to record through a camera when the action server callback is triggered. Stops when another action server callback is triggered. Handles naming of files by using the parameter server variables set in the launch file.

#### TODO/Future Updates:
- This package should eventually end up on a Raspberry Pi where it will be mapped to a network drive for data storage. 
- Defining location of video and rosbag storage will need to be updated to reflect this.
- Publishes feedback messages for when the camera starts recording and stops recording, eventually we may want a more elegant solution.

### infrastructure_msgs

#### Current State:
Contains all necessary custom ros messages for the system. All future sensor messages should be defined here.

#### TODO/Future Updates:
- Eventually will need more sensor messages as needed for an individual testbed, try to follow the naming scheme being used as best you can so it is consistent.

### infrastructure_raspi

#### Current State:
Contains the nodes that will run on the raspberry pi and talk to the hardware. Right now there are 2 nodes, test_parameters and reset. The reset node is meant to be used as an outline for all future testbed reset nodes, put all hardware related calls within the callback of the action server. The test_parameter node is meant to recieve any pre test parameters for the physical hardware sent from flexbe and should be treated as a blueprint like reset. The test_parameter message type is an array of floats that will correspond to certain settings depending on the testbed.

#### TODO/Future Updates:
= Once the tesbeds are in a semi permanent location it would be a good idea write a script and new launch file to automatically connect the testbed Pi's and the desktop so you dont have to go through a long process each time.
- Once the hardware and raspberry pi connection has been implemented the next step would be to implement a way to modify test parameters from flexbe or whatever portal we end up using. In its current state you have to use the same parameters for every test, a simple solution would be to use a csv file to hold parameters for each trial and read from that. Eventually we will need a permanent solution.
- When Pi hardware implementation is complete add launch file arguments for given testbeds so that it knows which nodes to launch.
- Feedback examples have been provided in both blueprint.

### arm_control

#### Current State:
This is just a placeholder until a more permanent solution can be found. There is a node inside which has a simple action server, the idea is that any arm related tasks should be done within callback. One method would be to right your own seperate moveit class and call it within the callback. 

#### TODO/Future Updates:
- Once we figure out how end users will upload arm code this will need a complete overhaul. Treat this as a temporary solution

### infrastructure_behaviors

#### Current State:
This contains all the necessary Flexbe states and behaviors (as well as a bunch of simple sample states you can use). It now only contains 1 behavior, System_Behaviour_Pi, the hope is that this is the only behavior we will ever need for every testbed, the only thing that will change between them is the hardware nodes in the infrastructure_raspi package. The necessary topic names are preset but you can change them if needed.  

#### TODO/Future Updates:
- Once we have hardware to test on we may need to make changes depending on how everything fits together.

## How to use:
### Interfacing with an arm
Place all path planning and moveit commands in the "start_arm_sequence" action server located in arm_control. (see example_arm_controller.py)

### Connecting to an apparatus
To be able to use one of the apparatuses, you must set up ROS communication between the main machine and the raspberry PI for the desired apparatus. 

1. SSH into the PI for the desired apparatus
    - Make sure PI is connected to same network as the PC via ethernet or wifi
    - Password for all apparatuses: *Password* 
    - testbed:
        ```console
        ssh ubuntu@raspi-testbed.local
        ```
    - door:
        ```console
        ssh ubuntu@raspi-door.local
        ```
    - drawer:
        ```console
        ssh ubuntu@raspi-drawer.local
        ```
3. On the master machine (PC):
    ```console
    export ROS_HOSTNAME=<PC-hostname>.local
    export ROS_MASTER_URI=http://<PC-hostname>.local:11311
    ```
    example:
    ```
    tesbed@testbed-tower:~$ export ROS_HOSTNAME=testbed-tower.local
    tesbed@testbed-tower:~$ export ROS_MASTER_URI=http://testbed-tower.local:11311
    ```
4. On the listener machine (PI):
    ```console
    export ROS_HOSTNAME=<PI-hostname>.local
    export ROS_MASTER_URI=http://<PC-hostname>.local:11311
    ```
    example:
    ```
    ubuntu@raspi-testbed:~$ export ROS_HOSTNAME=raspi-testbed.local
    ubuntu@raspi-testbed:~$ export ROS_MASTER_URI=http://testbed-tower.local:11311
    ```
5. (optional) Check that communication is working:
    - Start a roscore on the PC:
        ```console
        roscore
        ```
    - list rostopics on the PI:
        ```console
        rostopic list
        ```
    - You should see these topics listed on the PI:
        ```
        /rosout
        /rosout_agg
        ```
    - If you do not see those topics or PI says it was unable to connect to master:
        - Make sure that roscore is running on the PC before trying to list rostopics on the PI
        - Check that the PI and PC have the same ROS_MASTER_URI
    - Kill the roscore once finished (ctrl-c)

### Setting up remote home (for sensor data from door/drawer)
ADD
    
### Launching 
On the PC:
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
roslaunch infrastructure_raspi start_raspi_nodes.launch apparatus:=<string>
```
valid apparatus names:
```
testbed
drawer
door
```

### FlexBE
After running the launch file, FlexBe will pop up:

#### General Test
Load Behavior->System_Behaviour_Pi->Runtime Control->Insert full path to csv file containing the session parameters->Start Execution

##### Proper formatting of CSV file
```
testbed
object_index, angle, num_trials
... (repeat for each test)

OR

drawer/door
resistance_value, num_trials
... (repeat for each test)
```

### You're ready to go!
