# infrastructure-packages
#### All the Needed packages for the testing Infrastructure (Updated Raspberry Pi Version)


## Setup:

0. Install catkin tools if you havent already: https://catkin-tools.readthedocs.io/en/latest/installing.html

1. Create a ROS workspace and name it infrastructure system 
    ```console
    mkdir -p ~/infrastructure_system/src
    cd ~/infrastructure_system/
    catkin build
    ```
2. Install FlexBE Binaries and clone the app to your src folder
    ```console
    sudo apt install ros-$ROS_DISTRO-flexbe-behavior-engine
    git clone https://github.com/FlexBE/flexbe_app.git 
    ```
3. Clone this repository into your src folder

4. Build workspace again
    ```console
    catkin build
    ```

## Package Overview

### data_collection

#### Current State:
The main purpose of this package is to handle data collection for video recording and some rosbag behavior if it needs to. Uses OpenCV to record through a camera when the action server callback is triggered. Should be replaced with ros video capturing capabilities. Stops when another action server callback is triggered. Handles naming of files by using the parameter server variables set in the launch file.

#### TODO/Future Updates:
- This package should eventually end up on a Raspberry Pi where it will be mapped to a network drive for data storage.
- Defining location of video and rosbag storage will need to be updated to reflect this.
- Publishes feedback messages for when the camera starts recording and stops recording, eventually we may want a more elegant solution.

### infrastructure_msgs

#### Current State:
Contains all necessary custom ros messages for the system. All future sensor messages should be defined here.

### infrastructure_raspi

#### Current State:
Contains the nodes that run on the raspberry pi and talk to the hardware. Each apparatus has it's own controller that interacts with the source code for that apparatus.

### arm_control

#### Current State:
This is just a placeholder until a more permanent solution can be found. There is a node inside which has a simple action server, the idea is that any arm related tasks should be done within callback. One method would be to right your own seperate moveit class and call it within the callback. 

#### TODO/Future Updates:
- Once we figure out how end users will upload arm code this will need a complete overhaul. Treat this as a temporary solution

### infrastructure_behaviors

#### Current State:
This contains all the necessary Flexbe states and behaviors (as well as a bunch of simple sample states you can use). It now only contains 1 behavior, System_Behaviour_Pi, the hope is that this is the only behavior we will ever need for every testbed, the only thing that will change between them is the hardware nodes in the infrastructure_raspi package. The necessary topic names are preset but you can change them if needed.  

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
1. Install ssh server if not already installed on the PC:
    ```console
    sudo apt-get install openssh-server
    ```
2. Install sshfs if not already installed on the PI:
    ```console
    sudo apt-get install sshfs
    ```
3. If not already a ~/remhome dir on the PI, create one:
    ```console
    mkdir ~/remhome
    ```
4. Mount remote share folder on the PI:
    ```console
    sshfs <ROS_HOSTNAME> ~/remhome
    ```
5. That's all! If needed, the remote folder on the PI can be unmounted with:
    ```console
    fusermount -u ~/remhome
    ```


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
```
- IMPORTANT: you must include the header line _testbed_ or _drawer/door_
- See user_input_example.csv for an example

### You're ready to go!
