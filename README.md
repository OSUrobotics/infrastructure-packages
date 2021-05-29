# infrastructure-packages
#### All the Needed packages for the testing Infrastructure (Updated Raspberry Pi Version)


## Setup:

0. Install catkin tools if you havent already: https://catkin-tools.readthedocs.io/en/latest/installing.html

1. Create a ROS workspace and name it infrastructure system 
```
mkdir -p ~/infrastructure_system/src
cd ~/infrastructure_system/
catkin build
```
2. Install FlexBE Binaries and clone the app to your src folder
```
sudo apt install ros-$ROS_DISTRO-flexbe-behavior-engine
git clone https://github.com/FlexBE/flexbe_app.git 
```
3. Clone this repository into your src folder

### If you arent using hardware you can stop here otherwise:

4. Documentation here: https://docs.google.com/document/d/1FJUptD0hqH2z4wYqg2FA9XU88-Y16Aj19yGCMj52ljQ/edit?usp=sharing

## Package Overview

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
You will need to create your own Moveit class that you can use within the action server located in arm_control. Its still very much under developement so Im not gonna go into too much detail until we do further testing.
    
### Launching 
```
roslaunch infrastructure_flexbe_behaviors start_test.launch 

```
If you are planning to use the hardware be sure to follow the document at the beginning of the README, you must also use the hardware:=true flag when running the above launch file. In addition on the Pi you must run the launch file:
```
roslaunch infrastructure_raspi start_raspi_nodes.launch

```
This is a temporary solution until the testbeds are further along and everything can be condensed to one launch file.

This will launch FlexBe and all of the necesary nodes for a full trial. There are 4 arguments you can pass to the launch function for different behavior:
```
collect_data:=true (This activates a rosbag that records all topics with the suffix "_infsensor", stored in data collection package)

name:=<string> (This is used to state the name of the test that will be used as prefixes for data files. In the future this will also be used for determining test station nodes.)
  
video:=true (You can use this to change whether or not you want to record video)
  
hardware:=true (Should do more in the future but right now it just doesnt launch the raspi nodes so they can be run on remote hardware with another launch file.)
```
They automatically default to false so you have to explicitly state them if you wish to use any combination of these.


### FlexBE
After running the launch file FlexBe will pop up to use the testbed or door:

#### General Test
Load Behavior->System_Behaviour_Pi->Runtime Control->Change number of tests and trials to whatever youd like->Start Execution

This will eventually be the master Behavior that is used for every testbed. As mentioned above the new pi hardware is not yet supported and a couple features will need to be added once that is implemented.

### You're ready to go!
This is still very much under developement so there will be bugs and things will break. I most likely forgot something on here as well so if you need to use it with an arm or have any questions feel free to contact me at navek@oregonstate.edu. Good Luck!
