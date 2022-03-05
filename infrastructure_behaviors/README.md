# infrastructure_behaviors Packages
Includes two ROS packages:
- infrastructure_flexbe_behaviors
- infrastructure_flexbe_states

## infrastructure_flexbe_behaviors Package Overview
Contains generated source code and manifest for the flexbe behavior __System_Behaviour_Pi__. They are modified through the flexbe app and should not be directly modified.

Contains [main launch file](https://github.com/OSUrobotics/infrastructure-packages/blob/new_file_structure/infrastructure_behaviors/infrastructure_flexbe_behaviors/launch/start_test.launch) for the infrastructure system. Launches flexbe, rosbags if collect_data argument is set to True, [data_collection](https://github.com/OSUrobotics/infrastructure-packages/blob/new_file_structure/data_collection/src/data_collection.py) node, and the [example_arm_controller](https://github.com/OSUrobotics/infrastructure-arms/blob/main/arm_control/src/example_arm_controller.py) node found in the arm_control package. To learn how to use this launch file, see the main [readme](https://github.com/OSUrobotics/infrastructure-packages/blob/new_file_structure/README.md#launching) for infrastructure-packages.
- __Note:__ if example_arm_controller node is renamed, launch will fail.

### Rosbag Recording Overview
__rosbag_record_sensors__
- Records all topics with the suffix _\_infsensor_ (currently just _/hardware_infsensor_) for all tests
- Stores rosbags in the _stored_data/rosbags/$name_ folder inside the [data_collection](https://github.com/OSUrobotics/infrastructure-packages/tree/new_file_structure/data_collection) package. _$name_ is the name launch parameter set when [launching the main infrastructure system](https://github.com/OSUrobotics/infrastructure-packages/blob/new_file_structure/README.md#launching).

__rosbag_record_timestamps__
- Records all topics with the suffix _\_timestamps_ (currently just _/hardware_timestamps_) for all tests
- Stores rosbags in the  _stored_data/rosbags/timestamps_ inside the [data_collection](https://github.com/OSUrobotics/infrastructure-packages/tree/new_file_structure/data_collection) package.

## infrastructure_flexbe_states Package Overview
Contains source code for each state in the flexbe behavior __System_Behaviour_Pi__. Some states have the same source code but with different parameter values, as shown in the __System_Behaviour_Pi Behavior Overview__ section.

Also contains generic templates for common ROS nodes.

Used Nodes:
- parameter_ac.py
- stage_ac.py
- test_control_state.py
- trial_control_state.py

## System_Behaviour_Pi Behavior Overview

The flow of the states...

### States
1) Test Control:
  - In charge of controlling each test 
  - Takes in the test information from the csv file and outputs them as userdata.
  - Checks the validity of the user input.
  - Source code: [test_control_state.py](https://github.com/OSUrobotics/infrastructure-packages/blob/new_file_structure/infrastructure_behaviors/infrastructure_flexbe_states/src/infrastructure_flexbe_states/test_control_state.py)
2) Trial Control:
  - Takes in the test information from the csv file and outputs them as userdata.
  - Checks the validity of the user input.
  - Source code: [test_control_state.py](https://github.com/OSUrobotics/infrastructure-packages/blob/new_file_structure/infrastructure_behaviors/infrastructure_flexbe_states/src/infrastructure_flexbe_states/test_control_state.py)
