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
Contains source code for each state in the flexbe behavior __System_Behaviour_Pi__. Some states have the same source code but with different parameter values, as shown in the __Flexbe Overview__ section.

Also contains generic templates for common ROS nodes.



## data_collection Package Interface

### Action Servers:
- __set_data_collection__
  - Action server that the _Start Data Collection_ stage action client sends a goal to.
  - Used to signal the start of the data collection period. Increments the trial count for the current test. Sends result back after sleeping for three seconds to allow a [hardware controller](https://github.com/OSUrobotics/infrastructure-raspi/tree/main) that publishes data to trigger the stop_sleep_sub flag.
- __stop_data_collection__
  - Action server that the _Stop Data Collection_ stage action client sends a goal to.
  - Used to signal the end of the data collection period. Publishes the time stamp for the data collection period to the /hardware_timestamps topic. Sends result back immediately.
### Services:
- None
### Publishers:
- __time_stamp_pub__
  - Creates the _/hardware_timestamps_ topic and publishes all of the time stamps for each trial. Uses the DataTimestamps message.
### Subscribers:
- None
### Topics:
- /hardware_timestamps
- /start_data_collection/cancel
- /start_data_collection/feedback
- /start_data_collection/goal
- /start_data_collection/result
- /start_data_collection/status
- /stop_data_collection/cancel
- /stop_data_collection/feedback
- /stop_data_collection/goal
- /stop_data_collection/result
- /stop_data_collection/status

