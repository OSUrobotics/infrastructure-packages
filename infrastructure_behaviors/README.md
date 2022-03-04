# infrastructure_behaviors Packages
## Overview

Includes two ROS packages:
- 

Only publishes time stamps if collect_data argument is set to True in [main launch file](https://github.com/OSUrobotics/infrastructure-packages/blob/new_file_structure/infrastructure_behaviors/infrastructure_flexbe_behaviors/launch/start_test.launch) for the infrastructure system.

Only records camera feed if video argument is set to True in [main launch file](https://github.com/OSUrobotics/infrastructure-packages/blob/new_file_structure/infrastructure_behaviors/infrastructure_flexbe_behaviors/launch/start_test.launch) for the infrastructure system.

__Note__: start_rosbags.launch in launch folder is currently not being used. Rosbags and the data_collection node are launched through the main launch file in the [infrastructure_flexbe_behaviors](https://github.com/OSUrobotics/infrastructure-packages/tree/new_file_structure/infrastructure_behaviors) package.

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

