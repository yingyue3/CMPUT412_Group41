#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun my_package lane_based_behavior_controller_template.py

# wait for app to end
dt-launchfile-join