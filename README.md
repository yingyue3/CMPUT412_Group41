# Introduction
This is the github page for group 41 at University of Alberta for course CMPUT412/503. 
# Group Member
Sarah Amini (samini1@ualberta.ca)

Yingyue Cao (yingyue3@ualberta.ca)

# Code Running Ins
To run the code for differnent exercise, please follow the steps below:
1. Clone the repositery: using the command below:
git clone https://github.com/yingyue3/CMPUT412_Group41.git
2. cd to one of the execise file
3. Makes the python files executable using the command:
chmod +x [Path to the file]
4. Build the project
dts devel build -H ROBOT_NAME -f
5. Run the node:
dts devel run -H ROBOT_NAME -L [node name] 
if it is a publisher: add tag -n publisher

# Exercise 1
The code for this exercise is in the file Exercise 1. 

We created it based on the Duckietown manule (https://docs-old.duckietown.org/daffy/duckietown-robotics-development/out/basic_development.html)

# Exercise 2
In this exercise, we worked with ROS nodes and topics to control the Duckiebot’s movement and process sensor data. We began by subscribing to the camera topic, then implemented basic motion tasks such as moving in a straight line and rotating in place. Finally, we tackled a D-shaped trajectory, which required precise calibration. Throughout the exercise, we used ROSBag to record and analyze data, faced calibration challenges, and resolved issues like endless wheel spinning using rospy.on_shutdown().

# Exercise 3
In this exercise, we implement a color lane detection node that identifies lanes based on camera input. Using OpenCV tools, we detect lane colors and estimate their position relative to the Duckiebot’s location.

First, we programmed the Duckiebot to react differently based on the detected lane color. Then, using the detected yellow and white lanes, we coded the Duckiebot to follow the lane and complete a full lap on the mat. To achieve this, we calculated the lane position error from the image and implemented P, PD, and PID control methods to adjust the Duckiebot’s movement accordingly.

# Exercise 4
Within this exercise, we control our Duckiebot to perform safe navigation like detecting the road sign, stop and wait for the peDuckstrains in front of the crosswalk, and maneuver around the broken-down bot. 

Our main nodes are stored in the path: packages/safety_detection/src

## Part 1:
The publisher is lane_detection.py, the Subscriber and the control function is navigate_template.py.
To run this part, first run the publisher node:
dts devel run -H ROBOT_NAME -L lane_detection -n publisher
Then run the subscriber node:
dts devel run -H ROBOT_NAME -L navigate

## Part 2:
There is in total one function for this part, the subscriber and the publisher and both included in crosswalk.py.
To run this part, run:
dts devel run -H ROBOT_NAME -L crosswalk

## Part 2:
There is in total one function for this part, the subscriber and the publisher and both included in safe_navigation.py.
To run this part, run:
dts devel run -H ROBOT_NAME -L safe_navigations