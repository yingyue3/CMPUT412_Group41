# Introduction
This is the github page for group 41 at University of Alberta for course CMPUT412/503. 
# Group Member
Sarah Amini (samini1@ualberta.ca)

Yingyue Cao (yingyue3@ualberta.ca)
# Exercise 1
The code for this exercise is in the file Exercise 1. 

We created it based on the Duckietown manule (https://docs-old.duckietown.org/daffy/duckietown-robotics-development/out/basic_development.html)

# Exercise 2
In this exercise, we worked with ROS nodes and topics to control the Duckiebot’s movement and process sensor data. We began by subscribing to the camera topic, then implemented basic motion tasks such as moving in a straight line and rotating in place. Finally, we tackled a D-shaped trajectory, which required precise calibration. Throughout the exercise, we used ROSBag to record and analyze data, faced calibration challenges, and resolved issues like endless wheel spinning using rospy.on_shutdown().

# Exercise 3
In this exercise, we implement a color lane detection node that identifies lanes based on camera input. Using OpenCV tools, we detect lane colors and estimate their position relative to the Duckiebot’s location.

First, we programmed the Duckiebot to react differently based on the detected lane color. Then, using the detected yellow and white lanes, we coded the Duckiebot to follow the lane and complete a full lap on the mat. To achieve this, we calculated the lane position error from the image and implemented P, PD, and PID control methods to adjust the Duckiebot’s movement accordingly.
