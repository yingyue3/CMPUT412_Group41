#!/usr/bin/env python3

# potentially useful for question - 1.5

# import required libraries

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped
from duckietown_msgs.msg import WheelsCmdStamped
from duckietown_msgs.msg import LEDPattern 
from std_msgs.msg import Header, ColorRGBA, String
import numpy as np

class NavigationControl(DTROS):
    def __init__(self, node_name):
        super(NavigationControl, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)
        # add your code here

        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._left_encoder_topic = f"/{self._vehicle_name}/left_wheel_encoder_node/tick"
        self._right_encoder_topic = f"/{self._vehicle_name}/right_wheel_encoder_node/tick"
        self.wheels_topic = f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd"
        self._string_topic = f"/{self._vehicle_name}/control_node/control"

        self._ticks_left = 0
        self._ticks_right = 0
        self.sub_left = rospy.Subscriber(self._left_encoder_topic, WheelEncoderStamped, self.callback_left)
        self.sub_right = rospy.Subscriber(self._right_encoder_topic, WheelEncoderStamped, self.callback_right)
        self.sub_instruction = rospy.Subscriber(self._string_topic, String, self.callback_string)
        self.pub = rospy.Publisher(self.wheels_topic, WheelsCmdStamped, queue_size=1)
        
        # publisher for wheel commands
        # NOTE: you can directly publish to wheel chassis using the car_cmd_switch_node topic in this assignment (check documentation)
        # you can also use your exercise 2 code
        
        # robot params
        self._radius = rospy.get_param(f'/{self._vehicle_name}/kinematics_node/radius', 100)
        self.DISTANCE_PER_TICK = np.pi*2*self._radius/135
        self.start_dist = 0
        self.wheelbase = 0.205

        # define other variables as needed
        self.ROTATION_TARGET = np.pi*self.wheelbase / 4
        self.ARC_TARGET = np.pi*((self.wheelbase/2)+0.54) / 4


        self.instruction = None
        self.executed = False

        # self.control_start()
    
    def callback_string(self, data):
        # rospy.loginfo("I heard '%s'", data.data)
        self.instruction = data.data
        # rospy.loginfo(self.instruction)
        # self.control_start()

    def callback_left(self, data):
        self._ticks_left = data.data

    def callback_right(self, data):
        self._ticks_right = data.data
    
    def compute_distance_traveled(self, ticks):
        # left_distance = self._ticks_left* self.DISTANCE_PER_TICK
        # right_distance = self._ticks_right* self.DISTANCE_PER_TICK
        # dist = []
        # dist.append(left_distance)
        # dist.append(right_distance)
        distance = ticks* self.DISTANCE_PER_TICK
        return distance

        
    def publish_velocity(self, **kwargs):
        # add your code here
        pass
        
    def stop(self):
        # add your code here
        msg = WheelsCmdStamped()
        msg.vel_left = 0
        msg.vel_right = 0
        self.pub.publish(msg)
        pass
        
    def move_straight(self,  speed=0.5, direction=1, distance=0.3, calibrate = 1.3):
        # add your code here
        msg = WheelsCmdStamped()
        msg.vel_left = speed * direction * 1.3
        msg.vel_right = speed * direction

        self.start_dist = self.compute_distance_traveled(self._ticks_left)

        while np.abs(self.start_dist - self.compute_distance_traveled(self._ticks_left)) < distance and not rospy.is_shutdown():
            self.pub.publish(msg)

        msg.vel_left = 0
        msg.vel_right = 0
        self.pub.publish(msg)
        pass
        
    def turn_right(self, speed=0.7, calibrate = 1):
        # add your code here
        msg = WheelsCmdStamped()
        msg.vel_left = speed * calibrate
        msg.vel_right = speed * 0.38

        # Calculate target rotation distance (90 degrees)
         
        self.start_dist = self.compute_distance_traveled(self._ticks_left)


        while np.abs(self.start_dist - self.compute_distance_traveled(self._ticks_left)) < self.ARC_TARGET and not rospy.is_shutdown():
            self.pub.publish(msg)


        # Stop the robot after rotation
        msg.vel_left = 0
        msg.vel_right = 0
        self.pub.publish(msg)
        rospy.loginfo("Rotation arc complete (90 degrees clockwise).")
        pass
        
    def turn_left(self, speed=0.7, calibrate = 1):
        # add your code here
        msg = WheelsCmdStamped()
        msg.vel_left = speed * 0.38
        # msg.vel_right = -speed *0.8
        msg.vel_right = speed * calibrate

        # self._ticks_left = 0  
        # self._ticks_right = 0

        # Calculate target rotation distance (90 degrees)
         
        self.start_dist = self.compute_distance_traveled(self._ticks_left)


        while np.abs(self.start_dist - self.compute_distance_traveled(self._ticks_left)) < self.ARC_TARGET and not rospy.is_shutdown():
            self.pub.publish(msg)
            # rospy.sleep(0.1)


        # Stop the robot after rotation
        msg.vel_left = 0
        msg.vel_right = 0
        self.pub.publish(msg)
        rospy.loginfo("Rotation arc complete (90 degrees clockwise).")
        pass

    # add other functions as needed

    # TODO: add run function as needed
    def stop_robot(self):
        msg = WheelsCmdStamped()
        msg.vel_left = 0
        msg.vel_right = 0
        self.pub.publish(msg)
        rospy.loginfo("robot stopped")

    def control_start(self):
            # rospy.sleep(5)
        while not self.executed:
            msg = WheelsCmdStamped()
            msg.vel_left = 0.4 * 1.2
            msg.vel_right = 0.4 * 0.9
            self.pub.publish(msg)
            rospy.loginfo("move")
            if self.instruction is not None:
                rospy.loginfo("stop")
                self.stop_robot()
                rospy.sleep(5)
                if self.instruction == "left":
                    rospy.loginfo("turn left")
                    self.turn_left(calibrate = 0.7)
                    self.executed = True
                elif self.instruction == "right":
                    self.turn_right()
                    self.executed = True
                elif self.instruction == "straight":
                    rospy.loginfo("move straight started")
                    self.move_straight()
                    self.executed = True
                rospy.loginfo("executed")
                    
                        


if __name__ == '__main__':
    node = NavigationControl(node_name='navigation_control_node')
    # node.turn_left(calibrate = 0.5)

    node.control_start()
    rospy.spin()