#!/usr/bin/env python3

# potentially useful for question - 1.5

# import required libraries
import rospy
from duckietown.dtros import DTROS, NodeType

import os
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped
import numpy as np
from duckietown_msgs.msg import LEDPattern, WheelEncoderStamped

import cv2 as cv
from cv_bridge import CvBridge
import dt_apriltags as aptag
from std_msgs.msg import Header, ColorRGBA, Int32, String
import time

class NavigationControl(DTROS):
    def __init__(self, node_name):
        super(NavigationControl, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)
        # add your code here
        
        # publisher for wheel commands
        # NOTE: you can directly publish to wheel chassis using the car_cmd_switch_node topic in this assignment (check documentation)
        # you can also use your exercise 2 code
        
        # robot params

        # define other variables as needed
        # controller type
        self.control_type = "PID"  # it can be P or PD or PID

        # variables
        self.image_w = 400
        
        # PID gains 
        self.proportional_gain = 0.05
        self.derivative_gain = 0.03
        self.integral_gain = 0.001

        #Straight

        # self.proportional_gain = 0.05
        # self.derivative_gain = 0.03
        # self.integral_gain = 0.001
        
        # control variables
        self.prev_error = 0
        self.history = np.zeros((1,10))
        self.integral = 0
        
        # movement parameters
        self.speed = 0.4
        # self.speed = 0
        self.error = 0
        
        # distance tracking
        #self.calibration = 130

        self.calibration = -181

        self.test = []
        
        # initialize publisher/subscribers
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self.wheels_topic = f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd"
        # self.wheel_publisher = rospy.Publisher(self.wheels_topic, WheelsCmdStamped, queue_size=1)

        self.lane_topic = f"/{self._vehicle_name}/custom_node/image/black"

        self.lane_sub = rospy.Subscriber(self.lane_topic, Image, self.yellow_lane_callback)

        # yellow bound
        self.yellow_lower = np.array([20, 80, 100], np.uint8) 
        self.yellow_upper = np.array([40, 255, 255], np.uint8) 

        # while bound
        self.white_lower = np.array([0, 0, 180], np.uint8) 
        self.white_upper = np.array([180, 40, 255], np.uint8)

        # Set range for red color
        self.red_lower = np.array([130, 70, 80], np.uint8) 
        self.red_upper = np.array([190, 255, 255], np.uint8)


        self._bridge = CvBridge()

        # twisted topic
        twist_topic = f"/{self._vehicle_name}/car_cmd_switch_node/cmd"
        # form the message
        self._v = 0
        self._omega = 0
        # construct publisher
        self.twisted_publisher = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)


        # None lane-following stuff

        # self.line_disappear = False
        self.red_lane_message = "No"

        self._string_topic = f"/{self._vehicle_name}/control_node/red_lane"

        self._ticks_left = 0
        self._ticks_right = 0
        self.sub_instruction = rospy.Subscriber(self._string_topic, String, self.callback_string)
        self.pub = rospy.Publisher(self.wheels_topic, WheelsCmdStamped, queue_size=1)
        
        # publisher for wheel commands
        # NOTE: you can directly publish to wheel chassis using the car_cmd_switch_node topic in this assignment (check documentation)
        # you can also use your exercise 2 code
        
        # robot params
        self._radius = rospy.get_param(f'/{self._vehicle_name}/kinematics_node/radius', 100)
        self.DISTANCE_PER_TICK = np.pi*2*self._radius/135
        self.start_dist = 0

        self._apriltag_topic = f"/{self._vehicle_name}/control_node/apriltag"
        self.sub_april = rospy.Subscriber(self._apriltag_topic, Int32, self.callback_apriltag)
        self.aprilid = 1000

        # self.gray_topic = f"/{self._vehicle_name}/custom_node/image/gray"
        # self.lane_sub = rospy.Subscriber(self.gray_topic, Image, self.callback_image)

        self.led_topic = f"/{self._vehicle_name}/led_emitter_node/led_pattern"
        self.led_pub = rospy.Publisher(self.led_topic, LEDPattern, queue_size=1)

        self._tag_topic = f"/{self._vehicle_name}/control_node/tag"
        self.sub_instruction = rospy.Subscriber(self._tag_topic, String, self.tag_callback)

        self.tag_id = 1000

        
        self.gray = None
        self.x = (1.0, 1.0, 1.0, 1.0)
        self.prev_x = (1.0, 1.0, 1.0, 1.0)

        self.last_seen_sign = np.ones((1,400))*1000

    def callback_image(self, image):
        # add your code here
        
        self.gray = self._bridge.imgmsg_to_cv2(image) 
        # h,w = self.gray.shape

        # self.gray = self.gray[h//4:-h//4,w//2:]

        self.detect_tag()
    
   
    
    def callback_apriltag(self, data):
        aid = data.data
        # if aid != 1000:
        #     self.aprilid = aid
    def tag_callback(self, msg):
        self.tag_id = int(msg.data)
        self.last_seen_sign= np.roll(self.last_seen_sign, shift=-1, axis=1)  # Shift all values left
        self.last_seen_sign[0, -1] = self.tag_id
        self.sign_to_led()

    
    def callback_string(self, data):
        self.red_lane_message = data.data

    def calculate_p_control(self):
        # add your code here
        return self.proportional_gain * self.error

    def calculate_pd_control(self):
        # add your code here
        derivative = self.error - self.prev_error 
        return self.proportional_gain * self.error + self.derivative_gain * derivative
    
    def calculate_pid_control(self):
        # add your code here
        self.history = np.roll(self.history, shift=-1, axis=1)  # Shift all values left
        self.history[0, -1] = self.error
        self.integral = np.sum(self.history)
        derivative = self.error - self.prev_error 
        return self.proportional_gain * self.error + self.derivative_gain * derivative + self.integral_gain * self.integral
    
   
    
    def get_control_output(self):
        if self.control_type == "P":
            control = self.calculate_p_control()
        elif self.control_type == "PD":
            control = self.calculate_pd_control()
        elif self.control_type == "PID":
            control = self.calculate_pid_control()
        else:
            rospy.logwarn("Invalid control type!")
            control = 0.0
        self.prev_error = self.error

        self.publish_cmd(control)

        return control
    
    def publish_cmd(self, control):
        message = Twist2DStamped(v=self.speed, omega= -1 * control)
        self.twisted_publisher.publish(message)

        pass

    def yellow_lane_callback(self, image):

        # Single line approach
        
        image = self._bridge.imgmsg_to_cv2(image) 

        # Creating contour to track red color 
        contours, hierarchy = cv.findContours(image, 
                                            cv.RETR_TREE, 
                                            cv.CHAIN_APPROX_SIMPLE) 
        
        
        if len(contours)>0:
            # Sort contours by area in descending order and pick the top two
            max_contour = sorted(contours, key=cv.contourArea, reverse=True)[0]

            x_values = max_contour[:, 0, 0]  # Extracting x-coordinates

            # Compute the average x-coordinate
            avg_x = np.mean(x_values)
         
        # cv.line(image, (xc,yc), (xc+yc, yc), color=255)

        

        self.error = avg_x- image.shape[1]/2.0 + self.calibration

        

        return image
    
    def detect_line(self, imageFrame):
        hsvFrame = cv.cvtColor(imageFrame, cv.COLOR_BGR2HSV)
        # red mask
        # rospy.loginfo("line detecting")
        if self.color == 'r':
            mask = cv.inRange(hsvFrame, self.red_lower, self.red_upper) 
        elif self.color == "g":
            mask = cv.inRange(hsvFrame, self.green_lower, self.green_upper)
        elif self.color == "b":
            mask = cv.inRange(hsvFrame, self.blue_lower, self.blue_upper)

        kernel = np.ones((5, 5), "uint8") 
        mask = cv.dilate(mask, kernel) 
        res = cv.bitwise_and(imageFrame, imageFrame, 
                                mask = mask) 
        contours, hierarchy = cv.findContours(mask, 
                                            cv.RETR_TREE, 
                                            cv.CHAIN_APPROX_SIMPLE)

        for pic, contour in enumerate(contours): 
            area = cv.contourArea(contour) 
            if(area == 110): 
                x, y, w, h = cv.boundingRect(contour) 
                rospy.loginfo(y+h)
                # imageFrame = cv.rectangle(imageFrame, (x, y), 
                #                         (x + w, y + h), 
                #                         (0, 0, 255), 2) 
                
                # cv.putText(imageFrame, "Colour", (x, y), 
                #             cv.FONT_HERSHEY_SIMPLEX, 1.0, 
                #             (0, 0, 255))
                if y + h > 110:
                    self.line_disappear = True
                    rospy.loginfo("Stop")
        return imageFrame
    
    def on_shutdown(self):
        # stop = WheelsCmdStamped(vel_left=0, vel_right=0)
        # self.wheel_publisher.publish(stop)

        # rospy.loginfo(self.test)

        message = Twist2DStamped(v=0, omega=0)
        self.twisted_publisher.publish(message)
        self.x = (1.0, 1.0, 1.0, 1.0)
        self.publish_leds()
        
    def sign_to_led(self):

        
        if int(self.tag_id) == 50 or int(self.tag_id) == 133 or int(self.tag_id) == 15:
            self.x = (0.0, 0.0, 1.0, 1.0)
        elif int(self.tag_id) == 22 or int(self.tag_id) == 21:
            self.x = (1.0, 0.0, 0.0, 1.0)
        elif int(self.tag_id) == 93 or int(self.tag_id) == 94:
            self.x = (0.0, 1.0, 0.0, 1.0)
        else: 
            self.x = (1.0, 1.0, 1.0, 1.0)
        
        return 


    def publish_leds(self):    
        msg = LEDPattern()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        color_msg = ColorRGBA()
        color_msg.r, color_msg.g, color_msg.b, color_msg.a = self.x


        # Set LED colors
        msg.rgb_vals = [color_msg] * 5
        self.led_pub.publish(msg) 
        return
        # add your code here
        pass

    def stop(self):
        message = Twist2DStamped(v=0, omega= 0)
        self.twisted_publisher.publish(message)


    def start(self):
        rate = rospy.Rate(10)  # 10 Hz

        last_detected = time.time()
        while not rospy.is_shutdown():
            if self.prev_x != self.x:
                self.publish_leds()
                self.prev_x = self.x
            # rospy.loginfo(self.tag_id)
            if self.red_lane_message == "No":
                self.get_control_output()  # Call control function continuously
            else:
                self.red_lane_message = "No"

                

                    
                self.stop()
                last_detected = time.time()
                if int(np.min(self.last_seen_sign)) == 50 or int(np.min(self.last_seen_sign)) == 133 or int(np.min(self.last_seen_sign)) == 15:
                    # T Intersetion Tag
                    rospy.sleep(2)
                    self.tag_id == 1000
                elif int(np.min(self.last_seen_sign)) == 22 or int(np.min(self.last_seen_sign)) == 21:
                    # Stop sign
                    rospy.sleep(3)
                    self.tag_id == 1000
                elif int(np.min(self.last_seen_sign)) == 93 or int(np.min(self.last_seen_sign)) == 94:
                    # U of A sign
                    rospy.sleep(1)
                    self.tag_id== 1000
                else:
                    rospy.sleep(0.5)

                for i in range(10000):
                    self.get_control_output()
                self.red_lane_message = "No"
                    
                   
            rate.sleep()


    # add other functions as needed

if __name__ == '__main__':
    node = NavigationControl(node_name='navigation_control_node')
    node.start()
    rospy.spin()