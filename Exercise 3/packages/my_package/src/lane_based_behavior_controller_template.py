#!/usr/bin/env python3

# potentially useful for question - 1.6

# import required libraries
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped
from duckietown_msgs.msg import WheelsCmdStamped
from duckietown_msgs.msg import LEDPattern 
from std_msgs.msg import Header, ColorRGBA, String 
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge



class BehaviorController(DTROS):
    def __init__(self, node_name):
        super(BehaviorController, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)
        # add your code here
        # static parameters
        self.color = 'g'
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self._camera_info = f"/{self._vehicle_name}/camera_node/camera_info"
        self.led_topic = f"/{self._vehicle_name}/led_emitter_node/led_pattern"
        # camera calibration parameters (intrinsic matrix and distortion coefficients)
        self.K = None
        self.D = None
        self.sub_info = rospy.Subscriber(self._camera_info, CameraInfo, self.callback_info)
        
        # color detection parameters in HSV format
        # Set range for red color
        self.red_lower = np.array([130, 70, 80], np.uint8) 
        self.red_upper = np.array([190, 255, 255], np.uint8) 


        # Set range for green color 
        self.green_lower = np.array([36, 52, 72], np.uint8) 
        self.green_upper = np.array([102, 180, 180], np.uint8) 

        # Set range for blue color 
        self.blue_lower = np.array([110, 80, 80], np.uint8) 
        self.blue_upper = np.array([120, 200, 200], np.uint8) 
        rospy.loginfo("Starting")
        # initialize bridge and subscribe to camera feed
        self._bridge = CvBridge()
        self.disorted_image = None
        self.color_detect_image = None
        self.sub_image = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback_image)
        

        # lane detection publishers
        self._string_topic = f"/{self._vehicle_name}/control_node/control"
        self.string_pub = rospy.Publisher(self._string_topic, String, queue_size=1) # queue_size=10
        self._custom_topic = f"/{self._vehicle_name}/custom_node/image/compressed"
        self.pub = rospy.Publisher(self._custom_topic, Image, queue_size = 1) # queue_size=10

        # call navigation control node

        # LED stuff
        self.led_pub = rospy.Publisher(self.led_topic, LEDPattern, queue_size=1)
        
        # define other variables as needed
        self.executed = False
        self.line_disappear = False

        # self.execute_publish()


    def callback_info(self, msg):
        # rate = rospy.Rate(2)
        # https://stackoverflow.com/questions/55781120/subscribe-ros-image-and-camerainfo-sensor-msgs-format
        # http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html
        # https://github.com/IntelRealSense/realsense-ros/issues/709ss
        self.K = np.array(msg.K).reshape(3, 3)
        self.D = np.array(msg.D)
        # rospy.loginfo("Camera parameters received.")
        # rate.sleep()

    def callback_image(self, msg):
        # add your code here
        
        # convert compressed image to CV2
        rate = rospy.Rate(20)
        if self.K is None:
            return
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        # preprocess image
        imageFrame = self.preprocess_image(image).astype(np.uint8)
        # undistort image
        dst = self.undistort_image(image)
        # # preprocess image
        # imageFrame = self.preprocess_image(dst).astype(np.uint8)
        self.disorted_image = imageFrame
        # rospy.loginfo("Image Calibrated")
        # detect lanes - 2.1 

        # publish lane detection results
        
        # detect lanes and colors - 1.3
        # publish undistorted image
        self.color_detect_image = self.detect_line(self.disorted_image)
        # control LEDs based on detected colors

        # anything else you want to add here
        # rate.sleep()
        self.execute_publish()

    def undistort_image(self, image):
        # convert JPEG bytes to CV image
        # rate = rospy.Rate(3)
        if self.K is None:
            return
        # https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
        h,w = image.shape[:2]
        newcameramtx, roi = cv.getOptimalNewCameraMatrix(self.K, self.D, (w,h), 1, (w,h))
        dst = cv.undistort(image, self.K, self.D, None, newcameramtx)
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]
        # rospy.loginfo("Image Calibrated")
        return dst
        # rate.sleep()

    def preprocess_image(self, raw_image):
        new_width = 200
        new_height = 150
        resized_image = cv.resize(raw_image, (new_width, new_height), interpolation = cv.INTER_AREA)
        blurred_image = cv.blur(resized_image, (5, 5)) 
        return blurred_image
            
    def set_led_pattern(self, **kwargs):
        # add your code here
        x = ()
        if color == "blue":
            x = (0.0, 0.0, 1.0, 1.0)
        elif color == "red":
            x = (1.0, 0.0, 0.0, 1.0)
        elif color == "green":
            x = (0.0, 1.0, 0.0, 1.0)

        msg = LEDPattern()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        color_msg = ColorRGBA()
        color_msg.r, color_msg.g, color_msg.b, color_msg.a = x

        # Set LED colors
        msg.rgb_vals = [color_msg] * 5
        self.led_pub.publish(msg) 
        
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
            if(area > 100): 
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
        return imageFrame
    
    def use_leds(self):

        x = (0.0, 0.0, 1.0, 1.0)
        
        msg = LEDPattern()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        color_msg = ColorRGBA()
        color_msg.r, color_msg.g, color_msg.b, color_msg.a = x

        color_list = [color_msg] * 5
        

        # Set LED colors
        msg.rgb_vals = [color_msg] * 5
        self.led_pub.publish(msg) 


    def execute_blue_line_behavior(self, **kwargs):
        # static color: blue
        static_color = (0.0, 1.0, 0.0, 1.0)
        # signal color: red
        signal = (1.0, 0.0, 0.0, 1.0)
        msg = LEDPattern()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        color_msg = ColorRGBA()
        color_msg.r, color_msg.g, color_msg.b, color_msg.a = static_color
        signal_msg = ColorRGBA()
        signal_msg.r, signal_msg.g, signal_msg.b, signal_msg.a = signal

        color_list = [color_msg] * 5
        color_list[1] = signal_msg
        color_list[4] = signal_msg

        msg.rgb_vals = color_list
        self.led_pub.publish(msg) 


        
    def execute_green_line_behavior(self, **kwargs):
        # static color: blue
        static_color = (0.0, 0.0, 1.0, 1.0)
        # signal color: red
        signal = (1.0, 0.0, 0.0, 1.0)
        msg = LEDPattern()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        color_msg = ColorRGBA()
        color_msg.r, color_msg.g, color_msg.b, color_msg.a = static_color
        signal_msg = ColorRGBA()
        signal_msg.r, signal_msg.g, signal_msg.b, signal_msg.a = signal

        color_list = [color_msg] * 5
        color_list[0] = signal_msg
        color_list[3] = signal_msg

        msg.rgb_vals = color_list
        self.led_pub.publish(msg) 
        
    def execute_yellow_line_behavior(self, **kwargs):
        # add your code here
        pass

    def execute_publish(self):
        rate = rospy.Rate(20)
        if self.color == 'r':
            message = "straight"
        elif self.color == "g":
            message = "left"
        elif self.color == "b":
            message = "right"
        self.use_leds()
        if not rospy.is_shutdown():
            if self.color_detect_image is not None:
                # rospy.loginfo('publishing image')
                image_msg = self._bridge.cv2_to_imgmsg(self.color_detect_image, encoding="bgr8")
                self.pub.publish(image_msg)
            if self.line_disappear and not self.executed:
                # self.string_pub.publish(message)
                if self.color == "g":
                    self.execute_green_line_behavior()
                elif self.color == "b":
                    self.execute_blue_line_behavior()
                self.string_pub.publish(message)
                # self.executed = True
                rospy.loginfo(message)
            #self.pub.publish(self.raw_image)
        # rate.sleep()
    # add other functions as needed

if __name__ == '__main__':
    node = BehaviorController(node_name='behavior_controller_node')
    rospy.spin()