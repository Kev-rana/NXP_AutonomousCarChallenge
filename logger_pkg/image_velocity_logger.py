#! /usr/bin/python
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# rospy for the subscriber
import rclpy
from rclpy.node import Node
import os
# ROS Image message
import sensor_msgs.msg 
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3
# OpenCV2 for saving an image
import cv2
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from rclpy.exceptions import ParameterNotDeclaredException

from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import QoSProfile
from time import sleep

print("log 10")

class ImageVelocityLog(Node):

    def __init__(self):

        super().__init__("image_velocity_logger")
        self.x = 0
        self.z = 0
        
        self.count = 10000
        print("starting in 10 seconds")
        self.start_delay = 10.0
        sleep(self.start_delay)

        print("started")
        
        #self.rate = self.create_rate(0.1) 
        #self.rate = rclpy.create_node('simple_node').create_rate(2)
        # Get paramaters or defaults
        camera_image_topic_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='Camera image topic.')

        self.declare_parameter("camera_image", "Pixy2CMUcam5_sensor", 
            camera_image_topic_descriptor)
        self.cameraImageTopic = self.get_parameter("camera_image").value

        # Instantiate CvBridge
        self.bridge = CvBridge()

        #Subscribers
        self.imageSub = self.create_subscription(sensor_msgs.msg.Image, 
            '/trackImage0/image_raw', 
            self.image_callback, 
            qos_profile_sensor_data)

        # 
        self.cmd_vel_subscriber = self.create_subscription(Twist, 
            '/cupcar0/cmd_vel',
            self.vel_callback,
            10)

        self.create_timer(0.5, self.timer_callback)

        self.speed_vector = Vector3()
        self.steer_vector = Vector3()
        self.cmd_vel = Twist() # publishing message type/format

    def vel_callback(self, msg):
        self.x = msg.linear.x
        self.z = msg.angular.z

    def image_callback(self, msg):
        # Convert your ROS Image message to OpenCV2
        #print("callback")
        global cv2_img
        cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def timer_callback(self):
        # Save your OpenCV2 image as a jpeg          
        os.chdir("/home/kev/temp_folder/log_tracks/log10")
        os.chdir("/home/kev/temp_folder")
        
        #time = msg.header.stamp
        # print(str(time))
        #cv2.imwrite(''+str(time)+'.jpeg'    , cv2_img)
        self.count = self.count + 0.5
        cv2.imwrite(''+str(self.count)+'.jpeg', cv2_img)
        
        f = open("/home/kev/temp_folder/log_tracks/log10.csv", "a")
        f.write(str(self.count)+'.jpeg' + "," + str(self.x) + "," + str(self.z) + "\n")
        #print(self.x + ","self.z)
        print(self.count)
        f.close()
        #self.rate.sleep()

            #rclpy.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    #print("initialized")
    image_velocity_logger = ImageVelocityLog()
    #print("node initialized")
    #rate = image_velocity_logger.create_rate(0.1) 
    #rate.sleep()
    rclpy.spin(image_velocity_logger)
    #print("spin")
    rclpy.shutdown()
    #print("shut")
    

if __name__ == '__main__':
    main()