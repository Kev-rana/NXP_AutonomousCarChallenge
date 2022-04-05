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


import tensorflow as tf
import numpy as np

class LineFollow(Node):

    def __init__(self):

        self.start_delay = 20.0
        super().__init__('aim_line_follow')

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

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cupcar0/cmd_vel', 10)

        self.speed_vector = Vector3()
        self.steer_vector = Vector3()
        self.cmd_vel = Twist() # publishing message type/format


        # 
        #self.cmd_vel_subscriber = self.create_subscription(Twist, 
        #    '/cupcar0/cmd_vel',
        #    self.vel_callback,
        #    10)
        

        #self.create_timer(0.5, self.timer_callback)

        self.speed_vector = Vector3()
        self.steer_vector = Vector3()
        self.cmd_vel = Twist() # publishing message type/format


    def image_callback(self, img):
        img = self.bridge.imgmsg_to_cv2(img, "bgr8")

        img = img[150:800, 36:1260, :]
        #img = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
        img = cv2.GaussianBlur(img, (3, 3), 0)
        img = cv2.resize(img, (256, 128))
        img = img/255

        #img = preprocess(img)
        img = np.expand_dims(img, axis=0)
        global steer, speed
        pred = nxp_model.predict(img)
        steer = pred[0][1]
        speed = pred[0][0]
        
        if (float(speed) < 0.5):
            steer = float(steer) * 2.0
            speed = float(speed) * 0.95
        
        #speed = float(speed) *(1-min(np.abs(2.5*steer), 0.9))
        self.speed_vector.x = float(speed) 
        self.steer_vector.z = float(steer) 
        #print(pred[0][0], type(pred[0][0]))
        # Publish the message for the actuators

        
        self.cmd_vel.linear = self.speed_vector 
        self.cmd_vel.angular = self.steer_vector
        
        self.cmd_vel_publisher.publish(self.cmd_vel)
            #print(steer, speed)




def main(args=None):
    global nxp_model
    nxp_model = tf.keras.models.load_model('/home/kev/ros2ws/src/aim_line_follow/modelfinal1.h5')
    rclpy.init(args=args)
    line_follow = LineFollow()
    rclpy.spin(line_follow)
    line_follow.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    




