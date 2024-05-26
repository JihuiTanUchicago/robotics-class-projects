#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

"""
Non-OOP code to take images to use for model
"""

# init node 
rospy.init_node('image_capture_node')


bridge = CvBridge()

# use os for directory handling
image_dir = 'robotics-final-project/turtlebot_images'
os.makedirs(image_dir, exist_ok=True)

# capture 700 images at a time
num_images = 700

def image_callback(msg):
    global image_count

    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    # display cam feed
    cv2.imshow('Camera Feed', cv_image)
    cv2.waitKey(1)

    # take image and save to filename
    image_filename = os.path.join(image_dir, f'pepsiiiiiiiii_{image_count:03d}.jpg')
    cv2.imwrite(image_filename, cv_image)
    rospy.loginfo(f'Image captured and saved: {image_filename}')

    image_count += 1
    if image_count >= num_images:
        rospy.signal_shutdown('Captured all images')

image_count = 0

image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, image_callback)
rospy.spin()