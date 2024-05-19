#!/usr/bin/env python3

import rospy
import requests
# msgs needed for /cmd_vel
from geometry_msgs.msg import Twist, Vector3

class DriveSquare(object):
    """ This node publishes ROS messages containing the 3D coordinates of a single point """

    def __init__(self):
        # initialize the ROS node
        rospy.init_node('drive_square')
        # setup publisher to the cmd_vel ROS topic
        self.robot_movement_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    def get_force_signal(self):
        url = "http://ec2-35-94-176-147.us-west-2.compute.amazonaws.com:5050/get-data"
        response = requests.get(url)
        response.raise_for_status()  # Raise an exception for HTTP errors
        data = response.json()
        print(data)
        return data.get("params_dict", {}).get("force_signal", None)
    def go_straight(self):
        print("going straight")
        straight_twist = Twist(
            linear = Vector3(0.4, 0, 0),
            angular = Vector3(0, 0, 0)
        )
        rospy.sleep(1)
        self.robot_movement_pub.publish(straight_twist)
    def turn_right(self):
        print("going right")
        right_twist = Twist(
            linear = Vector3(0,0,0),
            angular = Vector3(0,0,0.83)
        )
        rospy.sleep(1)
        self.robot_movement_pub.publish(right_twist)
    def drive_square(self):
        while not rospy.is_shutdown():
            self.turn_right()
            rospy.sleep(1)
        
    def run(self):
        self.drive_square()
if __name__ == '__main__':
    # instantiate the ROS node and run it
    node = DriveSquare()
    node.run()
