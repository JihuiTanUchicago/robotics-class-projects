#!/usr/bin/env python3

import rospy

# msgs needed for /cmd_vel
from geometry_msgs.msg import Twist, Vector3

class DriveSquare(object):
    """ This node publishes ROS messages containing the 3D coordinates of a single point """

    def __init__(self):
        # initialize the ROS node
        rospy.init_node('drive_square')
        # setup publisher to the cmd_vel ROS topic
        self.robot_movement_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    def go_straight(self):
        straight_twist = Twist(
            linear = Vector3(0.4, 0, 0),
            angular = Vector3(0, 0, 0.17)
        )
        rospy.sleep(1)
        self.robot_movement_pub.publish(straight_twist)
    def turn_right(self):
        right_twist = Twist(
            linear = Vector3(0,0,0),
            angular = Vector3(0,0,0.17)
        )
        rospy.sleep(1)
        self.robot_movement_pub.publish(right_twist)
    def run(self):
        while True:
            self.go_straight()
            rospy.sleep(2)
            self.turn_right()
            rospy.sleep(1)
if __name__ == '__main__':
    # instantiate the ROS node and run it
    node = DriveSquare()
    node.run()
