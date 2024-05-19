#!/usr/bin/env python3

import rospy
# import the moveit_commander, which allows us to control the arms
import moveit_commander
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
from q_learning_project.msg import RobotMoveObjectToTag
from std_msgs.msg import String
import json

class Robot(object):

    def __init__(self):

        # initialize this node
        rospy.init_node('turtlebot3_movement')
        
        # Laser Scan sub
        self.scanner_sub = rospy.Subscriber('/scan', LaserScan, self.update_min_distance)
        self.tag_or_object_pub = rospy.Publisher('tag_or_object', String, queue_size=1)
        # publish to /cmd_vel
        self.robot_movement_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.action_complete_pub = rospy.Publisher('action_complete', String, queue_size=1)
        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        #self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")
        #self.move_group_arm = moveit_commander.MoveGroupCommander("arm")
        self.gripper_joint_close = [-0.005, -0.005]
        self.gripper_joint_open = [0.019, 0.019]
        self.holding_state = [0, -1.54, 1.26, -1.55]
        self.grabbing_state = [0, 0.84, -0.45, -0.52]
        self.is_grabbing = False
        self.min_distance = float('inf')
        self.hit_wall = False
        self.turn = None


        # Reset arm position
        #self.move_group_arm.go([0,0,0,0], wait=True)

        self.need_object = None 

        rospy.Subscriber("detection_info", String, self.center_target)

        print("ready")

    def update_min_distance(self, data):
        # filter out zero values from ranges array
        non_zero_ranges = [distance for distance in data.ranges if distance > 0]
        # determine the closest object, the "person"
        min_distance = min(non_zero_ranges)
        self.min_distance = min_distance

    def center_target(self, data):
        #print("incoming data from color handling detected, running...")
        tag_info = json.loads(data.data)
        avg_x = float(tag_info['avg_x'])
        target_corners = None
        # Check if the specific ID is in the list of detected IDs
        if avg_x  != -1:
            image_center_x = 120
            threshold = 18
            if avg_x < (image_center_x - threshold):
                self.turn_left()
                self.turn = "left"
            elif avg_x > (image_center_x + threshold):
                self.turn_right()
                self.turn = "right"
            else:
                self.go_straight()
                if self.min_distance < 0.43 or self.hit_wall == True:
                    ###TODO: logic to put down object
                    self.hit_wall = True
                    if self.min_distance < 1.0:
                        self.go_back()
                    else:
                        self.hit_wall = False
                        self.need_object = not self.need_object if self.need_object != None else False
                        print(f"need_object updated to {self.need_object}")
                        self.tag_or_object_pub.publish(json.dumps({}))
                        if self.need_object == True:
                            for _ in range(1):
                                print("Going to publish")
                                json_data = {}
                                self.action_complete_pub.publish(json.dumps(json_data))
                                rospy.sleep(0.5)
                                print("Should have published")
                        
        else:
            self.scan_environment()
            #print("Did not find target within image, turning....")


#####################BASIC OPERATIONS FOR ARM CONTROL########################################
    def set_holding_state(self, data=None):
        joint_status = self.gripper_joint_open if self.is_grabbing else self.gripper_joint_close
        self.move_group_gripper.go(self.gripper_joint_open, wait = True)
        self.move_group_gripper.stop()

        self.move_group_arm.go(self.holding_state, wait=True)
        self.move_group_arm.stop()
        print("1")

    def set_grab_state(self, data=None):
        self.move_group_gripper.go(self.gripper_joint_open, wait = True)
        self.move_group_gripper.stop()

        self.move_group_arm.go(self.grabbing_state, wait=True)
        self.move_group_arm.stop()

        self.move_group_gripper.go(self.gripper_joint_close, wait = True)
        self.move_group_gripper.stop()
        print("2")

#######################BASIC OPERATIONS FOR ROBOT MOVEMENT#############################
    def scan_environment(self):
        if self.turn == "right":
            self.turn_left()
        else:
            self.turn_right()
    def stop(self):
        straight_twist = Twist(
            linear = Vector3(0, 0, 0),
            angular = Vector3(0, 0, 0)
        )
        self.robot_movement_pub.publish(straight_twist)
    def go_straight(self):
        print("going straight")
        straight_twist = Twist(
            linear = Vector3(0.4, 0, 0),
            angular = Vector3(0, 0, 0)
        )
        self.robot_movement_pub.publish(straight_twist)
    def go_back(self):
        print("going back")
        straight_twist = Twist(
            linear = Vector3(-0.4, 0, 0),
            angular = Vector3(0, 0, 0)
        )
        self.robot_movement_pub.publish(straight_twist)
    def turn_right(self):
        print("going right")
        right_twist = Twist(
            linear = Vector3(0,0,0),
            angular = Vector3(0,0,-0.2)
        )
        self.robot_movement_pub.publish(right_twist)
    def turn_left(self):
        print("going left")
        left_twist = Twist(
            linear = Vector3(0,0,0),
            angular = Vector3(0,0,0.2)
        )
        self.robot_movement_pub.publish(left_twist)


    def action_handler(self, msg):
        self.desired_tag = msg.tag_id
        self.desired_object = msg.robot_object


    def run(self):
        rospy.spin()

if __name__ == "__main__":
    robot = Robot()
    robot.run()