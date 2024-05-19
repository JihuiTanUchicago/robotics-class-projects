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
        while True:
            try:
                self.move_group_arm = moveit_commander.MoveGroupCommander("arm")
                self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")
                break
            except Exception:
                print("keep trying")
        self.gripper_joint_close = [-0.01, -0.01]
        self.gripper_joint_open = [0.01, 0.01]
        self.holding_state = [0, -1.54, 1.26, -1.55]
        self.grabbing_state = [0, 0, 0.70, -0.66]
        self.is_grabbing = False
        self.min_distance = float('inf')
        self.hit_wall = False
        self.turn = None
        self.is_grabbed = False
        self.min_wall_distance = 0.48
        self.min_object_distance = 0.18


        # Reset arm position
        self.move_group_arm.go([0,0,0,0], wait=True)

        self.need_object = None 

        rospy.Subscriber("detection_info", String, self.center_target)

        print("ready")

    def update_min_distance(self, data):
        """
        Updates minimum distance based on scanner callback data
        LDS-01 only.

        Inputs:
            data = Subscriber dataa from lidar.

        Returns None
        """
        # filter out zero values from ranges array
        forward_case_one = list(range(0, 24))
        forward_case_two = list(range(336, 360))
        forward_angles = forward_case_one + forward_case_two
        populated_dictionary = {}
        for i, j in enumerate(data.ranges):
            if j != 0.0:
                if i not in populated_dictionary and i in forward_angles:
                    populated_dictionary[i] = j
        dict_to_list = [(k, v) for k, v in populated_dictionary.items()]
        dict_to_list.sort(key=lambda s: s[1])
        lowest_values = [(v[0], v[1]) for v in dict_to_list[:5]]
        sum_of_values = sum(v[1] for v in lowest_values)
        average_distance = sum_of_values / len(lowest_values)
        # determine the closest object, the "person"
        self.min_distance = average_distance
        print("Min distance", self.min_distance)

    def center_target(self, data):
        """
        Centers the target object (ar tag or object) to center of cam.

        Inputs:
            data = data from subscriber callback

        Returns None
        """
        #print("incoming data from color handling detected, running...")
        tag_info = json.loads(data.data)
        avg_x = float(tag_info['avg_x'])
        target_corners = None
        # Check if the specific ID is in the list of detected IDs
        if avg_x  != -1:
            image_center_x = 130
            threshold = 15
            if avg_x < (image_center_x - threshold):
                self.turn_left()
                self.turn = "left"
            elif avg_x > (image_center_x + threshold):
                self.turn_right()
                self.turn = "right"
            else:
                self.go_straight()
                if ((self.need_object != False and self.min_distance < self.min_object_distance) or (self.need_object == False and self.min_distance < self.min_wall_distance)) and self.hit_wall == False:
                    self.hit_wall = True
                    print("######################### NOTICE: enter into grabbing object or releasing object state #########################33")
                    while self.is_grabbed == False and self.need_object != False:
                        self.stop()
                        self.gripper_open()
                        rospy.sleep(5)
                        self.set_grab_state()
                        rospy.sleep(5)
                        self.gripper_close()
                        rospy.sleep(5)
                        self.set_holding_state()
                        rospy.sleep(5)
                        self.is_grabbed = True
                        print("stuck 1")
                        break
                    while self.is_grabbed == True and self.need_object == False:
                        self.stop()
                        self.set_grab_state()
                        rospy.sleep(5)
                        self.gripper_open()
                        rospy.sleep(5)
                        self.gripper_open()
                        rospy.sleep(5)
                        self.set_holding_state()
                        rospy.sleep(5)
                        self.is_grabbed = False
                        print("stuck 2")
                        break
                    print("################################3grabbing or releasing should finish!!!! OUT OF WHILE LOOP##################")
                    if self.min_distance < 0.52:
                        while self.min_distance < 0.55:
                            self.go_back()
                    self.hit_wall = False
                    self.need_object = not self.need_object if self.need_object != None else False
                    print(f"need_object updated to {self.need_object}")
                    self.tag_or_object_pub.publish(json.dumps({}))
                    if self.need_object == True:
                        print("Going to publish")
                        json_data = {}
                        self.action_complete_pub.publish(json.dumps(json_data))
                        self.is_grabbed = False
                        rospy.sleep(0.5)
                        print("Should have published")
                        
        else:
            self.scan_environment()
            #print("Did not find target within image, turning....")


#####################BASIC OPERATIONS FOR ARM CONTROL########################################
    def gripper_close(self, data=None):
        """
        Publishes an gripper grab command for the robot.
        Subscribes for callback.

        Inputs:
            data = Forced to None,

        Returns None
        """
        print("gripper should close now")
        self.move_group_gripper.go(self.gripper_joint_close, wait=True)
        rospy.sleep(5)
        self.move_group_gripper.stop()

    def gripper_open(self, data=None):
        """
        Publishes an gripper release command for the robot.
        Subscribes for callback.

        Inputs:
            data = Forced to None,

        Returns None
        """
        print("gripper should oepn now")
        self.move_group_gripper.go(self.gripper_joint_open, wait=True)
        rospy.sleep(5)
        self.move_group_gripper.stop()
    
    def set_holding_state(self, data=None):
        """
        Publishes an arm hold command for the robot.
        Subscribes for callback.

        Inputs:
            data = Forced to None,

        Returns None
        """
        print("entre idle")
        self.move_group_arm.go(self.holding_state, wait=True)
        print("arm_go")
        rospy.sleep(5)
        self.move_group_arm.stop()
        print("arm_stop")
        print("rospy sleep finished")

    def set_grab_state(self, data=None):
        """
        Publishes an arm grab command for the robot.
        Subscribes for callback.

        Inputs:
            data = Forced to None,

        Returns None
        """
        print("enter grabbing")
        self.move_group_arm.go(self.grabbing_state, wait=True)
        print("arm_go")
        rospy.sleep(5)
        self.move_group_arm.stop()
        print("arm_stop")
        print("rospy sleep finishd")


#######################BASIC OPERATIONS FOR ROBOT MOVEMENT#############################
    def scan_environment(self):
        """
        Publishes a spin command for the robot.

        Returns None
        """
        if self.turn == "right":
            self.turn_left()
        else:
            self.turn_right()
    def stop(self):
        """
        Publishes a stop command for the robot.

        Returns None
        """
        straight_twist = Twist(
            linear = Vector3(0, 0, 0),
            angular = Vector3(0, 0, 0)
        )
        self.robot_movement_pub.publish(straight_twist)
    def go_straight(self):
        """
        Publishes a forward command for the robot.

        Returns None
        """
        print("going straight")
        straight_twist = Twist(
            linear = Vector3(0.11, 0, 0),
            angular = Vector3(0, 0, 0)
        )
        self.robot_movement_pub.publish(straight_twist)
    def go_back(self):
        """
        Publishes a back up command for the robot.

        Returns None
        """
        print("going back")
        straight_twist = Twist(
            linear = Vector3(-0.11, 0, 0),
            angular = Vector3(0, 0, 0)
        )
        self.robot_movement_pub.publish(straight_twist)
    def turn_right(self):
        """
        Publishes a right turn for the robot

        Returns None
        """
        print("going right")
        right_twist = Twist(
            linear = Vector3(0,0,0),
            angular = Vector3(0,0,-0.05)
        )
        self.robot_movement_pub.publish(right_twist)
    def turn_left(self):
        """
        Publishes a left turn for the robot

        Returns None
        """
        print("going left")
        left_twist = Twist(
            linear = Vector3(0,0,0),
            angular = Vector3(0,0,0.05)
        )
        self.robot_movement_pub.publish(left_twist)


    def action_handler(self, msg):
        """
        Grabs the desired tags from a subscriber callback from main

        Inputs:
           msg = Callback data from main subscriber
        Returns none
        """
        self.desired_tag = msg.tag_id
        self.desired_object = msg.robot_object


    def run(self):
        """
        Rospy keeps the node alive.

        Returns None
        """
        self.stop()
        self.set_holding_state()
        rospy.sleep(5)
        self.gripper_open()
        rospy.sleep(5)
        rospy.spin()

if __name__ == "__main__":
    robot = Robot()
    robot.run()