#!/usr/bin/env python3

import rospy
# import the moveit_commander, which allows us to control the arms
import moveit_commander
import math
import numpy as np
import random
from q_learning_project.msg import RobotMoveObjectToTag
from std_msgs.msg import String
import json

class Robot(object):

    def __init__(self):

        # initialize this node
        rospy.init_node('turtlebot3_main')
        self.obj_to_grab = {
            0: "pink",
            1: "green",
            2: "blue"
        }
        # Publish The Needed AR Tag and Color Object
        # Also sets up the csv Q-matrix 
        self.publish_moves = rospy.Publisher('next_step', String, queue_size=1)
        self.prepare_reward_and_action_dict()
        self.prepare_actions()
        self.action_confirmation = rospy.Subscriber('action_complete', String, self.get_next_step)
        # Creates a default state for our booleans and reward values.
        self.current_reward = 0
        self.current_state = 0
        self.is_next_step = False
        self.next_move = None
        

    def get_next_step(self, data_no_use = None):
        """
        Reads the values from the CSV and uses them to grab
        next actions for tags and objects.
        Takes in data from a subscriber and 
        is used as a way to force a callback to here.

        Input:
            data_no_use = Subscriber data forced to None

        Returns None
        """
        # Grabs the next states based on the dictionariees 
        next_states = self.action_dict[self.current_state]
        next_state = random.choice(list(next_states.keys()))
        current_action = self.action_dict[self.current_state][next_state]
        color_object, tag_id = self.actions[current_action]
        # Prepares The Object and Tags for publishing 
        print(f"next step, color_object: {color_object}, tag_id: {tag_id}")
        self.next_move = {"color_object": self.obj_to_grab[color_object], "tag_id": str(tag_id)}
        print("CFURRENT REWARD", self.current_reward)
        print("CURRENT STATE", self.current_state)
        self.current_reward = self.reward_dict[self.current_state][next_state]
        print("UPDATED REWARD", self.current_reward)
        self.current_state = next_state
    
    def prepare_reward_and_action_dict(self):
        """
        Reads the location of csv data and upacks it.

        Returns None
        """
        source_folder = "./action_states/"
        #loading Q-matrix
        q_matrix = np.loadtxt(source_folder+"q_matrix.csv", delimiter=',',dtype=str).astype(np.float)
        action_matrix = np.loadtxt(source_folder+'action_matrix.txt', dtype=int)
        print(f"q_matrix loaded: {q_matrix}")
        
        # Sets up dictionaries with values from q_matrix
        reward_dict = {}
        action_dict = {}
        for row in range(q_matrix.shape[0]):
            cols = np.nonzero(q_matrix[row])[0]
            if cols.size > 0:
                reward_dict[row] = {col: q_matrix[row][col] for col in cols}
                action_dict[row] = {col: action_matrix[row][col] for col in cols}
        print(f"reward_dict: {reward_dict}")
        print(f"action_dict: {action_dict}")
        self.reward_dict = reward_dict
        self.action_dict = action_dict

    def prepare_actions(self):
        """
        This helps translate the CSV Q-Matrix Data to actual actions for the robot.

        Returns None
        """
        source_folder = "./action_states/"
        self.actions = np.loadtxt(source_folder+'actions.txt', dtype=int)
        print(f"actions {self.actions}")


    def run(self):
        """
        Keeps node alive using ROS

        Returns None
        """
        self.get_next_step()
        while True:
            self.publish_moves.publish(json.dumps(self.next_move))
            rospy.sleep(0.5)
            print("next step published to color handling", self.next_move)


            

if __name__ == "__main__":
    robot = Robot()
    robot.run()