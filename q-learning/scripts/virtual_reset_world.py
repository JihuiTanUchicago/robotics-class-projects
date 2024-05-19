#!/usr/bin/env python3

import rospy


from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from q_learning_project.msg import QLearningReward
from q_learning_project.msg import RobotMoveObjectToTag

from std_msgs.msg import Header

from random import shuffle
from tf.transformations import quaternion_from_euler, euler_from_quaternion


class ResetWorld(object):

    def __init__(self):

        # initialize this node
        rospy.init_node('virtual_reset_world_q_learning')
        # reward amounts
        self.positive_reward = 100
        self.negative_reward = 0

        # goal locations
        self.goal_robot_object = {
            "pink": 3,
            "green": 1,
            "blue": 2
        }

        # current locations of the robot objects relative to the tags
        self.current_robot_object = {
            "pink": 0,
            "green": 0,
            "blue": 0
        }

        # keep track of the iteration number
        self.iteration_num = 0

        # ROS subscribe to the topic publishing actions for the robot to take
        rospy.Subscriber("/q_learning/robot_action", RobotMoveObjectToTag, self.send_reward)
        
        # ROS publishers
        self.reward_pub = rospy.Publisher("/q_learning/reward", QLearningReward, queue_size=10)
        self.run()

    def send_reward(self, data):
        print(data)
        # update locations based on latest object movement
        self.current_robot_object[data.robot_object] = data.tag_id

        # default reward
        reward_amount = self.positive_reward
        reset_world = True

        # check if objects are in correct position & if the world should be reset
        for robot_object in self.current_robot_object.keys():
            if self.current_robot_object[robot_object] != self.goal_robot_object[robot_object]:
                reward_amount = self.negative_reward


            if self.current_robot_object[robot_object] == 0:
                reset_world = False
        if reward_amount == self.positive_reward:
            print(f"send_reward: ******** Positive Reward({reward_amount}) Received *******")
        if reset_world == True:
            print(f"send_reward: ******** RESETTING WORLD... *******")


        # prepare reward msg
        reward_msg = QLearningReward()
        reward_msg.header = Header(stamp=rospy.Time.now())
        reward_msg.reward = reward_amount
        reward_msg.iteration_num = self.iteration_num
        self.reward_pub.publish(reward_msg)
        print("Published reward: ", reward_amount)

        # increment iteration if world needs to be reset
        # reset object positions if world needs to be rest
        if reset_world:
            print("reseting the world")
            self.iteration_num += 1

            for robot_object in self.current_robot_object.keys():
                self.current_robot_object[robot_object] = 0




    def run(self):
        rospy.spin()

if __name__=="__main__":

    node = ResetWorld()
