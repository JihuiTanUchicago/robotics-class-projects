#!/usr/bin/env python3
import rospy
import sys
import numpy as np
from q_learning import QLearning
from q_learning_project.msg import QLearningReward, RobotMoveObjectToTag

class QDataTraining(object):
    def __init__(self) -> None:
        rospy.init_node('q_data_training')
        self.learning_rate = 1
        self.discount_factor = 0.8
        self.robot_id_dict = {
            0: "pink",
            1: "green",
            2: "blue"
        }
        # Opens file location
        self.action_matrix = np.loadtxt('./action_states/action_matrix.txt', dtype=int)
        print(f"action_matrix: {self.action_matrix}")
        self.actions = np.loadtxt('./action_states/actions.txt', dtype=int)
        print(f"actions {self.actions}")
        self.q_matrix_old = np.zeros_like(self.action_matrix)
        self.q_matrix = np.zeros_like(self.action_matrix)
        print(f"initial_q_matrix: {self.q_matrix}")
        self.reward_amount = 0
        action_dict = {} # a dictionay of dictionaries, e.g. rows are action_dict.keys, available columns are action_dict[i].keys, and available actions are action_dict[i].values
        for i, row in enumerate(self.action_matrix):
            sub_dict = {}
            for j, value in enumerate(row):
                if value >= 0:
                    sub_dict[j] = value
            action_dict[i] = sub_dict
        self.action_dict = action_dict
        print(f"self.action_dict: {self.action_dict}")
        rospy.Subscriber("/q_learning/reward", QLearningReward, self.receive_reward)
        self.action_pub = rospy.Publisher("/q_learning/robot_action", RobotMoveObjectToTag, queue_size=10)
        rospy.sleep(0.1)
        print("init finished")

    
    def receive_reward(self, reward_data) -> None:
        self.reward_amount = reward_data.reward
        print(f"reward_amount received = {self.reward_amount}")
        self.update_q_matrix(self.current_state, self.next_state)

    def update_q_matrix(self, current_state, next_state):
        # Bug: used states is 100 while the next state is 0, so it gives a max of -100 which is not being discounted either
        # The fact it is not is counting is why we are seeing everything reset to 0.
        action_index = self.action_matrix[current_state][next_state]
        print(f"update_q_matrix: current state = {current_state}, next state = {next_state},action_index = {action_index} ")
        object_id, tag_id = self.actions[action_index]
        future_value = max(self.q_matrix[next_state])
        print(f"update_q_matrix: object_id {object_id}, tag_id {tag_id}, future_value {future_value}")
        self.q_matrix[current_state][next_state] += self.learning_rate * (
            self.reward_amount + self.discount_factor * future_value - self.q_matrix[current_state][next_state]
        )
        print(f"update_q_matrix: self.q_matrix = {self.q_matrix}")
    
    def perform_action(self):
        available_states = self.action_dict[self.current_state]
        next_state = np.random.choice(list(available_states.keys()))
        self.next_state = next_state
        next_action = self.action_dict[self.current_state][self.next_state]
        print(f"current state = {self.current_state}")
        print(f"next state = {self.next_state}")
        print(f"next action = {next_action}")
        # publish the action
        object_id, tag_id = self.actions[next_action]
        action_msg = RobotMoveObjectToTag(robot_object=self.robot_id_dict[object_id], tag_id=tag_id)
        self.action_pub.publish(action_msg)
        rospy.sleep(0.5)
        print("Published action: ", action_msg)

    # This will run the Q-Learning Matrix
    def data_process(self) -> None:
        iteration = 0
        while iteration < 200:
            self.q_matrix_old = np.copy(self.q_matrix)
            iteration += 1
            print(f"**********************************iteration {iteration}**********************************")
            print(f"current q_matrix(old/holder): {self.q_matrix_old}")
            self.current_state = 0
            while self.current_state not in [27, 30, 39, 45, 54, 57] and len(self.action_dict[self.current_state]) != 0: # Stats when all three tags have objects
                self.perform_action()

                #update q-matrix based on reward
                current_reward = self.reward_amount
                print(f"reward_amount: {current_reward}")
            
                self.current_state = self.next_state
                print(f"********current iteration finished!***********")
        self.save_q_matrix()


    def save_q_matrix(self):
        # TODO: You'll want to save your q_matrix to a file once it is done to
        # avoid retraining
        print("Finished convergence!!!!!!!!!!!!!!!!1")
        np.savetxt("q_matrix.csv", self.q_matrix, delimiter=",", fmt='%.2f')
            

    #We can decide if this spins based on rospy.spin or something else
    def run(self) -> None:
        pass

if __name__ == "__main__":
    np.set_printoptions(threshold=sys.maxsize)
    q_data = QDataTraining()
    q_data.data_process()
    print(q_data.q_matrix)