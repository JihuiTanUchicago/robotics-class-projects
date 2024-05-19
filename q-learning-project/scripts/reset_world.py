#!/usr/bin/env python3

import rospy


from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from q_learning_project.msg import QLearningReward
from std_msgs.msg import Header

from random import shuffle
from tf.transformations import quaternion_from_euler, euler_from_quaternion


class ResetWorld(object):

    def __init__(self):

        # initialize this node
        rospy.init_node('reset_world_q_learning')

        # reward amounts
        self.positive_reward_amount = 100
        self.negative_reward_amount = 0

        self.robot_objects = ["pink", "green", "blue"]

        # object model names
        self.object_model_names = {
            "pink": "robot_object_pink",
            "green": "robot_object_green",
            "blue": "robot_object_blue"
        }

        # reset positions and orientations of objects
        self.reset_xyz_positions_of_objects = [
            Point(x=1.0635, y=-0.5, z=0.1905),
            Point(x=1.0635, y=0.0, z=0.1905),
            Point(x=1.0635, y=0.5, z=0.1905)
        ]
        reset_quat_orientation_of_objects_list = quaternion_from_euler(1.5708, 0.0, 0.0)
        self.reset_quat_orientation_of_objects = Quaternion()
        self.reset_quat_orientation_of_objects.x = reset_quat_orientation_of_objects_list[0]
        self.reset_quat_orientation_of_objects.y = reset_quat_orientation_of_objects_list[1]
        self.reset_quat_orientation_of_objects.z = reset_quat_orientation_of_objects_list[2]
        self.reset_quat_orientation_of_objects.w = reset_quat_orientation_of_objects_list[3]

        # numbered tag model names
        self.numbered_tag_model_names = {
            1: "robot_tag_1",
            2: "robot_tag_2",
            3: "robot_tag_3"
        }

        # reset position and orientations of the numbered tags
        self.reset_numbered_tags_positions = [
            Point(x=-1.5, y=-1.0, z=0.1),
            Point(x=-1.5, y=0.0, z=0.1),
            Point(x=-1.5, y=1.0, z=0.1)
        ]
        self.reset_quat_orientation_of_numbered_tags = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # reset position and orientation of the robot
        self.robot_model_name = "robot"
        self.robot_reset_position = Point(x=0.0, y=0.0, z=0.0)
        self.robot_reset_orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)


        # goal locations
        self.goal_matches_objects_to_numbered_tags = {
            "pink": 3,
            "green": 1,
            "blue": 2
        }

        # current location of the numbered tags
        self.current_numbered_tags_locations = None

        # current locations of the robot objects relative to the tags
        self.current_robot_object_locations = {
            "pink": 0,
            "green": 0,
            "blue": 0
        }

        # flag to keep track of the state of when we're resetting the world and when we're not
        # to avoid sending too many duplicate messages
        self.reset_world_in_progress = False

        # keep track of the iteration number
        self.iteration_num = 0

        # ROS subscribers
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_received)

        # ROS publishers
        self.model_states_pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=10)
        self.reward_pub = rospy.Publisher("/q_learning/reward", QLearningReward, queue_size=10)

        self.run()


    def is_in_front_of_any_tag(self, pose):

        tag_id_in_front_of = 0

        for tag_id in range(1, 4):
            if (self.is_in_front_of_tag_id(pose, tag_id)):
                tag_id_in_front_of = tag_id

        return tag_id_in_front_of



    def is_in_front_of_tag_id(self, pose, tag_id):

        x = pose.position.x
        y = pose.position.y

        x_delta_range = 0.5

        y_delta_range = 0.4

        if (tag_id not in self.current_numbered_tags_locations):
            print("is_in_front_of_1_tag(): invalid tag_id")
            return False

        x_min = self.current_numbered_tags_locations[tag_id].x + 0.4
        x_max = x_min + x_delta_range
        y_min = self.current_numbered_tags_locations[tag_id].y - y_delta_range
        y_max = self.current_numbered_tags_locations[tag_id].y + y_delta_range

        if (x >= x_min and x <= x_max and y >= y_min and y <= y_max):
            return True
        else:
            return False


    def model_states_received(self, data):

        # get the initial locations of the three numbered tags
        if (self.current_numbered_tags_locations == None):
            self.current_numbered_tags_locations = {}
            for tag_id in self.numbered_tag_model_names:
                tag_idx = data.name.index(self.numbered_tag_model_names[tag_id])
                self.current_numbered_tags_locations[tag_id] = data.pose[tag_idx].position

        object_tag_mapping = {}

        for robot_object in self.robot_objects:
            object_idx = data.name.index(self.object_model_names[robot_object])
            object_tag_mapping[robot_object] = self.is_in_front_of_any_tag(data.pose[object_idx])

        # if a object has moved in front of a numbered tag, send a reward
        is_robot_object_update = False
        for robot_object in self.robot_objects:
            if (self.current_robot_object_locations[robot_object] != object_tag_mapping[robot_object]):
                is_robot_object_update = True
                # print(robot_object, " from ", self.current_robot_object_locations[robot_object], " to ", object_tag_mapping[robot_object])
                self.current_robot_object_locations[robot_object] = object_tag_mapping[robot_object]

        if (is_robot_object_update and not self.reset_world_in_progress):

            # assign reward
            reward_amount = -1
            reached_goal_locations = True
            for robot_object in self.robot_objects:
                if (self.goal_matches_objects_to_numbered_tags[robot_object] != object_tag_mapping[robot_object]):
                    reached_goal_locations = False

            if (reached_goal_locations):
                reward_amount = self.positive_reward_amount
            else:
                reward_amount = self.negative_reward_amount

            # publish reward
            reward_msg = QLearningReward()
            reward_msg.header = Header(stamp=rospy.Time.now())
            reward_msg.reward = reward_amount
            reward_msg.iteration_num = self.iteration_num
            self.reward_pub.publish(reward_msg)
            print("Published reward: ", reward_amount)




        # if all 3 objects are in front of numbered tag, reset the world
        objects_in_final_position = True
        for robot_object in self.robot_objects:
            if (object_tag_mapping[robot_object] == 0):
                objects_in_final_position = False

        if (objects_in_final_position and not self.reset_world_in_progress):

            self.reset_world_in_progress = True

            # reset world (objects position)
            shuffle(self.robot_objects)
            for i in range(len(self.robot_objects)):
                p = Pose(position=self.reset_xyz_positions_of_objects[i],
                         orientation=self.reset_quat_orientation_of_objects)
                t = Twist(linear=Vector3(0,0,0), angular=Vector3(0,0,0))
                m_name = self.object_model_names[self.robot_objects[i]]

                robot_object_model_state = ModelState(model_name=m_name, pose=p, twist=t)

                self.model_states_pub.publish(robot_object_model_state)

                self.current_robot_object_locations[self.robot_objects[i]] = 0

            # reset world (numbered tags positions)
            numbered_tags_position_order = [1, 2, 3]
            shuffle(numbered_tags_position_order)
            for i in range(len(numbered_tags_position_order)):
                p = Pose(position=self.reset_numbered_tags_positions[i],
                         orientation=self.reset_quat_orientation_of_numbered_tags)
                t = Twist(linear=Vector3(0,0,0), angular=Vector3(0,0,0))
                m_name = self.numbered_tag_model_names[numbered_tags_position_order[i]]

                numbered_tag_model_state = ModelState(model_name=m_name, pose=p, twist=t)

                self.model_states_pub.publish(numbered_tag_model_state)

                self.current_numbered_tags_locations[numbered_tags_position_order[i]] = self.reset_numbered_tags_positions[i]

            # reset world (robot position)
            p = Pose(position=self.robot_reset_position, orientation=self.robot_reset_orientation)
            t = Twist(linear=Vector3(0,0,0), angular=Vector3(0,0,0))
            robot_model_state = ModelState(model_name=self.robot_model_name, pose=p, twist=t)
            self.model_states_pub.publish(robot_model_state)


        elif (not objects_in_final_position and self.reset_world_in_progress):
            self.reset_world_in_progress = False
            self.iteration_num += 1




    def run(self):
        rospy.spin()





if __name__=="__main__":

    node = ResetWorld()