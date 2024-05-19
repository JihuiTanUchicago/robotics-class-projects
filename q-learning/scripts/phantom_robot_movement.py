#!/usr/bin/env python3

import rospy

from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from q_learning_project.msg import RobotMoveObjectToTag

import time

from tf.transformations import quaternion_from_euler, euler_from_quaternion


class RobotAction(object):

    def __init__(self, robot_object, goal_tag_num):
        self.robot_object = robot_object
        self.goal_tag_num = goal_tag_num

    def __str__(self):
        output_str = ("Robot action: move " + self.robot_object.upper() +
                      " to tag " + str(self.goal_tag_num))
        return output_str


class PhantomRobotMovement(object):

    def __init__(self):

        # initialize this node
        rospy.init_node('turtlebot3_phantom_movement')

        # ROS subscribe to the topic publishing actions for the robot to take
        rospy.Subscriber("/q_learning/robot_action", RobotMoveObjectToTag, self.prepare_to_take_robot_action)

        # ROS subscribe to the Gazebo topic publishing the locations of the models
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_received)

        # ROS publish to the Gazebo topic to set the locations of the models
        self.model_states_pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=10)

        # information about the robot action to take
        self.robot_action_queue = []

        # numbered tag model names
        self.numbered_tag_model_names = {
            1: "robot_tag_1",
            2: "robot_tag_2",
            3: "robot_tag_3"
        }

        # numbered tag locations + dimensions
        self.current_numbered_tag_locations = {}
        self.numbered_tag_side_length = 0.8

        # object model names
        self.object_model_names = {
            "pink": "robot_object_pink",
            "green": "robot_object_green",
            "blue": "robot_object_blue"
        }

        self.current_object_locations = {}

        quat_orientation_of_objects_list = quaternion_from_euler(1.5708, 0.0, 0.0)
        self.quat_orientation_of_objects = Quaternion()
        self.quat_orientation_of_objects.x = quat_orientation_of_objects_list[0]
        self.quat_orientation_of_objects.y = quat_orientation_of_objects_list[1]
        self.quat_orientation_of_objects.z = quat_orientation_of_objects_list[2]
        self.quat_orientation_of_objects.w = quat_orientation_of_objects_list[3]


    def execute_robot_action(self):

        time.sleep(0.5)

        if (len(self.robot_action_queue) > 0):

            robot_action_to_take = self.robot_action_queue[0]

            pt = Point(
                x=(self.current_numbered_tag_locations[robot_action_to_take.goal_tag_num].x + 0.6),
                y=self.current_numbered_tag_locations[robot_action_to_take.goal_tag_num].y,
                z=self.current_numbered_tag_locations[robot_action_to_take.goal_tag_num].z
            )

            print(robot_action_to_take)

            p = Pose(position=pt,
                     orientation=self.quat_orientation_of_objects)
            t = Twist(linear=Vector3(0,0,0), angular=Vector3(0,0,0))
            m_name = self.object_model_names[robot_action_to_take.robot_object]

            robot_object_model_state = ModelState(model_name=m_name, pose=p, twist=t)

            self.model_states_pub.publish(robot_object_model_state)

            # reset the flag and the action in the queue
            self.robot_action_queue.pop(0)


    def model_states_received(self, data):

        # if we have a robot action in our queue, get the locations of the
        # objects and numbered tags
        if (len(self.robot_action_queue) > 0):

            # get the numbered tag locations
            for tag_id in self.numbered_tag_model_names:
                tag_idx = data.name.index(self.numbered_tag_model_names[tag_id])
                self.current_numbered_tag_locations[tag_id] = data.pose[tag_idx].position

            # get the object locations
            for robot_object in self.object_model_names:
                object_idx = data.name.index(self.object_model_names[robot_object])
                self.current_object_locations[robot_object] = data.pose[object_idx].position

            self.execute_robot_action()



    def prepare_to_take_robot_action(self, data):
        # print(data)
        self.robot_action_queue.append(RobotAction(data.robot_object, data.tag_id))


    def run(self):
        rospy.spin()







if __name__=="__main__":

    node = PhantomRobotMovement()
    node.run()
