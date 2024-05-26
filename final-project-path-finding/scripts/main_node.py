#!/usr/bin/env python3
import json
import rospy, cv2, cv_bridge, moveit_commander
import numpy as np
import math
import os
from openai import OpenAI
import csv
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, Vector3
from robocourier.msg import RangeUpdate, RobotAction
from std_msgs.msg import Empty, String
import speak
import record_voice
import play_music

# Removed api key to push
client = OpenAI(api_key = "API key")
tools = [
            {
                "type": "function",
                "function": {
                    "name": "get_destination_node",
                    "description": "Output the destination node that the user specifies. It could only be one of the following nodes: A, B, C, D, E, F, G, H, I, J, K, L, M, N, O, P, Q, R, S, T, O, P, Q, R, S, T, U, V, W, X, Y, Z, AA, AB, AC, AD, AE, AF, AG, AH, AI, AJ, AK, AL, AM",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "destination_node": {
                                "type": "string",
                                "description": "Return only the string representation of a node. The function could ONLY output one of the following nodes: A, B, C, D, E, F, G, H, I, J, K, L, M, N, O, P, Q, R, S, T, O, P, Q, R, S, T, U, V, W, X, Y, Z, AA, AB, AC, AD, AE, AF, AG, AH, AI, AJ, AK, AL, AM. For example, if the user specifies the destination node to be \"AF\", output only the string \"AF\"",
                            }
                        },
                        "required": ["destination_node"]
                    },
                },
            }
        ]

# helper function convert node names to indices
def to_index(node_name):
    if len(node_name) == 2:
        return 26 + ord(node_name.lower()[1])-97
    elif len(node_name) == 1:
        return ord(node_name.lower())-97

# HSV color ranges for tape maze
color_ranges = {
    "orange": [np.array([0,35,204]), np.array([14,101,229])],
    "blue": [np.array([0,0,0]), np.array([0,0,0])]
}

# mapping of object nodes in map
object_nodes = {
    1: to_index("A"),
    2: to_index("B"),
    3: to_index("C")
}

dest_nodes = {
    1: to_index("Q"),
    2: to_index("X"),
    3: to_index("AB")
}

path_prefix = os.path.dirname(__file__) + "/"

# robot action order goes:
# await_action (waiting for robot to receive next object/target)
# calculate_obj_path (perform A* from robot's current position to object area)
# pursue_obj_area (follow calculated path to object area)
# one substate:
### drive_straight (for either driving forward or backward)
###########################################
# init_process_image (make camera send image to node)
# process_image (await image result from node)
###########################################
# calculate_target_path (perform A* from robot's current position to desired target)
# pursue_target (follow calculated path to target)
# drop_obj (place object down at target)
# (after this point, go back to await_action)

class RoboCourrier(object):
    def __init__(self):
        # initialize this node
        rospy.init_node("robocourier")

        ### ROBOT CONTROL VARIABLES ###
        # provide list of color ranges to be use/be updated
        self.position = to_index("K") # should be 32/AG for starting node
        self.direction = 0
        self.path = []
        self.path_index = 0
        self.object_mapping = {
            1: None,
            2: None,
            3: None
        }
        self.object_index = 1
        self.last_motion = None
        self.move_backward = False
        self.state = "await_action"
        self.twist = Twist()

        # read adjacency matrix file to initialize additional matrices
        self.init_matrices(path_prefix + "adjacency.csv")


        ########################################################################


        ### CONNECTIONS FOR ROBOT COMPONENTS ###
        # establish range_update subscriber
        rospy.Subscriber("/debug/range_update", RangeUpdate, self.handle_range)

        # establish /camera/rgb/image_raw subscriber
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.handle_image)
        self.bridge = cv_bridge.CvBridge()

        # establish /cmd_vel publisher
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # establish /image_input publisher, for running image recognition
        self.image_pub = rospy.Publisher("/robocourier/image_input", Image, queue_size=10)

        # establish /image_output subscriber, for receiving output of image recognition
        rospy.Subscriber("/robocourier/image_output", String, self.handle_image_output)

        # establish interface for group of joints making up robot arm
        self.robot_arm = moveit_commander.MoveGroupCommander("arm")

        # establish interface for group of joints making up robot gripper
        self.robot_gripper = moveit_commander.MoveGroupCommander("gripper")

        # move arm and gripper into ready position
        self.robot_arm.go([math.radians(0), math.radians(6), math.radians(38), math.radians(-52)], wait=True)

        self.gripper_close = [0.005, 0.005]
        self.gripper_open = [0.019, 0.019]

        self.robot_gripper.go(self.gripper_open)
        self.robot_gripper.stop()


        ########################################################################


        ### CONNECTIONS WITH ACTION MANAGER'S TOPICS ###
        # establish /robocourier/robot_action subscriber
        rospy.Subscriber("/robocourier/robot_action", RobotAction, self.handle_action)
        self.obj = ""
        self.tag = -1

        # establish /robocourier/state publisher
        self.state_pub = rospy.Publisher("/robocourier/state", Empty, queue_size=10)


        ########################################################################


        # send first ping message to action manager
        rospy.sleep(3)
        self.state_pub.publish(Empty())

        #rospy.spin()
        self.main_loop()


    # initialize adjacency and direction matrices from file
    def init_matrices(self, filename):
        # read file matrix into object
        with open(filename, 'r') as file:
            reader = csv.reader(file)
            matrix = list(reader)

        # initialize adjacency and direction matrices
        self.adj_matrix = []
        self.dir_matrix = []
        
        # iterate through every row in file matrix
        for row in matrix:
            adj_row = []
            dir_row = []

            # iterate through every cell in row
            for cell in row:
                # if cell is empty, append invalid values to adj/dir rows
                if cell == '[]':
                    adj_row.append(float('inf'))
                    dir_row.append(float('inf'))
                # otherwise, parse cell values
                else:
                    # convert string "[distance, direction]" into values
                    cell = cell[1:-1]
                    distance, direction = cell.split(",")
                    adj_row.append(float(distance))
                    dir_row.append(int(direction))
            
            # append adj/dir rows to matrices
            self.adj_matrix.append(adj_row)
            self.dir_matrix.append(dir_row)

        # manual overrides
        self.adj_matrix[to_index("N")][to_index("O")] += 15
        self.adj_matrix[to_index("O")][to_index("N")] += 15


    # handler for action manager
    def handle_action(self, data):
        print("received action {}".format(data))
        # check for empty action, signaling completion
        if data.obj == "":
            # set object and tag to empty values
            self.obj = ""
            self.tag = -1
        # otherwise, update robot's goals and begin journey to object area
        else:
            self.obj = str(data.obj)
            self.tag = int(data.tag)
            
            # transition robot to next state and calculate object path
            self.state = "calculate_obj_path"
            print("transitioning to calculate object path state and calculating path")

            # try and see if any of the nodes have been visited and have desired object
            # if not, visit first node at index
            object_node = object_nodes[self.object_index]
            for i in object_nodes:
                if object_nodes[i] == data.obj:
                    object_node = object_nodes[i]
                    break

            self.find_path(object_node)


    # handler for range updater
    def handle_range(self, data):
        color_ranges[data.color][int(data.is_upper)] = np.array(data.range)
        print("updated range to {}".format(data.range))


    # handler for image recognition output
    def handle_image_output(self, output):
        output = output.data
        print("soda detected in image is {}".format(output))
        print("desired soda is {}".format(self.obj))
        # check if received output is equal to desired object
        if output == self.obj:
            print("objects are the same!")
            # pick up object, which makes robot pursue target afterwards
            self.pick_up_object()

        # otherwise, make robot approach next target node and navigate
        else:
            print("objects are not the same!")
            # update object mapping for current object index, then increment index
            self.object_mapping[self.object_index] = output
            self.object_index += 1

            # error if the target wasn't found at the last node
            if self.object_index > 3:
                print("WARNING! Target not found anywhere!")
                self.state = "error"

            # otherwise, navigate to the next node
            else:
                # update state to calculate_obj_path and calculate path
                self.state = "calculate_obj_path"
                object_node = object_nodes[self.object_index]
                self.find_path(object_node)


    # handler for rbpi camera
    def handle_image(self, data):
        # perform action only when state is drive_straight
        #if self.state == "asdfjhalksdjgh":
        if self.state == "drive_straight":
            # convert ROS message to cv2 and hsv
            image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # retrieve lower and upper bounds for what to consider desired color
            # hardcoded color for now
            lower_range, upper_range = color_ranges["orange"]

            # remove all pixels that aren't within desired color range
            mask_inner = cv2.inRange(hsv, lower_range, upper_range)
            mask_full = mask_inner.copy()

            # limit pixel search for only pixels in center-bottom range
            h, w, d = image.shape
            middle_index = w // 2
            left = middle_index - 30
            right = middle_index + 30
            left_outer = middle_index - 60
            right_outer = middle_index + 60
            search_top = int(4*h/5)
            #search_bot = int(3*h/4 + 20)

            # for mask_inner, limit search for only pixels in center bottom range
            mask_inner[0:search_top, 0:w] = 0
            mask_inner[search_top:h, 0:left] = 0
            mask_inner[search_top:h, right:w] = 0

            # for mask_full, limit search for only pixels in bottom range
            mask_full[0:search_top, 0:w] = 0
            mask_full[search_top:h, 0:left_outer] = 0
            mask_full[search_top:h, right_outer:w] = 0

            # use moments() function to find center of path pixels
            M_inner = cv2.moments(mask_inner)
            M_full = cv2.moments(mask_full)

            # prioritize center pixel range; otherwise search everywhere else as backup
            cx = -1
            cy = -1
            # check if there were any inner-path pixels
            if M_inner['m00'] > 0:
                # center of detected pixels in the image
                cx = int(M_inner['m10']/M_inner['m00'])
                cy = int(M_inner['m01']/M_inner['m00'])
            # check if there were any pixels at all
            elif M_full['m00'] > 0:
                cx = int(M_full['m10']/M_full['m00'])
                cy = int(M_full['m01']/M_full['m00'])

            # check if pixels were detected at all
            if cx != -1:
                # draw circle at center of pixels in debugging window
                cv2.circle(image, (cx, cy), 20, (0,0,255),-1)

                # make bot turn depending on how far away center of pixels are from center
                e_t = (w // 2) - cx
                k_p = 0.3 / (w // 2)

                # invert the correction if bot is driving backwards
                if self.move_backward:
                    self.twist.angular.z = -e_t * k_p
                else:
                    self.twist.angular.z = e_t * k_p

                # check once again to make sure robot hasn't transitioned back to pursue_obj_area
                if self.state != "drive_straight":
                    # if it has, cancel the movement
                    self.vel_pub.publish(Twist())

                # otherwise, make robot move as normal
                else:
                    self.vel_pub.publish(self.twist)
            # otherwise, just drive straight
            else:
                self.twist.angular.z = 0
                self.vel_pub.publish(self.twist)

            # show debugging window
            #cv2.imshow("window", image)
            #cv2.waitKey(3)

        # if robot is in init_process_image state, send image to image recognition node
        elif self.state == "init_process_image":
            # transition state to process_image to prevent further processing
            self.state = "process_image"
            
            # publish image to image recognition node
            self.image_pub.publish(data)


    # perform dijkstra's algorithm from self.position to destination
    def find_path(self, destination):
        # initialize arrays for dijkstra's algorithm
        source = self.position
        n = len(self.adj_matrix)
        dist = [float('inf')] * n
        dist[source] = 0
        visited = [False] * n
        predecessor = [-1] * n

        # perform dijkstra's algorithm
        for _ in range(n):
            # find vertex with the minimum distance from the set of vertices not yet visited
            u = min((v for v in range(n) if not visited[v]), key=lambda v: dist[v])
            visited[u] = True

            # update distance of the adjacent vertices of the selected vertex
            for v in range(n):
                if self.adj_matrix[u][v] > 0 and not visited[v] and dist[v] > dist[u] + self.adj_matrix[u][v]:
                    dist[v] = dist[u] + self.adj_matrix[u][v]
                    predecessor[v] = u

        # reconstruct the shortest path from source to destination using the predecessor array
        path = []
        step = destination
        while step != -1:
            # add current step to array and backtrack w/predecessor array
            path.append(step)
            step = predecessor[step]

            # if backtrack leads to source, end loop
            if step == source:
                path.append(step)
                break
        
        # update self.path with the path array reversed so robot can follow it to destination array
        self.path = path[::-1]
        self.path_index = 0

        # after path is calculated, transition to pursue_obj_area state
        print("path calculated; transitioning to pursue object area state")
        print("path is {}".format(self.path))
        
        # update state depending on whether pursuing object area or target
        if self.state == "calculate_obj_path":
            self.state = "pursue_obj_area"
        elif self.state == "calculate_target_path":
            self.state = "pursue_target"

        play_music.play()

    # main thread loop, used for managing time for driving straight and turning
    def main_loop(self):
        while not rospy.is_shutdown():
            # if state is pursue_obj_area, calculate target and transition state accordingly
            if self.state == "pursue_obj_area" or self.state == "pursue_target":
                # determine next action
                action = self.get_next_action()
                
                # if robot should drive forward or back, update state accordingly
                if action == "forward" or action == "back":
                    previous_state = self.state
                    self.last_motion = action
                    print("robot driving straight!")
                    # update twist linear velocity for camera function
                    self.move_backward = action == "back"
                    if self.move_backward:
                        self.twist.linear.x = -0.1
                    else:
                        self.twist.linear.x = 0.1
                    self.twist.angular.z = 0
                    
                    # since distances are reported in centimeters, also report speed in cm/s
                    print("length of path is {}".format(len(self.path)))
                    print("path index is {}".format(self.path_index))
                    next_node = self.path[self.path_index + 1]
                    dist = self.adj_matrix[self.position][next_node]
                    assert(dist != float('inf'))
                    #time_to_wait = dist / 10 * 1.05
                    time_to_wait = dist / 10
                    print("distance is {}, driving for {} seconds".format(dist, time_to_wait))

                    # transition state to drive_straight to allow camera to process orange pixels
                    self.state = "drive_straight"

                    # drive forward
                    self.vel_pub.publish(self.twist)

                    # sleep so robot can drive to target in distance
                    rospy.sleep(time_to_wait)

                    # stop robot
                    self.state = previous_state
                    self.twist.linear.x = 0
                    self.twist.angular.z = 0
                    self.vel_pub.publish(self.twist)

                    # update robot position 
                    self.path_index += 1
                    self.position = self.path[self.path_index]

                    # check if robot has reached end of path
                    if self.path_index == len(self.path) - 1:
                        # if it has, turn robot left if pursuing object area
                        if self.state == "pursue_obj_area":
                            # transition to scan for object state
                            self.state = "init_process_image"
                            #pass
                            #self.pick_up_object()
                        # otherwise, turn the robot towards target and put down
                        elif self.state == "pursue_target":
                            # TODO: turn robot towards target

                            # put object down
                            self.state = "put_obj_down"
                            self.put_down_object()

                    # otherwise, continue along path
                    else:
                        # transition state back to previous state to reenter loop
                        self.state = previous_state

                    # sleep briefly so bot doesn't lose distance
                    rospy.sleep(0.5)

                # if robot should turn left or right, do so here
                elif action == "left" or action == "right":
                    print("robot turning {}!".format(action))
                    self.turn_robot(action)

        # make robot stop after loop terminates from ctrl+C
        self.vel_pub.publish(Twist())


    # turn robot 90 degrees to left or right
    def turn_robot(self, direction):
        # make robot drive either forwards or backwards slightly
        self.twist.linear.x = 0.05
        self.twist.angular.z = 0
        self.vel_pub.publish(self.twist)
        rospy.sleep(0.5)
        self.vel_pub.publish(Twist())

        # update twist value
        self.twist.linear.x = 0
        if direction == "left":
            self.twist.angular.z = 0.2
            self.direction = (self.direction - 90) % 360
        else:
            self.twist.angular.z = -0.2
            self.direction = (self.direction + 90) % 360

        # make robot turn for set amount of time, then stop
        self.vel_pub.publish(self.twist)
        rospy.sleep(8.52)
        self.twist.angular.z = 0
        self.vel_pub.publish(self.twist)


    # pick up an object directly in front of robot
    def pick_up_object(self):
        # TODO: reimplement this for soda bottles

        # close gripper after stopping to grab object
        self.robot_arm.go([math.radians(0), math.radians(43), math.radians(3), math.radians(-45)], wait=True)
        rospy.sleep(4)
        
        self.robot_gripper.go(self.gripper_close)
        self.robot_gripper.stop()
        rospy.sleep(1)

        # raise arm after picking up object
        self.robot_arm.go([math.radians(0), math.radians(-80), math.radians(38), math.radians(-52)], wait=True)
        rospy.sleep(2)
        play_music.stop()
        arrive_ack = "I have picked up the drink! Where do you want me to go?"
        speak.speak_text(arrive_ack)
        print(arrive_ack)
        has_destination = False
        while has_destination == False and not rospy.is_shutdown():
            # check if index isn't out of range
            record_voice.record_voice()
            audio_file= open("output.mp3", "rb")
            translation = client.audio.translations.create(
                model="whisper-1",
                file=audio_file
            )
            audio_file.close()
            user_message = translation.text
            print(f"user_message is: {user_message}")
            completion = client.chat.completions.create(
                    model="gpt-4o",
                    messages=[
                        {"role": "system", "content": "You are asking which destination node the user wants you to go. Pay close attention if the user specifies a destination node within the range A-Z and AA-AM. Also, just be a funny AI that can amuse the user."},
                        {"role": "user", "content": user_message}
                    ],
                    tools=tools,
                    tool_choice="auto"
            )
            response_message = completion.choices[0].message
            tool_calls = response_message.tool_calls
            if tool_calls:
                # transition to calculate_target_path after grabbing robot
                dest_node = json.loads(tool_calls[0].function.arguments).get("destination_node")
                print(f"{dest_node}, the destination is {dest_node}! Here I come!")
                speak.speak_text(f"{dest_node}, the destination is {dest_node}! Here I come!")
                #calculate path from current position to tag node
                speak.speak_text("calculating the quickest path to your place")
                print("calculating the quickest path to your place")
                self.state = "calculate_target_path"
                self.find_path(to_index(dest_node))
                has_destination = True
            else:
                print(completion.choices[0].message.content)
                speak.speak_text(completion.choices[0].message.content)



    # put down object directly in front
    def put_down_object(self):
        play_music.stop()
        speak.speak_text("Here you go!")
        # TODO: implement this
        self.robot_arm.go([math.radians(0), math.radians(6), math.radians(38), math.radians(-52)], wait=True)
        rospy.sleep(8)
        self.robot_gripper.go(self.gripper_open)
        self.robot_gripper.stop()
        rospy.sleep(1)
        # transition state back to await_action
        # and reset object_index
        self.state = "await_action"
        self.object_index = 1

        # notify action manager to receive next action
        self.state_pub.publish(Empty())


    # using current position+rotation, determine whether bot should drive straight (forward or back)
    # or turn left/right
    def get_next_action(self):
        # make sure robot position and node at path index are the same
        assert(self.position == self.path[self.path_index])

        # init some variables
        cur_node = self.position
        cur_dir = self.direction

        # get next node in path and direction needed to get there
        next_node = self.path[self.path_index + 1]
        next_dir = self.dir_matrix[cur_node][next_node]
        
        print("robot direction is {}; goal direction is {}".format(cur_dir, next_dir))

        # compare current direction and necessary direction
        # if current and necessary direction are the same, robot should drive forward
        if self.direction == next_dir:
            return "forward"
        # if current and necessary direction are 180 deg apart, robot should drive backwards
        elif abs(self.direction - next_dir) == 180:
            return "back"
        # if necessary direction is 90 degrees from current direction, robot should turn right
        elif (self.direction + 90) % 360 == next_dir:
            return "right"
        # otherwise, turn left
        else:
            return "left"

if __name__ == "__main__":
    node = RoboCourrier()

