#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3


class FollowWall:
    def __init__(self):
        rospy.init_node('follow_wall')
        
        # subscribe to /scan
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        # publish to /cmd_vel
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Create a default twist msg (all values 0).
        lin = Vector3()
        ang = Vector3()
        self.twist = Twist(linear=lin,angular=ang)

        self.cos45 = 0.525

        self.wall_distance = 0.6  # (in meter) as determining whether the robot is away from wall
        self.safe_distance = 0.43 # (in meter) as safe distance
    def scan_callback(self, data):
        # filter out zero values from ranges array
        non_zero_ranges = [distance for distance in data.ranges if distance > 0]
        # determine the closest object, the "person"
        min_distance = min(non_zero_ranges)
        min_index = data.ranges.index(min_distance)
        print(f"min_distance: {min_distance}")
        print(f"min_index: {min_index}")

        if min_distance > self.wall_distance:
            self.find_wall(data)
        else:
            self.follow_wall(data)

    def go_straight(self):
        # setup the Twist message we want to send
        my_twist = Twist(
            linear=Vector3(0.2, 0, 0),
            angular=Vector3(0, 0, 0)
        )
        print(f"going straight")
        # Publish msg to cmd_vel
        self.twist_pub.publish(my_twist)

    def go_right(self):
        # setup the twist message we want to send
        my_twist = Twist(
            linear=Vector3(),
            angular=Vector3(0,0,5)
        )
        print(f"going right")
        # Publish msg too cmd_vel
        self.twist_pub.publish(my_twist)
    
    def go_left(self):
        #setup the twist message we want to send
        my_twist = Twist(
            linear=Vector3(),
            angular=Vector3(0,0,-10)
        )
        print(f"going left")
        # Publish msg too cm_vel
        self.twist_pub.publish(my_twist)
        

    def follow_wall(self, data):
        """
        we split the robot into 4 regions: front, back, left, right
        if the front's distance average is less than the safe distance:
            if the left's distance average < right's distance average:
                turn right until left and back's distance averages are minimum, then go straight
            otherwise
                turn left until right and back's distance averages are minimum, then go straight
        otherwise
            if the left side is about to hit the wall but the front is farther than safe distance:
                turn right
            else if the right side is about to hit the wall but the front is farther than safe distance:
                turn left
            otherwise
                go straight safely
        """
        data_ranges = data.ranges
        data_length = len(data_ranges)
        # divide the data ranges to distinguish front, left, right, back's distance data
        front_data_ranges = data_ranges[7 * data_length//8 : -1] + data_ranges[0: data_length//8]
        right_data_ranges = data_ranges[data_length//4 : 3 * data_length//8]
        back_data_ranges = data_ranges[3 * data_length//8 : 5 * data_length//8]
        left_data_ranges = data_ranges[5 * data_length//8 : 7 * data_length//8]

        front_data_ranges = [distance for distance in front_data_ranges if distance > 0]
        right_data_ranges = [distance for distance in right_data_ranges if distance > 0]
        back_data_ranges = [distance for distance in back_data_ranges if distance > 0]
        left_data_ranges = [distance for distance in left_data_ranges if distance > 0]

        # based on the data, calculate each direction's average distance to determine what to do next
        front_distance_avg = sum(front_data_ranges) / len(front_data_ranges) if len(front_data_ranges) != 0 else float('inf')
        right_distance_avg = sum(right_data_ranges) / len(right_data_ranges) if len(right_data_ranges) != 0 else float('inf')
        back_distance_avg = sum(back_data_ranges) / len(back_data_ranges) if len(back_data_ranges) != 0 else float('inf')
        left_distance_avg = sum(left_data_ranges) / len(left_data_ranges) if len(left_data_ranges) != 0 else float('inf')
    
        print(f"front_dis: {front_distance_avg}")
        print(f"right_dis: {right_distance_avg}")
        print(f"back_dis: {back_distance_avg}")
        print(f"left_dis: {left_distance_avg}")

        # the following is the algorithm, see the function description for more details
        if front_distance_avg < self.safe_distance:
            if right_distance_avg >= left_distance_avg:
                #turn right until left and back's distance averages are minimum
                if left_distance_avg > front_distance_avg or left_distance_avg > right_distance_avg or back_distance_avg > front_distance_avg or back_distance_avg > right_distance_avg:
                    self.go_right()
                else:
                    self.go_straight()
            else:
                #turn left until right and back's distance averages are minimum
                if right_distance_avg > front_distance_avg or right_distance_avg > left_distance_avg or back_distance_avg > front_distance_avg or back_distance_avg > left_distance_avg:
                    self.go_left()
                else:
                    self.go_straight()
        else:
            """
            to determine a tricky case where the left or right is about to hit the wall,
            but the front is still larger than the safe distance,
            we estimate the distance from the corner of the robot to the wall:
            left corner to wall distance estimate 1: cos(45) * left_distance_avg
            left corner to wall distance estimate 2: cos(45) * front_distance_avg
            similarly, calculate right corner distance estimates 1 & 2

            The principle is that, if both estimates for the same corner is less than safe_distance,
            it is very likely that the robot needs to make a turn in the opposite direction immediately to prevent collision
            """
            if left_distance_avg * self.cos45 < self.safe_distance and front_distance_avg * self.cos45 < self.safe_distance:
                self.go_right()
            elif right_distance_avg * self.cos45 < self.safe_distance and front_distance_avg * self.cos45 < self.safe_distance:
                self.go_left()
            else:
                # go straight
                self.go_straight()
        
    def find_wall(self, data):
        # filter out zero values from ranges array
        non_zero_ranges = [distance for distance in data.ranges if distance > 0]
        # determine the closest object, the "person"
        min_distance = min(non_zero_ranges)
        min_index = data.ranges.index(min_distance)
        print(f"min_distance: {min_distance}")
        print(f"min_index: {min_index}")

        # determine if turning right(-1) or left(1)
        turn_direction = 1 if min_index <= len(data.ranges)/2 else -1
        
        # setup the Twist message we want to send
        my_twist = Twist(
            linear=Vector3(0.4, 0, 0) if min_distance >= self.safe_distance else Vector3(-0.1,0,0), #only move foward if greater than safe distance
            angular=Vector3(0, 0, data.angle_increment * turn_direction * 10)
        )
        print(f"my_twist: {my_twist}")

        
        # Publish msg to cmd_vel
        self.twist_pub.publish(my_twist)

    def run(self):
        # Keep the program alive
        rospy.spin()

if __name__ == '__main__':
    try:
        follower = FollowWall()
        follower.run()
    except rospy.ROSInterruptException:
        pass
