#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3


class FollowPerson:
    def __init__(self):
        rospy.init_node('follow_person')
        
        # subscribe to /scan
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        # publish to /cmd_vel
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Create a default twist msg (all values 0).
        lin = Vector3()
        ang = Vector3()
        self.twist = Twist(linear=lin,angular=ang)

        # safe distances
        self.safe_distance = 0.4  # (in meter) as safe distance
        
    def scan_callback(self, data):
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
            linear=Vector3(1.0, 0, 0) if min_distance >= self.safe_distance else Vector3(-0.1,0,0), #only move foward if greater than safe distance
            angular=Vector3(0, 0, data.angle_increment * turn_direction * 100)
        )
        print(f"my_twist: {my_twist}")

        
        # Publish msg to cmd_vel
        self.twist_pub.publish(my_twist)

    def run(self):
        # Keep the program alive
        rospy.spin()

if __name__ == '__main__':
    try:
        follower = FollowPerson()
        follower.run()
    except rospy.ROSInterruptException:
        pass
