#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def stop_robot():
    rospy.init_node('stop_robot', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    stop_twist = Twist()

    # Set all velocities to 0
    stop_twist.linear.x = 0.0
    stop_twist.linear.y = 0.0
    stop_twist.linear.z = 0.0
    stop_twist.angular.x = 0.0
    stop_twist.angular.y = 0.0
    stop_twist.angular.z = 0.0

    while not rospy.is_shutdown():
        pub.publish(stop_twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        stop_robot()
    except rospy.ROSInterruptException:
        pass

