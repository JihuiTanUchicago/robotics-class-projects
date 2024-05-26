#!/usr/bin/env python3
import rospy
from robocourier.msg import RangeUpdate

class RangeUpdater(object):
    def __init__(self):
        rospy.init_node("range_updater")
        print("range updater called")
        # establish /q_learning/range_updater publisher
        self.range_pub = rospy.Publisher("/debug/range_update", RangeUpdate, queue_size=10)

        # keep track of current color range we're updating
        self.color = ""

        self.main_loop()

    def main_loop(self):
        colors = ["orange", "blue"]
        color = ""
        while True:
            color_input = input("List of colors:\n[0] Orange\n[1] Blue\nSelect a color: ")
            if color_input.isnumeric() and 0 <= int(color_input) <= 1:
                color = colors[int(color_input)]
                break
            else:
                print("Invalid choice!")

        # begin listening for color updates
        while True:
            choice = input("[0] lower range\n[1] upper range\nWhich range to update? ")
            if not choice.isnumeric() and (int(choice) != 0 and int(choice) != 1):
                print("Invalid number!")
            else:
                num = int(choice)
                ranges = input("Input a range in the format of 'H S V': ")
                if len(ranges.split(" ")) != 3:
                    print("invalid range!")
                else:
                    full_range = [int(num) for num in ranges.split(" ")]
                    message = RangeUpdate()
                    message.color = color
                    message.is_upper = bool(num)
                    message.range = full_range
                    self.range_pub.publish(message)
                    print("Range published!")

if __name__ == "__main__":
    print("starting range updater")
    node = RangeUpdater()
