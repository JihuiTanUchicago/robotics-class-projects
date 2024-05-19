#!/usr/bin/env python3

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, Vector3
from q_learning_project.msg import RobotMoveObjectToTag
from std_msgs.msg import String
import json

class ColorFinder:

        def __init__(self):

                rospy.init_node('color_finder')

                # set up ROS / OpenCV bridge
                self.bridge = cv_bridge.CvBridge()

                # initalize the debugging window
                self.latest_image = None
                self.new_image_flag = False
                cv2.namedWindow("window", 1)

                # This subscribes to the next move in main.py
                self.next_step = rospy.Subscriber('next_step', String, self.action_handler)

                # subscribe to the robot's RGB camera data stream
                self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                        Image, self.target_tag_or_object)
                self.need_object_sub = rospy.Subscriber('tag_or_object', String, self.update_need_object)
                # All of the movement publisher
                self.desired_tag = None
                self.desired_object = None
                
                self.tags_pub = rospy.Publisher('detection_info', String, queue_size=1)

                self.need_object = True #true if targeting object, false if targeting tag

        def update_need_object(self, data_no_use = None):
                self.need_object = not self.need_object
                print(f"need_object updated to {self.need_object}")
        def target_tag_or_object(self,msg):
                avg_x = self.get_color_object(msg) if self.need_object else self.get_ar_tag(msg)
                
                json_data = {'avg_x': str(avg_x)}
                print(f"json_data= {json_data}")
                self.tags_pub.publish(json.dumps(json_data)) 

        def get_ar_tag(self, msg):
                print("getting into get_ar_tag()")
                print("desired tag ", self.desired_tag)
                print("desired_object ", self.desired_object)
                # Creating an ar_tag_dict
                image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                gray_scale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
                print("Dict", self.dictionary)
                self.parameters = cv2.aruco.DetectorParameters()
                print("P/q_learning/robot_actionaram", self.parameters)
                self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
                print("detect", self.detector)
                # Grayscale for the aruco markers
                corners, ids, rejected_points = self.detector.detectMarkers(gray_scale)
                corners = list(corners)
                print("The corners",corners)
                try:
                        ids = ids.tolist()
                except Exception:
                        ids = []
                print("the ids", ids)

                corner_data = -1
                for i in range(len(ids)):
                        print(f"into for loop: {ids[i][0]}")
                        if ids[i][0] == self.desired_tag:
                                print("corner_data updated to positive")
                                corner_data = corners[i]
                print("Final corner data", corner_data)
                avg_x = numpy.mean(corner_data)       

                print("---------------START OF DATA------------------")
                print("Our ids: ", ids)
                print("Our corners: ", corners)
                print("Our rejects: ", rejected_points)
                print("----------------END OF DATA-------------")

                self.latest_image = image
                self.new_image_flag = True
                # cv2.imshow("window", image)
                # cv2.waitKey(3)   
                return avg_x

        def get_color_object(self, msg):
                print("getting into get_color_object")
                # converts the incoming ROS message to OpenCV format and HSV (hue, saturation, value)
                image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

                # BGR
                pink_bgr = numpy.uint8([[[113, 58, 150]]]) # 150,58,113
                green_bgr = numpy.uint8([[[72, 127, 109]]]) # 109,127,72
                blue_bgr = numpy.uint8([[[144, 122, 56]]])
                # HSV
                pink_hsv = cv2.cvtColor(pink_bgr,cv2.COLOR_BGR2HSV)
                green_hsv = cv2.cvtColor(green_bgr,cv2.COLOR_BGR2HSV)
                blue_hsv = cv2.cvtColor(blue_bgr,cv2.COLOR_BGR2HSV)
                # print("HSV", pink_hsv, green_hsv, blue_hsv)
                lower_pink = numpy.array([pink_hsv[0][0][0]-10, 100, 100])
                upper_pink = numpy.array([pink_hsv[0][0][0]+10, 255, 255])
                # print("PINK", pink_hsv_low, pink_hsv_high)
                lower_green = numpy.array([green_hsv[0][0][0]-10, 100, 100])
                upper_green = numpy.array([green_hsv[0][0][0]+10, 255, 255])
                # print("GREEN", green_hsv_low, green_hsv_high)
                lower_blue = numpy.array([blue_hsv[0][0][0]-10, 100, 100])
                upper_blue = numpy.array([blue_hsv[0][0][0]+10, 255, 255])
                
                # this erases all pixels that aren't colored
                mask = cv2.inRange(hsv, lower_pink, upper_pink)
                mask2 = cv2.inRange(hsv, lower_blue, upper_blue)
                mask3 = cv2.inRange(hsv, lower_green, upper_green)

                # using moments() function, the center of the color pixels is determined
                M = cv2.moments(mask)
                M2 = cv2.moments(mask2)
                M3 = cv2.moments(mask3)
                # if there are any color pixels found
                if self.desired_object == "pink" and M['m00'] > 0:
                        # center of the color pixels in the image
                        cx = int(M['m10']/M['m00'])
                        cy = int(M['m01']/M['m00'])
                        print('My cx', cx)
                        print('My cy', cy)

                        # a red circle is visualized in the debugging window to indicate
                        # the center point of the colored pixels
                        # hint: if you don't see a red circle, check your bounds for what is considered 'colored'
                        cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
                elif self.desired_object == "blue" and M2['m00'] > 0:
                        cx = int(M2['m10']/M2['m00'])
                        cy = int(M2['m01']/M2['m00'])
                        print('New bx', cx)
                        print('New by', cy)

                        cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

                elif self.desired_object == "green" and M3['m00'] > 0:
                        cx = int(M3['m10']/M3['m00'])
                        cy = int(M3['m01']/M3['m00'])
                        print('gr cy', cx)
                        print('gr cy', cy)

                        cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
                else:
                        print("color object not detected")
                        cx = -1

                # shows the debugging window
                # hint: you might want to disable this once you're able to get a red circle in the debugging window
                # cv2.imshow("window", image)
                # cv2.waitKey(3)
                self.latest_image = image
                self.new_image_flag = True

                return cx

        def action_handler(self, struct):
                print("We are getting the tag")
                struct = json.loads(struct.data)
                self.desired_object = struct['color_object']
                print("Our desired tag", self.desired_tag)
                self.desired_tag = int(struct['tag_id'])

        def run(self):
                rate = rospy.Rate(30)  # Set an appropriate rate (e.g., 30Hz)
                while not rospy.is_shutdown():
                        if self.new_image_flag:
                                cv2.imshow("window", self.latest_image)
                                cv2.waitKey(3)    
                                self.new_image_flag = False
                        rate.sleep()
                
if __name__ == '__main__':

       
        follower = ColorFinder()
        follower.run()