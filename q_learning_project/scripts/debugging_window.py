import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image

class Debugger():

    def __init__(self):
        cv2.namedWindow("window", 1)
        self.bridge = cv_bridge.CvBridge()
        # If running in real world, run rosrun image_transport republish compressed in:=raspicam_node/image raw out:=camera/rgb/image_raw  or change camera topic name  accordingly
        self.sub_img = rospy.Subscriber("camera/rgb/image_raw", Image, self.process_image)
        self.running = True
        self.display_image = None
        # AR dictionary
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

    def process_image(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # TODO Debugging
        self.display_image = image

    def run(self):
        while self.running:
            if self.display_image is not None:
                cv2.imshow("window", self.display_image)

            if cv2.waitKey(3) == ord('q'):
                self.running = False


if __name__ == "__main__":
    rospy.init_node("debugger")
    dbg = Debugger()
    dbg.run()
