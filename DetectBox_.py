#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
import cv2
from cv_bridge import CvBridge
import numpy as np

desired_aruco_dictionary = "DICT_4X4_50"
bridge = CvBridge()

# The different ArUco dictionaries built into the OpenCV library.
ARUCO_DICT = {
  "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
  "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
  "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
  "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
  "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
  "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
  "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
  "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
  "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
  "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
  "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
  "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
  "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
  "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
  "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
  "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
  "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}

class Follower:
    def __init__(self):
        #cv2.namedWindow("original", 1)
        self.circlepoint = [0,0]
        # self.this_aruco_dictionary = cv2.aruco.get_Dictionary(ARUCO_DICT[desired_aruco_dictionary])
        self.this_aruco_dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[desired_aruco_dictionary])
        # self.this_aruco_parameters = cv2.aruco.DetectorParameters_create()
        self.this_aruco_parameters = cv2.aruco.DetectorParameters()

        self.image_sub = rospy.Subscriber('/camera/color/image_rect_color', 
                                            Image, 
                                            self.image_callback,
                                            queue_size=1,
                                            buff_size=2**24)
        # self.circle_sub = rospy.Subscriber('cone_vertix_pix', Int32MultiArray, self.circle_callback)
        self.cornerspix = Int32MultiArray()
        self.goal_pub = rospy.Publisher('marker_goal', Int32MultiArray, queue_size=10)


    def image_callback(self, msg):
        image = bridge.imgmsg_to_cv2(msg)
        image = cv2.cvtColor(image,cv2.COLOR_RGB2BGR )
        try:
            (corners, ids, rejected) = cv2.aruco.detectMarkers(image, self.this_aruco_dictionary, parameters=self.this_aruco_parameters)
            marker_centre = np.array([0.0, 0.0])
            print(corners)
            for i in range(0,4):
                marker_centre = marker_centre + np.array([corners[0][0,i,0], corners[0][0,i,1]])
            marker_centre = marker_centre/4.0
            image = cv2.circle(image, (int(marker_centre[0]), int(marker_centre[1])), radius=5, color=(0, 0, 0), thickness=2)
            image = cv2.circle(image, (int(corners[0][0,0,0]), int(corners[0][0,0,1])), radius=3, color=(255, 0, 0), thickness=2) #blue
            image = cv2.circle(image, (int(corners[0][0,1,0]), int(corners[0][0,1,1])), radius=6, color=(0, 255, 0), thickness=2) #green
            image = cv2.circle(image, (int(corners[0][0,2,0]), int(corners[0][0,2,1])), radius=9, color=(0, 0, 255), thickness=2) #red
            image = cv2.circle(image, (int(corners[0][0,3,0]), int(corners[0][0,3,1])), radius=12, color=(0, 0, 0), thickness=2)
            print('hey!')
            cv2.imshow("original", image)
            cv2.waitKey(3)
            print('hey2!')
            self.cornerspix.data = (int(corners[0][0,2,0]), int(corners[0][0,2,1]),
                                    int(corners[0][0,1,0]), int(corners[0][0,1,1]),
                                    int(marker_centre[0]), int(marker_centre[1]))
            self.goal_pub.publish(self.cornerspix)
            print('%s, %s' % (int(marker_centre[0]), int(marker_centre[1])))
            #cv2.waitKey(3)
        except:
            pass


if __name__ == '__main__':
    rospy.init_node('detect_box')
    detect_box = Follower()
    rospy.spin()
