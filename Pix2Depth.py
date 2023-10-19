#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Point
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge, CvBridgeError
import sys
import os
import numpy as np
import pyrealsense2 as rs2
if (not hasattr(rs2, 'intrinsics')):
    import pyrealsense2.pyrealsense2 as rs2

class ImageListener:
    def __init__(self, depth_image_topic, depth_info_topic):
        self.corner0pix = (0, 0)
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(depth_image_topic, msg_Image, self.imageDepthCallback)
        self.sub_info = rospy.Subscriber(depth_info_topic, CameraInfo, self.imageDepthInfoCallback)
        confidence_topic = depth_image_topic.replace('depth', 'confidence')
        self.sub_conf = rospy.Subscriber(confidence_topic, msg_Image, self.confidenceCallback)
        self.intrinsics = None
        self.pix = None
        self.pix_grade = None
        self.centrepix = (0, 0)
        self.corner1pix = (0, 0)
        self.corner2pix = (0, 0)
        # Publisher for coordinates of closer point
        self.centre = Point()
        self.centre.x = 0.0
        self.centre.y = 0.0
        self.centre.z = 0.0
        self.corner0 = Point()
        self.corner0.x = 0.0
        self.corner0.y = 0.0
        self.corner0.z = 0.0
        self.forward = Point()
        self.forward.x = 0.0
        self.forward.y = 0.0
        self.forward.z = 0.0
        # self.conepix = Int32MultiArray()
        self.pub_boxcentre = rospy.Publisher('centre', Point, queue_size=10)
        self.pub_boxcorner0 = rospy.Publisher('corner0', Point, queue_size=10)
        self.pub_forward = rospy.Publisher('forward', Point, queue_size=10)
        # self.conevertixpix = rospy.Publisher('cone_vertix_pix', Int32MultiArray, queue_size=10)
        self.sub_goal = rospy.Subscriber('marker_goal', Int32MultiArray, self.goalCallback)

    def goalCallback(self, msg):
        self.corner0pix = (msg.data[0], msg.data[1])
        self.corner1pix = (msg.data[2], msg.data[3])
        self.centrepix = (msg.data[4], msg.data[5])

    def imageDepthCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            # pick one pixel among all the pixels with the closest range:
            # indices = np.array(np.where(cv_image == cv_image[cv_image > 0].min()))[:,0]
            # pix = (indices[1], indices[0])
            # self.conepix.data = pix
            line = '\rDepth at pixel(%3d, %3d): %7.1f(mm).' % (self.centrepix[0], self.centrepix[1], cv_image[self.centrepix[1], self.centrepix[0]])
            # self.conevertixpix.publish(self.conepix)
            if self.intrinsics:
                # centre
                depth = cv_image[self.centrepix[1], self.centrepix[0]]
                result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [self.centrepix[0], self.centrepix[1]], depth)
                self.centre.x = result[0]
                self.centre.y = result[1]
                self.centre.z = result[2]
                self.pub_boxcentre.publish(self.centre)
                line += '  Coordinate: %8.2f %8.2f %8.2f.' % (result[0], result[1], result[2])
                # forward vector
                depth = cv_image[self.corner0pix[1], self.corner0pix[0]]
                result0 = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [self.corner0pix[0], self.corner0pix[1]], depth)
                self.corner0.x = result0[0]
                self.corner0.y = result0[1]
                self.corner0.z = result0[2]
                self.pub_boxcorner0.publish(self.corner0)
                line += '  corner0: %8.2f %8.2f %8.2f.' % (result0[0], result0[1], result0[2])
                depth = cv_image[self.corner1pix[1], self.corner1pix[0]]
                result1 = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [self.corner1pix[0], self.corner1pix[1]], depth)
                line += '  corner1: %8.2f %8.2f %8.2f.' % (result1[0], result1[1], result1[2])
                vec = np.array([result1[0]-result0[0], result1[1]-result0[1], result1[2]-result0[2]])
                # vec is published in frame O
                self.forward.x = -vec[1]
                self.forward.y = -vec[0]
                self.forward.z = 0.0
                self.pub_forward.publish(self.forward)

            if (not self.pix_grade is None):
                line += ' Grade: %2d' % self.pix_grade
            line += '\r'
            sys.stdout.write(line)
            sys.stdout.flush()

        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            return

    def confidenceCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            grades = np.bitwise_and(cv_image >> 4, 0x0f)
            if (self.pix):
                self.pix_grade = grades[self.pix[1], self.pix[0]]
        except CvBridgeError as e:
            print(e)
            return



    def imageDepthInfoCallback(self, cameraInfo):
        try:
            if self.intrinsics:
                return
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.K[2]
            self.intrinsics.ppy = cameraInfo.K[5]
            self.intrinsics.fx = cameraInfo.K[0]
            self.intrinsics.fy = cameraInfo.K[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs2.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.intrinsics.model = rs2.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in cameraInfo.D]
        except CvBridgeError as e:
            print(e)
            return

def main():
    # depth_image_topic = '/camera/depth/image_rect_raw'
    depth_image_topic = '/camera/aligned_depth_to_color/image_raw'
    # depth_info_topic = '/camera/depth/camera_info'
    depth_info_topic = '/camera/aligned_depth_to_color/camera_info'

    print ('')
    print ('show_center_depth.py')
    print ('--------------------')
    print ('App to demontrate the usage of the /camera/depth topics.')
    print ('')
    print ('Application subscribes to %s and %s topics.' % (depth_image_topic, depth_info_topic))
    print ('Application then calculates and print the range to the closest object.')
    print ('If intrinsics data is available, it also prints the 3D location of the object')
    print ('If a confedence map is also available in the topic %s, it also prints the confidence grade.' % depth_image_topic.replace('depth', 'confidence'))
    print ('')

    listener = ImageListener(depth_image_topic, depth_info_topic)
    rospy.spin()

if __name__ == '__main__':
    print('hi')
    obtain_depth = os.path.basename(sys.argv[0]).split('.')[0]
    rospy.init_node(obtain_depth)
    print('hi')
    main()
