#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class DepthImageProcessor:
    def __init__(self):
        rospy.init_node('depth_image_processor')

        self.bridge = CvBridge()
        self.min_threshold = 0.45
        self.max_threshold = 6
        self.image = None
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.camera_callback)
        rospy.Subscriber('/camera/depth_registered/image_raw', Image, self.depth_callback)
        self.binary_pub = rospy.Publisher('/depth_proc/img_binary', Image, queue_size=1)
        self.image_pub = rospy.Publisher('/depth_proc/masked_img', Image, queue_size=1)

    def camera_callback(self, data):
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

    def depth_callback(self, data):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
            # the shape of received depth image is 400*640
            depth_image = np.pad(depth_image, ((40, 40), (0, 0)), mode='constant', constant_values=np.nan)
            binary_image = cv2.inRange(depth_image, self.min_threshold, self.max_threshold)
            binary_msg = self.bridge.cv2_to_imgmsg(binary_image, encoding="passthrough")
            if self.image is None:
                return
            image = self.image.copy()
            image[binary_image == 0] = 0
            image_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
            self.image_pub.publish(image_msg)
            self.binary_pub.publish(binary_msg)

        except CvBridgeError as e:
            rospy.logerr(e)

if __name__ == '__main__':
    processor = DepthImageProcessor()
    rospy.loginfo('init node.')
    rospy.spin()
