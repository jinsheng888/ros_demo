#!/usr/bin/env python
# coding=utf-8

import math
import rospy
from sensor_msgs.msg import Image
import cv2
import cv_bridge
import numpy as np
from yolov5_ros_msgs.msg import BoundingBox

HSV = [
    (15, 43, 20),
    (35, 111, 255)
]
CUT = 240


def get_target(image):
    # image = image[CUT:]

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, *HSV)
    mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE,
                            kernel=(3, 3), iterations=3)
    # cnts, _ = cv2.findContours(mask, mode=cv2.RETR_LIST, method=cv2.CHAIN_APPROX_NONE)
    xywh = []
    M = cv2.moments(mask)
    cx = M['m10'] / M['m00']
    cy = M['m01'] / M['m00']
    # for cnt in cnts:
    # rospy.logwarn("shape inside {}".format(cnt.shape))
    #    x, y, w, h = cv2.boundingRect(np.array(cnt))
    #    area = cv2.contourArea(cnt)
    area = (mask > 0).sum()
    w = h = math.sqrt(area)

    return (cx - w/2, cy - h/2, w, h), mask


class Follower:

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        # 订阅rgb相机
        rospy.logwarn("Init...")
        self.image_sub = rospy.Subscriber(
            "/camera/rgb/image_raw", Image, self.image_callback)
        self.det_pub = rospy.Publisher('detector', BoundingBox, queue_size=1)
        self.box_pub = rospy.Publisher('bounding_box', Image, queue_size=1)
        self.det_msg = BoundingBox()
        self.det_msg.Class = 'chair'

    def image_callback(self, msg):
        #        rospy.logwarn("running...")
        #        rospy.logwarn("encoding: {}\nheader: {}\nh, w: {}, {}\ndata: {}".format(
        #             msg.encoding, msg.header, msg.height, msg.width, msg.data
        #         ))
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        try:
            x = y = w = h = 0
            xywh, mask = get_target(image)
            x, y, w, h = xywh
            self.det_msg.xmin = x
            self.det_msg.xmax = x + w
            self.det_msg.ymin = y
            self.det_msg.ymax = y + h
            image = cv2.rectangle(image, (int(x), int(y)), (int(x + w),
                          int(y + h)), color=(255, 0, 0), thickness=3)
        except Exception as e:
            self.det_msg.xmin = 0
            self.det_msg.xmax = 0
            self.det_msg.ymin = 0
            self.det_msg.ymax = 0
        # cv2.imshow("p", image)
        # cv2.waitKey(1)
        self.det_pub.publish(self.det_msg)
        self.publish_image(image)
    
    def publish_image(self, imgdata, height=480, width=640):
        image_temp = Image()
        header = Header(stamp=rospy.Time.now())
        header.frame_id = self.camera_frame
        image_temp.height = height
        image_temp.width = width
        image_temp.encoding = 'bgr8'
        image_temp.data = np.array(imgdata).tobytes()
        image_temp.header = header
        image_temp.step = width * 3
        self.image_pub.publish(image_temp)

        compressed_image_msg = CompressedImage()
        compressed_image_msg.header = header
        compressed_image_msg.format = "jpeg"
        compressed_image_msg.data = np.array(cv2.imencode('.jpg', imgdata)[1]).tostring()
        self.image_compressed.publish(compressed_image_msg)


if __name__ == '__main__':
    rospy.logwarn("Start...")
    rospy.init_node("hsv_detector")
    follower = Follower()
    rospy.spin()
