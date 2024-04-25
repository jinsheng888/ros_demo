#!/usr/bin/env python
# coding=utf-8

import rospy
import time
import os
import sys
from sensor_msgs.msg import Image
import cv2
import cv_bridge
import numpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger, TriggerResponse


class Follower:

    def __init__(self):
        rospy.loginfo("Init line follower...")
        self.work_stage = 0   # 0:ready  1:working  2:finished   3:parking
        self.last_error = 0

        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.before_img_pub = rospy.Publisher("/line_follow/image", Image, queue_size=1)
        self.after_img_pub = rospy.Publisher("/line_follow/image_mask", Image, queue_size=1)
        self.service = rospy.Service('line_follow', Empty, self.handle_line_follow)
        self.service = rospy.Service('parking', Empty, self.handle_parking)
        self.twist = Twist()

    def image_callback(self, msg):
        if self.work_stage == 0 or self.work_stage == 2:
            return
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image = cv2.resize(image, (320, 240),
                           interpolation=cv2.INTER_AREA)  # 提高帧率
        h, w, d = image.shape
        old_image = image
        
        b, g, r = cv2.split(image)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        image = cv2.GaussianBlur(image, (7, 7), 0)
        img_show = self.bridge.cv2_to_imgmsg(image)
        self.before_img_pub.publish(img_show)

        _, mask = cv2.threshold(image, 70, 255, cv2.THRESH_BINARY)
        # mask[200:240, 0:40] = 255
        # mask[200:240, 280:320] = 255
        mask[200:240, :] = 255
        masked = cv2.bitwise_and(image, image, mask=mask)
        img_show = self.bridge.cv2_to_imgmsg(mask)
        self.after_img_pub.publish(img_show)

        kp, kd = 6, 30
        self.twist.linear.x = 0.36  # 0.18

        if self.work_stage == 3:    # parking backward
            self.twist.linear.x = -0.2
            kp, kd = -3, 0

        M = cv2.moments(mask)
        if M['m00'] < (h * w - 50) * 255:
            cx = M['m10']/M['m00']
            cy = M['m01']/M['m00']
            error =  cx - w/2
            d_error = error - self.last_error
            if error != 0:
                self.twist.angular.z = float(error)*kp + float(d_error)*kd
                rospy.loginfo("error: {}, angular: {}, area: {}".format(error, self.twist.angular.z, M['m00']))
            else:
                self.twist.angular.z = 0
            self.last_error = error
        else:
            self.twist.linear.x = 0.1
            self.twist.angular.z = 5
            if self.work_stage == 3:
                self.twist.angular.z = 0
        
        if self.check_stop(old_image):
            rospy.loginfo("line follow finished")
            self.twist.linear.x = 0.3
            self.twist.angular.z = 0   
            rate = rospy.Rate(10)
            for i in range(15):
                self.cmd_vel_pub.publish(self.twist)
                rate.sleep()
            # self.cmd_vel_pub.publish(self.twist)   
            # rospy.sleep(3.5)     
            self.work_stage = 2
            self.twist.linear.x = 0
            self.twist.angular.z = 0
        
        self.cmd_vel_pub.publish(self.twist)
        return True

    def handle_parking(self, req):
        self.work_stage = 3
        rospy.sleep(3)
        self.work_stage = 2
        return []

    def handle_line_follow(self, req):
        self.work_stage = 1
        rate = rospy.Rate(10)
        while self.work_stage == 1:
            rate.sleep()
        rospy.sleep(1.5)
        return []

    def check_stop(self, image, threshold=1000000):  #6000000
        col_green = (35, 43, 46, 77, 255, 255)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        v = cv2.equalizeHist(v)
        hsv = cv2.merge((h, s, v))
        kernel = numpy.ones((3, 3), numpy.uint8)
        hsv_erode = cv2.erode(hsv, kernel, iterations=1)
        hsv_dilate = cv2.dilate(hsv_erode, kernel, iterations=1)
        lowerbH = col_green[0]
        lowerbS = col_green[1]
        lowerbV = col_green[2]
        upperbH = col_green[3]
        upperbS = col_green[4]
        upperbV = col_green[5]
        mask = cv2.inRange(hsv_dilate, (lowerbH, lowerbS,
                           lowerbV), (upperbH, upperbS, upperbV))
        M = cv2.moments(mask)
        rospy.loginfo('green{}'.format(M['m00']))
        if M['m00'] > threshold:
            return True
        return False

        ''' old: use bgr channels to check
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        img_show = self.bridge.cv2_to_imgmsg(image)
        self.before_img_pub.publish(img_show)
        _, mask = cv2.threshold(image, 0, 200, cv2.THRESH_BINARY)
        masked = cv2.bitwise_and(image, image, mask=mask)
        img_show = self.bridge.cv2_to_imgmsg(mask)
        self.after_img_pub.publish(img_show)
        M = cv2.moments(mask)
        if M['m00'] > 100:
            return True
        return False
        '''


def test_module():
    rospy.wait_for_service('/line_follow')
    try:
        my_service = rospy.ServiceProxy('/line_follow', Empty)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s"%e)
    try:
        my_service()
        rospy.loginfo("Service call was successful")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s"%e)


if __name__ == '__main__':
    rospy.logwarn("Start...")
    rospy.init_node("line_follower")
    follower = Follower()
    rospy.loginfo('line_follow finished.')
    rospy.spin()
