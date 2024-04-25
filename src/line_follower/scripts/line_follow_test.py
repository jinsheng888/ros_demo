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
last_erro = 0
cnt = 0


class Follower:

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        # 订阅rgb相机
        rospy.logwarn("Init...")
        # self.cap = cv2.VideoCapture(0)
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.before_img_pub = rospy.Publisher("before_img", Image, queue_size=1)
        self.after_img_pub = rospy.Publisher("after_img", Image, queue_size=1)
        # self.cmd_str_pub = rospy.Publisher("cmd_lf", String, queue_size=1)
        self.twist = Twist()
        self.max_stop = 10
        self.curent_tries = 0
        self.cmd_flag = False
        rospy.logwarn("init follower...")

    def image_callback(self, msg):
        global last_erro
        # rospy.logwarn("running...")
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # _, image = self.cap.read()
        # image = image[:h//2]
        # image[:, :, 0] = 255
        image[:, :, 1] = 60
        image = cv2.resize(image, (320, 240),
                           interpolation=cv2.INTER_AREA)  # 提高帧率
        h, w, d = image.shape
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # image = cv2.GaussianBlur(image, (7, 7), 0)
        w = image.shape[1]
        # image = cv2.blur(image, (7,7))
        img_show = self.bridge.cv2_to_imgmsg(image)
        self.before_img_pub.publish(img_show)
        _, mask = cv2.threshold(image, 70, 255, cv2.THRESH_BINARY)
        masked = cv2.bitwise_and(image, image, mask=mask)

#	cv2.imshow("before", mask)
        # 在图像某处绘制一个指示，因为只考虑20行宽的图像，所以使用numpy切片将以外的空间区域清空

        search_top = h-40
        search_bot = h
        kp, kd = 6, 30
        # kp, kd = 0.3, 0.2
        # cv2.imshow("after_masked", mask)
        # cv2.waitKey(1)
        img_show = self.bridge.cv2_to_imgmsg(mask)
        self.after_img_pub.publish(img_show)
        # mask[0:h//2, 0:w] = 0
	    # cv2.imshow("after_masked", mask)
        #mask[search_bot:h, 0:w] = 0
        # 计算mask图像的重心，即几何中心
        M = cv2.moments(mask)
        # if M['m00'] > 0:
        if M['m00'] < h * w * 255:
            # cx = int(M['m10']/M['m00'])
            cx = M['m10']/M['m00']
            # cy = int(M['m01']/M['m00'])
            cy = M['m01']/M['m00']
            # cv2.circle(image, (cx, cy), 10, (255, 0, 255), -1)
                # 计算图像中心线和目标指示线中心的距离
            erro =  cx - w/2
            d_erro = erro-last_erro
            self.twist.linear.x = 0.36  # 0.18
            if erro != 0:
                self.twist.angular.z = float(erro)*kp + float(d_erro)*kd
                rospy.logwarn("{}, {}".format(erro, self.twist.angular.z))
            else:
                self.twist.angular.z = 0
            last_erro = erro
            self.curent_tries = 0
        else:
            # rospy.loginfo('finished')
            # self.twist.linear.x = 0
            self.twist.linear.x = 0.1
            self.twist.angular.z = 5
        '''
        else:
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            self.curent_tries += 1
            if self.curent_tries >= self.max_stop and not self.cmd_flag:
                # os.chdir('/home/agilex/zycatkin_ws')
                #self.twist.angular.z = -.5
                #self.twist.linear.x = 0
                #times = 5
                # for i in range(times):
                #   self.cmd_vel_pub.publish(self.twist)
                #    rospy.logwarn("T: {}".format(i))
                #    time.sleep(.9)
                #self.twist.angular.z = 0
                # self.cmd_vel_pub.publish(self.twist)
                os.popen(
                    "gnome-terminal -e 'bash -c \"source /home/agilex/zycatkin_ws/devel/setup.bash; roslaunch scout_bringup open_rslidar.launch; \"'")
                os.popen(
                    "gnome-terminal -e 'bash -c \"source /home/agilex/zycatkin_ws/devel/setup.bash; roslaunch scout_bringup navigation_4wd.launch; \"'")
                # os.popen("gnome-terminal -e 'bash -c \"source /home/agilex/zycatkin_ws/devel/setup.bash; roslaunch usb_cam usb_cam-test.launch arg1:=/dev/video1; \"'")
                # os.popen("gnome-terminal -e 'bash -c \"source /home/agilex/zycatkin_ws/devel/setup.bash; roslaunch darknet_ros darknet_ros.launch; \"'")
                os.popen(
                    "/usr/bin/python /home/agilex/zycatkin_ws/src/obj_detector/template.py")
                # os.popen("/usr/bin/python /home/agilex/.ros/hello_world.py")
                # msg = os.popen("/usr/bin/python /home/agilex/.ros/hello_world.py")
                self.cmd_flag = True
                # rospy.logwarn("{}".format())
                rospy.signal_shutdown("line track closed.")
                return False
                time.sleep(1800)
        '''
        self.cmd_vel_pub.publish(self.twist)
        return True


if __name__ == '__main__':
    rospy.logwarn("Start...")
    rospy.init_node("line_follower")
    follower = Follower()
    # ret = True
    # while ret:
    #     ret = follower.image_callback()
    rospy.spin()
