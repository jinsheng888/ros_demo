#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from yolov5_ros_msgs.msg import BoundingBox, BoundingBoxes
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

class ObjectFollower:
    def __init__(self):
        rospy.init_node('object_follower')
        rospy.Subscriber('/BoundingBoxes', BoundingBoxes, self.bounding_boxes_callback)
        self.patrol_pause_service = rospy.ServiceProxy('pause_patrol', Empty)
        self.patrol_continue_service = rospy.ServiceProxy('continue_patrol', Empty)
        self.approach_service = rospy.Service('approaching', Empty, self.approach_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.bridge = CvBridge()
        self.twist = Twist()
        self.x, self.y = None, None

        # threshold is measured by the square of boundingboxes
        self.box_area = 0
        self.distance_threshold = 1500
        self.stop_threshold = 49000  #50000
        self.bounding_box_count = 0

        self.approaching_flag = False
        self.finished_flag = False
        
        self.last_error = 0
        self.error = 0

    def approach_callback(self, req):
        self.approaching_flag = True
        self.finished_flag = False
        rate = rospy.Rate(10)
        while not self.finished_flag:
            rate.sleep()
        return []

    def bounding_boxes_callback(self, data):
        if data.bounding_boxes:
            self.bounding_box_count += 1
            if self.bounding_box_count >= 5 and self.approaching_flag:
                selected_box = max(data.bounding_boxes, key=lambda box: box.probability)
                center_x = (selected_box.xmin + selected_box.xmax) / 2
                center_y = (selected_box.ymin + selected_box.ymax) / 2
                box_area = (selected_box.xmax - selected_box.xmin) * (selected_box.ymax - selected_box.ymin)
                self.x, self.y = center_x, center_y
                rospy.loginfo("box_area: {}".format(box_area))
                self.box_area = box_area
                if box_area > self.stop_threshold:
                    rospy.loginfo("Target reached!")
                    self.finished_flag = True   
                    self.twist.linear.x = 0
                    self.twist.angular.z = 0
                    self.cmd_vel_pub.publish(self.twist)
                    self.finished_call()

                if  box_area > self.distance_threshold and not self.finished_flag:
                    try:
                        self.patrol_pause_service()
                        rospy.loginfo("Patrol paused, starting approach.")
                    except rospy.ServiceException as e:
                        rospy.logwarn("Service call failed: %s" % e)
                    self.approaching()

    def approaching(self):
        # rospy.loginfo("Approaching the target...")
        target_x = self.x
        target_y = self.y 
        c_x = 640 // 2
        c_y = 480 // 2
        # error_x = target_x - self.x
        # error_y = target_y - self.y
        self.error = c_x - target_x
        d_error = self.error - self.last_error
        kp, kd = .001, 0.001
        # self.twist.linear.x = 0.5
        if self.box_area < 5000:
            self.twist.linear.x = 0.66
        elif self.box_area < 11000 and self.box_area >= 7000:
            self.twist.linear.x = 0.4
        else:
            self.twist.linear.x = 0.2
            
        # if target_x < 80 or target_x > c_x - 80:
        #     self.twist.linear.x = 0.3

        rospy.loginfo("speed: {}".format(self.twist.linear.x))
        if self.error != 0:
            self.twist.angular.z = float(self.error)*kp + float(d_error)*kd
            rospy.loginfo("error: {}, angular: {}".format(self.error, self.twist.angular.z))
        else:
            self.twist.angular.z = 0
        self.last_error = self.error
        self.cmd_vel_pub.publish(self.twist)

    def finished_call(self):
        self.finished_flag = True
        rospy.loginfo("Finished!")

if __name__ == '__main__':
    follower = ObjectFollower()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if follower.approaching_flag == True:
            rospy.sleep(6)
            if follower.finished_flag == False:
                follower.patrol_continue_service()
    rospy.spin()