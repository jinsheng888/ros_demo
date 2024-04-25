#!/usr/bin/env python
import rospy
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
import time
from PythonAPI import *

def rotate_half_circle(times=25):
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    twist = Twist()
    twist.angular.z = 0.5
    rospy.loginfo("begin to rotate...")
    rate = rospy.Rate(10)
    for i in range(times):
        pub.publish(twist)
        rate.sleep()
    twist.angular.z = 0
    pub.publish(twist)
    rospy.loginfo("rotate finished.")

def backward(times=15):
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    twist = Twist()
    twist.angular.z = 0
    twist.linear.x = -0.5
    rospy.loginfo("begin to go backward...")
    rate = rospy.Rate(10)
    for i in range(times):
        pub.publish(twist)
        rate.sleep()
    twist.angular.z = 0
    twist.linear.x = 0
    pub.publish(twist)
    rospy.loginfo("back finished.")

def forward(speed=0.5, times=10):
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    twist = Twist()
    twist.angular.z = 0
    twist.linear.x = speed
    rospy.loginfo("begin to go forward...")
    rate = rospy.Rate(10)
    for i in range(times):
        pub.publish(twist)
        rate.sleep()
    twist.angular.z = 0
    twist.linear.x = 0
    pub.publish(twist)
    rospy.loginfo("back finished.")


def call_service(service_name, service_type):
    rospy.wait_for_service(service_name)
    try:
        service_call = rospy.ServiceProxy(service_name, service_type)
        service_call()
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

def move_arms():
    open_port('/dev/ttyUSB0',115200)
    rospy.logwarn('init port')
    rospy.sleep(1)
    ping(1)
    get_version()
    set_control_signal(1)


if __name__ == '__main__':
    rospy.init_node('service_caller')
    # move_arms()
    rospy.loginfo('master.')
    call_service('pause_patrol', Empty)     # inroder to cancel all goals
    rospy.loginfo("LineFollow Service: Start")
    call_service('line_follow', Empty)
    rospy.loginfo("LineFollow Service: Success")
    rotate_half_circle(20)

    rospy.loginfo("Patrol Service: Start")
    call_service('start_patrol', Empty)
    rospy.loginfo("Patrol Service: Success")

    rospy.loginfo("Approaching Service: Start")
    call_service('approaching', Empty)
    rospy.loginfo("Approaching Service: Success")
    rospy.sleep(1)
    backward()
    rotate_half_circle()

    call_service('return_to_start', Empty)
    rospy.loginfo("Call service return to start: Success")
    forward(0.2, 30)
    # backward(30)
    # forward()
    # rospy.loginfo("begin to exec parking")
    # call_service('parking', Empty)
    # rospy.loginfo("Call service return to start: Success")