#!/usr/bin/env python2
import rospy
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
import time

def rotate_half_circle():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    twist = Twist()
    twist.angular.z = 1
    rospy.loginfo("begin to rotate...")
    pub.publish(twist)
    time_to_rotate = 3.14159 / twist.angular.z
    time.sleep(time_to_rotate)
    twist.angular.z = 0
    pub.publish(twist)
    rospy.loginfo("rotate finished.")


def call_service(service_name, service_type):
    rospy.wait_for_service(service_name)
    try:
        service_call = rospy.ServiceProxy(service_name, service_type)
        service_call()
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)


if __name__ == '__main__':
    rospy.init_node('service_caller')

    call_service('line_follow', Empty)
    rospy.loginfo(f"LineFollow Service: Success")

    call_service('start_patrol', Empty)
    rospy.loginfo(f"Patrol Service: Success")

    call_service('approaching', Empty)
    rospy.loginfo(f"Approaching Service: Success")
    rotate_half_circle()

    call_service('return_to_start', Empty)
    rospy.loginfo(f"Call service return to start: Success")

    # parking()