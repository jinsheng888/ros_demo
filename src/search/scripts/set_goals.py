#!/usr/bin/env python3
import rospy
import configparser
from geometry_msgs.msg import PoseStamped

class GoalSaver:
    def __init__(self, path):
        self.config = configparser.ConfigParser()
        self.config_file_path = path
        self.goal_subscriber = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        self.goal_count = 0

    def goal_callback(self, data):
        # 当接收到一个新的目标点时，保存到配置文件
        self.goal_count += 1
        section_name = 'Goal{}'.format(self.goal_count)
        self.config.add_section(section_name)
        self.config.set(section_name, 'x', str(data.pose.position.x))
        self.config.set(section_name, 'y', str(data.pose.position.y))
        self.config.set(section_name, 'z', str(data.pose.position.z))
        self.config.set(section_name, 'qx', str(data.pose.orientation.x))
        self.config.set(section_name, 'qy', str(data.pose.orientation.y))
        self.config.set(section_name, 'qz', str(data.pose.orientation.z))
        self.config.set(section_name, 'qw', str(data.pose.orientation.w))

        # 写入配置文件
        with open(self.config_file_path, 'w') as configfile:
            self.config.write(configfile)
        rospy.loginfo("Saved goal {} to config file".format(self.goal_count))

def main():
    rospy.init_node('goal_saver')
    path = rospy.get_param('~config_path', 'goals.ini')
    goal_saver = GoalSaver(path)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
