#!/usr/bin/env python3
import rospy
import actionlib
import configparser
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion

class PatrolBot:
    def __init__(self, config_file_path):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.config = configparser.ConfigParser()
        self.config.read(config_file_path)
        self.goals = self.load_goals()

    def load_goals(self):
        goals = []
        for section in self.config.sections():
            x = self.config.getfloat(section, 'x')
            y = self.config.getfloat(section, 'y')
            z = self.config.getfloat(section, 'z')
            qx = self.config.getfloat(section, 'qx')
            qy = self.config.getfloat(section, 'qy')
            qz = self.config.getfloat(section, 'qz')
            qw = self.config.getfloat(section, 'qw')
            pose = Pose(Point(x, y, z), Quaternion(qx, qy, qz, qw))
            goals.append(pose)
        return goals

    def send_goal(self, pose):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = pose
        self.client.send_goal(goal)
        # 设置超时时间
        self.client.wait_for_result(rospy.Duration.from_sec(20.0))
        return self.client.get_state()

def main():
    rospy.init_node('patrol_bot')
    path = rospy.get_param("~config_path") 
    patrol_bot = PatrolBot(path)
    rospy.loginfo("load goals...")
    rospy.loginfo(len(patrol_bot.goals))
    while not rospy.is_shutdown():
        for pose in patrol_bot.goals:
            result = patrol_bot.send_goal(pose)
            if result != actionlib.GoalStatus.SUCCEEDED:
                rospy.logerr("Failed to reach the goal")
            else:
                rospy.loginfo("Goal reached successfully")
        rospy.loginfo("Completed a patrol cycle, starting over...")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
