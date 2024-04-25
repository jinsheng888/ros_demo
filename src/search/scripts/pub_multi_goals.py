#!/usr/bin/env python3
import rospy
import actionlib
import configparser
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from std_srvs.srv import Empty, Trigger, TriggerResponse

class PatrolBot:
    def __init__(self, config_file_path, start_config_path):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.config = configparser.ConfigParser()
        self.config.read(config_file_path)
        self.goals = self.load_goals(self.config)
        self.start_points = self.load_start_points(start_config_path)

        self.pause_service = rospy.Service('pause_patrol', Empty, self.handle_pause)
        self.start_service = rospy.Service('start_patrol', Empty, self.handle_start)
        self.start_service = rospy.Service('continue_patrol', Empty, self.handle_continue)
        self.return_service = rospy.Service('return_to_start', Empty, self.handle_return)

        self.paused = True
        self.current_goal_index = 0

    def load_start_points(self, start_config_file_path):
        start_config = configparser.ConfigParser()
        start_config.read(start_config_file_path)
        return self.load_goals(start_config)

    def load_goals(self, config):
        goals = []
        for section in config.sections():
            x = config.getfloat(section, 'x')
            y = config.getfloat(section, 'y')
            z = config.getfloat(section, 'z')
            qx = config.getfloat(section, 'qx')
            qy = config.getfloat(section, 'qy')
            qz = config.getfloat(section, 'qz')
            qw = config.getfloat(section, 'qw')
            pose = Pose(Point(x, y, z), Quaternion(qx, qy, qz, qw))
            goals.append(pose)
        return goals

    def send_goal(self, pose, timeout=20.0):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = pose
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration.from_sec(timeout))
        return self.client.get_state()

    def handle_pause(self, req):
        self.client.cancel_all_goals()
        self.paused = True
        rospy.loginfo('patrol paused ')
        self.current_goal_index += 1
        return []

    def handle_continue(self, req):
        if self.paused:
            self.paused = False
            self.current_goal_index = self.current_goal_index % len(self.goals)
        rospy.loginfo('continue patrol')
        return []

    def handle_start(self, req):
        if self.paused:
            self.paused = False
            self.current_goal_index = 0
        rospy.loginfo('start patrol')
        return []

    def handle_return(self, req):
        self.client.cancel_all_goals()
        self.paused = True
        for i in range(len(self.start_points)):
            result = self.send_goal(self.start_points[i], timeout=60.0)
        return []


def main():
    rospy.init_node('patrol_bot')
    path = rospy.get_param("~config_path")
    start_config_path = rospy.get_param("~start_config_path")
    patrol_bot = PatrolBot(path, start_config_path)

    while not rospy.is_shutdown():
        if not patrol_bot.paused:
            pose = patrol_bot.goals[patrol_bot.current_goal_index]
            result = patrol_bot.send_goal(pose)
            patrol_bot.current_goal_index = (patrol_bot.current_goal_index + 1) % len(patrol_bot.goals)

            if result != actionlib.GoalStatus.SUCCEEDED:
                rospy.logerr("Failed to reach the goal")
            else:
                rospy.loginfo("Goal reached successfully")
        else:
            rospy.sleep(0.5)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass