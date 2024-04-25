#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped

def create_goal(x, y, yaw):
    # 创建一个新的目标
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # 设置目标位置
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0.0

    # 将欧拉角转换为四元数，此处为简单起见直接设置yaw
    from tf.transformations import quaternion_from_euler
    q = quaternion_from_euler(0, 0, yaw)
    goal.target_pose.pose.orientation.x = q[0]
    goal.target_pose.pose.orientation.y = q[1]
    goal.target_pose.pose.orientation.z = q[2]
    goal.target_pose.pose.orientation.w = q[3]

    return goal

def main():
    rospy.init_node('patrol')

    # 创建SimpleActionClient，连接到move_base服务器
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # 定义一系列目标点
    waypoints = [
        (2.0, 2.0, 0.0),
        (2.0, -2.0, 1.57),
        (-2.0, -2.0, 3.14),
        (-2.0, 2.0, -1.57)
    ]

    # 依次导航到每个目标点
    for wp in waypoints:
        goal = create_goal(wp[0], wp[1], wp[2])
        client.send_goal(goal)
        client.wait_for_result()
        
        # 检查是否成功到达目标点
        if client.get_state() != actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Failed to reach the destination.")
            break
        rospy.loginfo("Successfully reached the destination.")

    rospy.loginfo("Patrol complete.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

