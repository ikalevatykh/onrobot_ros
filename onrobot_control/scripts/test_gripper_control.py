#!/usr/bin/env python

import actionlib
import rospy
from control_msgs.msg import (GripperCommandAction, GripperCommandGoal)


def main():
    rospy.init_node('test_object_recognition')

    client = actionlib.SimpleActionClient(
        '/gripper_cmd', GripperCommandAction)
    client.wait_for_server()

    def gripper_cmd(position, effort):
        rospy.loginfo('Cmd:\nposition=%.2f, max_effort=%.2f', position, effort)

        goal = GripperCommandGoal()
        goal.command.position = position
        goal.command.max_effort = effort
        client.send_goal(goal)

        if client.wait_for_result():
            result = client.get_result()
            rospy.loginfo('Result:\n%s', result)

    gripper_cmd(position=1, effort=0)
    gripper_cmd(position=0, effort=0)
    gripper_cmd(position=1, effort=1)
    gripper_cmd(position=0, effort=1)


if __name__ == '__main__':
    main()
