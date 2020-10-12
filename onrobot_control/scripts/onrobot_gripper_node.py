#!/usr/bin/env python

import rospy
from onrobot_control import GripperControlServer


def main():
    rospy.init_node('onrobot_gripper', anonymous=True)
    GripperControlServer()
    rospy.spin()


if __name__ == '__main__':
    main()
