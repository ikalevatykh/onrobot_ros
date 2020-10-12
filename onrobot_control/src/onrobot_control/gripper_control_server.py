"""This module contains implementation of the GripperControlServer."""

import actionlib
import rospy
from control_msgs.msg import (GripperCommandAction, GripperCommandFeedback,
                              GripperCommandResult)
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF

from .gripper_controller import GripperController

__all__ = ('GripperControlServer')


class GripperControlServer:
    """The ObjectRecognitionServer class."""

    def __init__(self, gripper):
        """Constructor for a class GripperControlServer.

        Arguments:
            gripper {GripperController} -- gripper controller
        """
        self._state_publish_rate = rospy.get_param("~state_publish_rate", 50)
        self._action_monitor_rate = rospy.get_param('~action_monitor_rate', 20)
        self._joint_name = rospy.get_param('~joint', 'gripper_motor_joint')
        self._limit, self._mimics = _get_joint_info(self._joint_name)

        self._gripper = GripperController()

        self._server = actionlib.SimpleActionServer(
            'gripper_cmd', GripperCommandAction,
            execute_cb=self.execute, auto_start=True)

        self._joint_states_pub = rospy.Publisher(
            '/joint_states', JointState, queue_size=10)

        period = rospy.Duration(1.0 / self._state_publish_rate)
        self.timer = rospy.Timer(period, self.publish_state_cb)

    def execute(self, goal):
        """Action server callback.

        Gripper has only two commands: fully open, fully close.

        Args:
            goal (GripperCommandGoal): action goal
        """
        try:
            # gripper has only two commands: fully open, fully close
            close = goal.command.position == 0
            # we can choose from two effort modes
            low_force_mode = goal.command.max_effort == 0

            if close:
                self._gripper.close(low_force_mode, wait=False)
            else:
                self._gripper.open(low_force_mode, wait=False)

            rate = rospy.Rate(1.0 / self._action_monitor_rate)
            while self._server.is_active():
                width = self._gripper.opening * self._limit.upper
                force = self._limit.effort * (0.125 if low_force_mode else 1.0)

                if self._gripper.is_ready:
                    reached_goal = abs(goal.command.position - width) < 0.001
                    stalled = not reached_goal

                    result = GripperCommandResult(
                        position=width,
                        effort=force,
                        stalled=stalled,
                        reached_goal=reached_goal,
                    )
                    self._server.set_succeeded(result)
                    return

                feedback = GripperCommandFeedback(
                    position=width,
                    effort=force,
                    stalled=False,
                    reached_goal=False,
                )
                self._server.publish_feedback(feedback)

                rate.sleep()
        except Exception as ex:
            rospy.logerr('Griper commmand failed: %s', ex)
            self._server.set_aborted(text=str(ex))

    def publish_state_cb(self, timer):
        """Periodic task to publish the joint states message.

        Arguments:
            timer {Timer} -- timer object
        """
        try:
            width = self._gripper.opening * self._limit.upper

            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            msg.name.append(self._joint_name)
            msg.position.append(width)
            msg.velocity.append(0.0)
            msg.effort.append(0.0)

            for name, mimic in self._mimics.items():
                msg.name.append(name)
                msg.position.append(width * mimic.multiplier + mimic.offset)
                msg.velocity.append(0.0)
                msg.effort.append(0.0)

            self._joint_states_pub.publish(msg)
        except Exception as ex:
            rospy.logerr('Joint state publish failed: %s', ex)


def _get_joint_info(joint_name):
    limit = None
    mimics = {}

    robot = URDF.from_parameter_server()
    for joint in robot.joints:
        if joint.name == joint_name:
            limit = joint.limit
        elif joint.mimic is not None:
            if joint.mimic.joint == joint_name:
                mimics[joint.name] = joint.mimic

    if limit is None:
        raise RuntimeError(
            'Cannot find limits for joint "{}" in the robot description'.format(joint_name))

    return limit, mimics
