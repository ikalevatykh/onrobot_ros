"""This module contains implementation of the GripperController."""

import rospy
from rospy.exceptions import ROSException, ROSInterruptException
from std_msgs.msg import String
from ur_msgs.msg import IOStates, ToolDataMsg
from ur_msgs.srv import SetIO

__all__ = ('GripperController')


class GripperController:
    """The GripperController class."""

    def __init__(self, enable=True):
        """Constructor for a class GripperController.

        Keyword Arguments:
            enable {bool} -- enable gripper (default: {True})
        """
        self._tool_voltage = 0
        self._ready = False
        self._state = 0

        self._position_voltage = 0.0
        self._max_position_voltage = rospy.get_param(
            '~max_position_voltage', 3.0)

        ns = rospy.get_param('~ur_hardware_interface',
                             '/ur_hardware_interface').rstrip('/ ')

        self._set_io = rospy.ServiceProxy('{}/set_io'.format(ns), SetIO)
        self._set_io.wait_for_service(timeout=5.0)

        self._io_states_sub = rospy.Subscriber(
            '{}/io_states'.format(ns), IOStates, self._io_states_cb, queue_size=10)

        self._tool_data_sub = rospy.Subscriber(
            '{}/tool_data'.format(ns), ToolDataMsg, self._tool_data_cb, queue_size=10)

        self._script_command_pub = rospy.Publisher(
            '{}/script_command'.format(ns), String, queue_size=2, latch=True)

        if enable:
            self.enable()

    def enable(self):
        """Enable gripper.

        Set the UR tool voltage to 24V and wait until the gripper becomes ready.

        Raises:
            ROSException: timeout exception
        """
        self._set_tool_voltage(24)
        if not self._wait_for(lambda: self._tool_voltage == 24 and self._ready):
            raise ROSException('Cannot enable the gripper')

    def disable(self):
        """Disable gripper.

        Set the tool voltage to zero.

        Raises:
            ROSException: timeout exception
        """
        self._set_tool_voltage(0)
        if not self._wait_for(lambda: self._tool_voltage == 0):
            raise ROSException('Cannot disable the gripper')

    @property
    def is_ready(self):
        """Gripper is ready to receive a command.

        Returns:
            bool -- ready flag
        """
        return self._ready

    @property
    def opening(self):
        """Fingers opening in percents.

        Returns:
            float -- opening 0..1 such that 0 - fully closed, 1 - fully opened
        """
        return max(0.0, min(1.0, self._position_voltage / self._max_position_voltage))

    def open(self, low_force_mode=False, wait=True):
        """Open the gripper.

        Keyword Arguments:
            low_force_mode {bool} -- use the low force mode (5N instead of 40N) (default: {False})
            wait {bool} -- wait until the end of the movement (default: {False})

        Raises:
            ROSException: timeout exception
        """
        self._move(0, low_force_mode, wait)

    def close(self, low_force_mode=False, wait=True):
        """Close the gripper.

        Keyword Arguments:
            low_force_mode {bool} -- use the low force mode (5N instead of 40N) (default: {False})
            wait {bool} -- wait until the end of the movement (default: {False})

        Raises:
            ROSException: timeout exception
        """
        self._move(1, low_force_mode, wait)

    def _move(self, target, low_force_mode=False, wait=True):
        self._set_digital_out(17, 1 if low_force_mode else 0)
        self._set_digital_out(16, target)
        self._ready = False
        if wait and not self._wait_for(lambda: self._ready and self._state == target):
            raise ROSException('Cannot move the gripper')

    def _io_states_cb(self, io_states):
        for digital in io_states.digital_in_states:
            if digital.pin == 16:
                self._state = int(digital.state)
            elif digital.pin == 17:
                self._ready = bool(digital.state)

    def _tool_data_cb(self, tool_data):
        self._tool_voltage = tool_data.tool_output_voltage
        self._position_voltage = tool_data.analog_input2

    def _set_tool_voltage(self, voltage):
        # self._set_io(fun=4, state=voltage) is not implemented in the driver
        cmd = '\n'.join([
            'sec MyProgram():',
            'set_tool_voltage({:d})'.format(voltage),
            'end'])
        self._script_command_pub.publish(data=cmd)

    def _set_digital_out(self, pin, state):
        self._set_io(fun=1, pin=pin, state=state)

    def _wait_for(self, condition_fn, timeout=5.0):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        while True:
            if condition_fn():
                return True
            if rospy.Time.now() > deadline:
                return False
            if rospy.core.is_shutdown():
                raise ROSInterruptException("rospy shutdown")
            rospy.rostime.wallsleep(0.01)
