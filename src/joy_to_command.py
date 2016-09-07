#!/usr/bin/env python

""" This node translates joystick input into commands for the car.
Car commands are case insensitive strings.  Multiple command can appear in one string.  Space separated.
"""

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String

from joy_to_command_translation import speed_increase_command, speed_decrease_command, joystick_axes_to_command

pub = rospy.Publisher('car_command', String, queue_size=10)

def button_index_to_command(argument):
    switcher = {
        8: speed_decrease_command(),
        9: speed_increase_command(),
    }
    return switcher.get(argument, "")


def callback(joy_message):
    """
     :type joy_message: message from joystick
    """
    command = ""
    for i in range(len(joy_message.buttons)):
        if joy_message.buttons[i] != 0:
            cmd = button_index_to_command(i)
            if cmd != "":
                command += cmd + " "

    command += joystick_axes_to_command(joy_message.axes[0], joy_message.axes[1])

    if command != "":
        pub.publish(command)


def run():
    rospy.init_node('joy_to_command_node', anonymous=True)
    rospy.Subscriber('joy', Joy, callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
