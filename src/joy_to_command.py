#!/usr/bin/env python

""" This node translates joystick input into commands for the car.
Car commands are just stings, case insensitive.  Multiple command can appear in one string.  Space separated.
List of commands:
F: move forward
B: move backward
"""

from std_msgs.msg import String
from sensor_msgs.msg import Joy
import rospy

pub = rospy.Publisher('car_command', String, queue_size=10)


def callback(joy_message):
    """
 
    :type joy_message: Joy
    """
    command = ""
    if joy_message.buttons[4] != 0:
        command += "f "
    if joy_message.buttons[6] != 0:
        command += "b "

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
