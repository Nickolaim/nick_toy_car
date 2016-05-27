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

def button_index_to_command(argument):
    switcher = {
        3: "S",
        4: "F",
        6: "B",
    }
    return switcher.get(argument, "")


def callback(joy_message):
    """
 
    :type joy_message: Joy
    """
    command = ""
    for i in range(len(joy_message.buttons)):
        if joy_message.buttons[i] != 0:
            cmd = button_index_to_command(i)
            if cmd != "":
                command += cmd + " "

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
