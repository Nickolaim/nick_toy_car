#!/usr/bin/env python

from __future__ import print_function
from std_msgs.msg import String
from subprocess import PIPE
import psutil
import rospy


def execute(command):
    popen = psutil.Popen(command, stderr=PIPE, universal_newlines=True)
    output_lines = iter(popen.stderr.readline, "")

    for output_line in output_lines:
        yield output_line.rstrip('\n')

    popen.stderr.close()
    return_code = popen.wait()
    if return_code != 0:
        raise psutil.CalledProcessError(return_code, command)


def main_loop():
    pub = rospy.Publisher('sixad', String, queue_size=10)
    rospy.init_node('sixad', anonymous=True)
    rate = rospy.Rate(10)

    for output in execute(["sixad", "--start"]):
        rospy.loginfo(output)
        pub.publish(output)
        rate.sleep()
        if rospy.is_shutdown():
            break


if __name__ == '__main__':
    try:
        main_loop()
    except rospy.ROSInterruptException:
        pass
