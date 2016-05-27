#!/usr/bin/python

from Adafruit_MotorHAT import Adafruit_MotorHAT
import atexit
import rospy
from std_msgs.msg import String

maxValueForSpeed = 64   # 255 is the max speed, but it is 12V, too much for the current motor
mh = Adafruit_MotorHAT(addr=0x60)


# Recommended for auto-disabling motors on shutdown!
def turnOffMotors():
    mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)


atexit.register(turnOffMotors)
mainMotor = mh.getMotor(3)
mainMotor.setSpeed(maxValueForSpeed / 2)


def on_command(command):
    """
    :type command: String
    """
    data = command.data
    if "F" in data:
        mainMotor.run(Adafruit_MotorHAT.FORWARD);
    elif "B" in data:
        mainMotor.run(Adafruit_MotorHAT.BACKWARD);
    elif "S" in data:
        mainMotor.run(Adafruit_MotorHAT.RELEASE);


def run():
    rospy.init_node('car_move_node', anonymous=True)
    rospy.Subscriber('car_command', String, on_command)
    rospy.spin()


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
