#!/usr/bin/python

from Adafruit_MotorHAT import Adafruit_MotorHAT
import atexit
import rospy
from std_msgs.msg import String

maxValueForSpeed = 64  # 255 is the max speed, but it is 12V, too much for the current motor
mh = Adafruit_MotorHAT(addr=0x60)
speed = maxValueForSpeed / 10
speedStep = maxValueForSpeed / 64
speedUpdateTime = rospy.Time(0, 0)
speedUpdateInterval = rospy.Duration.from_sec(0.25)

# Recommended for auto-disabling motors on shutdown
def turnOffMotors():
    mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)


atexit.register(turnOffMotors)
activeMotors = [mh.getMotor(3), mh.getMotor(4)]
map(lambda x: x.setSpeed(speed), activeMotors)


def on_command(command):
    """
    :type command: String
    """
    data = command.data
    if "+" in data:
        speed_update(min(speed + speedStep, maxValueForSpeed))
    elif "-" in data:
        speed_update(max(speed - speedStep, speedStep))
    elif "F" in data:
        map(lambda x: x.run(Adafruit_MotorHAT.FORWARD), activeMotors)
    elif "B" in data:
        map(lambda x: x.run(Adafruit_MotorHAT.BACKWARD), activeMotors)
    elif "S" in data:
        map(lambda x: x.run(Adafruit_MotorHAT.RELEASE), activeMotors)


def speed_update(speed_update_lambda):
    global speed, speedUpdateTime
    if rospy.get_rostime() - speedUpdateTime > speedUpdateInterval:
        speed = speed_update_lambda
        map(lambda x: x.setSpeed(speed), activeMotors)
        speedUpdateTime = rospy.get_rostime()


def run():
    global speedUpdateTime
    rospy.init_node('car_move_node', anonymous=True)
    rospy.Subscriber('car_command', String, on_command)
    speedUpdateTime = rospy.get_rostime()
    rospy.spin()


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
