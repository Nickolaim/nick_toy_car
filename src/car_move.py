#!/usr/bin/env python

import atexit
from math import cos

import rospy
from Adafruit_MotorHAT import Adafruit_MotorHAT
from std_msgs.msg import String

from joy_to_command_translation import speed_increase_command, speed_decrease_command, command_to_motor_coef

maxValueForSpeed = 128  # 255 is the max speed, but it is 12V, too much for the current motor
mh = Adafruit_MotorHAT(addr=0x60)
speed = maxValueForSpeed / 10
speedStep = maxValueForSpeed / 16
speedUpdateTime = rospy.Time(0, 0)
speedUpdateInterval = rospy.Duration.from_sec(0.25)

# Recommended for auto-disabling motors on shutdown
def turnOffMotors():
    mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)


atexit.register(turnOffMotors)
leftMotor = mh.getMotor(3)
rightMotor = mh.getMotor(4)

def joystick_x_y_to_motor_coef(degree):
    c1 = cos(degree)
    if x >= 0:
        left = v
        right = 0. if x == 0 else 1.
    else:
        left = 1.
        right = v

    if y < 0:
        left = -left
        right = -right

    return left, right


def on_command(commandline):
    """
    :type commandline: String
    """
    commands = commandline.data.to_upper.split()
    left_joy_x = left_joy_y = 0.
    left_motor_coef = right_motor_coef = 0.
    for data in commands:
        if speed_increase_command() in data:
            speed_update(min(speed + speedStep, maxValueForSpeed))
        elif speed_decrease_command() in data:
            speed_update(max(speed - speedStep, speedStep))
        else:
            left_motor_coef, right_motor_coef = command_to_motor_coef(data)

    left_motor_speed = int(abs(left_motor_coef) * maxValueForSpeed)
    if left_motor_speed <= 1:
        leftMotor.run(Adafruit_MotorHAT.RELEASE)
    else:
        leftMotor.setSpeed(left_motor_speed)
        leftMotor.run(Adafruit_MotorHAT.FORWARD if left_motor_coef > 0 else Adafruit_MotorHAT.BACKWARD) 

    right_motor_speed = int(abs(right_motor_coef) * maxValueForSpeed)
    if right_motor_speed <= 1:
        rightMotor.run(Adafruit_MotorHAT.RELEASE)
    else:
        rightMotor.setSpeed(right_motor_speed)
        rightMotor.run(Adafruit_MotorHAT.FORWARD if right_motor_coef > 0 else Adafruit_MotorHAT.BACKWARD)

    print("Left: speed" + str(left_motor_speed) + ", coef:" + str(left_motor_coef) + "Right: speed" + str(right_motor_speed) + ", right:" + str(right_motor_coef) + "joystick: " + str(left_joy_x) + "," + str(left_joy_y))


def speed_update(speed_update_lambda):
    global speed, speedUpdateTime
    if rospy.get_rostime() - speedUpdateTime > speedUpdateInterval:
        speed = speed_update_lambda
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
