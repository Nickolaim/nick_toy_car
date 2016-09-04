#!/usr/bin/env python

import math


def joystick_axes_to_degree(axis0, axis1):
    """
    Convert joystick's axis 0 and axis 1 value into degrees.

    :param axis0: integer value for axis 0 from joystick
    :param axis1: integer value for axis 1 from joystick
    :return: For joystick in the center None is returned.
    Otherwise a degree is returned, where 0 degree points to North, 90 to East, -90 to West and 180/-180 to South
    """
    if axis0 == axis1 == 0:
        return None

    arc_tan_rad = math.atan2(-axis1, axis0)
    arc_tan_degree = math.degrees(arc_tan_rad) - 90

    arc_tan_degree = arc_tan_degree - 360 if arc_tan_degree >= 180 else arc_tan_degree
    arc_tan_degree = arc_tan_degree + 360 if arc_tan_degree <= -180 else arc_tan_degree

    return arc_tan_degree


def joystick_axes_to_command(axis0, axis1):
    """
    Convert joystick's axis 0 and axis 1 value into robot MOVE/STOP commands
    :param axis0: integer value for axis 0 from joystick
    :param axis1: integer value for axis 1 from joystick
    :return: robot command as a string
    """
    degree = joystick_axes_to_degree(axis0, axis1)
    return stop_command() if degree is None else move_command(degree)


def stop_command():
    """
    :return: the command for stop
    """
    return 'STOP'


def move_command(degree=None):
    """
    :param degree: degree, 0 pointing ahead.  If none, only string literal for the command is returned
    :return: the command for move
    """
    return 'MOVE' + ('' if degree is None else str(int(degree)))


def speed_command():
    """
    :return: the command for the speed
    """
    return 'SPEED'


def speed_increase_command():
    """
    :return: the command for increasing the speed
    """
    return speed_command() + '+'


def speed_decrease_command():
    """
    :return: the command for decreasing the speed
    """
    return speed_command() + '-'
