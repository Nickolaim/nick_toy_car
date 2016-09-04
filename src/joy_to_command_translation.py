#!/usr/bin/env python

import math


def joystick_axes_to_degree(axis0, axis1):
    """
    joystick_axes_to_degree(axis0, axis1)

    Convert joystick's axis 0 and axis 1 value into degrees.
    For joystick in the center None is returned.
    Otherwise a degree is returned, where 0 degree points to North, 90 to East, -90 to West and 180/-180 to South
    """
    if axis0 == axis1 == 0:
        return None

    arc_tan_rad = math.atan2(-axis1, axis0)
    arc_tan_degree = math.degrees(arc_tan_rad) - 90

    arc_tan_degree = arc_tan_degree - 360 if arc_tan_degree >= 180 else arc_tan_degree
    arc_tan_degree = arc_tan_degree + 360 if arc_tan_degree <= -180 else arc_tan_degree

    return arc_tan_degree
