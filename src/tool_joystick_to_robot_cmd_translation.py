#!/usr/bin/env python

import pipes
import sys

from joy_to_command_translation import joystick_axes_to_command, speed_increase_command, speed_decrease_command

if len(sys.argv) != 2:
    arg0 = sys.argv[0]
    sys.exit('Converts output of jstest into commands for the robot.\n'
        'Usage: %s ''command_produces_jstest_output''\n'
        'Example: ./test_joystick_to_command_translation.py ''jstest /dev/input/js0''\n'
        % arg0)

pipe_cmd = sys.argv[1]
print 'Command:' + pipe_cmd

marker_length = len(' X:')

t = pipes.Template()
t.append(pipe_cmd, '--')
f = t.open('', 'r')

while True:
    line = f.read(1024)
    if not line:
        print 'Finished reading'
        break

    pos2 = 0
    while True:
        posAxis = line.find('Axes:', pos2)
        pos0 = line.find(' 0:', posAxis)
        pos1 = line.find(' 1:', pos0)
        pos2 = line.find(' 2:', pos1)
        if pos0 == -1 or pos1 == -1 or pos2 == -1:
            break
        v1 = int(line[pos0 + marker_length:pos1])
        v2 = int(line[pos1 + marker_length:pos2])
        robot_cmd = joystick_axes_to_command(v1, v2)

        if line.find('8:on') > -1:
            robot_cmd += ' ' + speed_decrease_command()

        if line.find('9:on') > -1:
            robot_cmd += ' ' + speed_increase_command()

        print "Axis0: {:>7}, Axis1: {:>7}, Command:{:<12}".format(v1, v2, robot_cmd)
f.close()

