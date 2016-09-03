#!/usr/bin/env python

import pipes
import sys

if len(sys.argv) != 2:
    arg0 = sys.argv[0]
    sys.exit('Converts output of jstest into commands for the robot.\n'
        'Usage: %s ''command_produces_jstest_output''\n'
        % arg0)

cmd = sys.argv[1]
print 'Command:' + cmd

marker_length = len(' X:') + 1

t = pipes.Template()
t.append(cmd, '--')
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
        print v1, v2
f.close()

