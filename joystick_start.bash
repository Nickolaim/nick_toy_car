#!/usr/bin/env bash

sixad_running_check () {
    ps -e | grep sixad-bin > /dev/null
}

if [ ! -a /dev/input/js0 ]; then
    if [ "$(whoami)" != 'root' ]; then
        echo "The script $0 must be run as root."
        exit 1;
    fi

    if (!(sixad_running_check))
        then
        /usr/sbin/sixad-bin 0 0 0 &>> /tmp/nick_toy_car_joystick_start.log &
    fi

    if (!(sixad_running_check))
        then
        echo "sixad is not started."
        exit 1
    fi
fi
