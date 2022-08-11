#!/usr/bin/env bash

. /usr/home/ws/devel/setup.bash
roscore > /dev/null &
exec "$@"
