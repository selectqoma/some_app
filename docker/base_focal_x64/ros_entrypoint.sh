#!/bin/bash
set -e

# setup ros environment
source /opt/ros/$ROS_DISTRO/setup.bash
source /home/kpv/catkin_ws/devel/setup.bash

# disable generation of __pycache__
export PYTHONDONTWRITEBYTECODE=1

# start cron service
sudo service cron start

exec "$@"
