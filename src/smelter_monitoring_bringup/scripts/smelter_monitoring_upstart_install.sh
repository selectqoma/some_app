#!/bin/bash

# remove previous version of smelter-monitoring service if it exists
rosrun robot_upstart uninstall smelter-monitoring

# create new smelter-monitoring service
rosrun robot_upstart install \
  --job smelter-monitoring \
  --user kpv \
  --setup /home/kpv/catkin_ws/src/smelter-monitoring/src/smelter_monitoring_bringup/scripts/setup.sh \
  --logdir /home/kpv/.ros/ \
  smelter_monitoring_bringup/launch/smelter_monitoring_bringup.launch

# start new smelter-monitoring service
sudo systemctl daemon-reload && sudo systemctl start smelter-monitoring
