#!/bin/bash

# remove previous version of smelter-monitoring service if it exists
rosrun robot_upstart uninstall cascade-monitoring

# create new smelter-monitoring service
rosrun robot_upstart install \
  --job cascade-monitoring \
  --user kpv \
  --setup /home/kpv/catkin_ws/src/smelter-monitoring/src/smelter_monitoring_bringup/scripts/setup.sh \
  --logdir /home/kpv/.ros/ \
  --wait \
  smelter_monitoring_bringup/launch/cascade_monitoring_bringup.launch

# start new smelter-monitoring service
sudo systemctl daemon-reload && sudo systemctl start cascade-monitoring

