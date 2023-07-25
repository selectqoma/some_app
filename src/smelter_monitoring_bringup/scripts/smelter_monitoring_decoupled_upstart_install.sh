#!/bin/bash

# install roscore upstart service
./roscore_upstart_install.sh

# install cascade upstart service
./cascade_monitoring_upstart_install.sh

# install slagsquare upstart service
./slagsquare_monitoring_upstart_install.sh

# install smelter-common upstart service
./smelter_common_upstart_install.sh
