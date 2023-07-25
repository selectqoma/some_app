#!/bin/bash

# uninstall roscore upstart service
./roscore_upstart_uninstall.sh

# uninstall cascade upstart service
./cascade_monitoring_upstart_uninstall.sh

# uninstall slagsquare upstart service
./slagsquare_monitoring_upstart_uninstall.sh

# uninstall smelter-common upstart service
./smelter_common_upstart_uninstall.sh
