# Backup existing configuration before update
Create a tarfile of the catkin workspace and copy old image to file
```shell
$ mkdir ~/backups
$ tar -czf ~/backups/catkin_ws-backup_$(date -I).tar ~/catkin_ws/
$ docker save --output=~/backups/smelter-monitoring-docker-img-backup_$(date -I).tar registry.gitlab.com/kapernikov/umicore/smelter-monitoring/base_focal_x64
```
