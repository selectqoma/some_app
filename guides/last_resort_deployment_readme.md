# Last resort deployment

If something fails in the update steps or something doesn't work with docker 
you can create a service to run the cascade application natively in the server

It's assumed that the required repositories and dependencies are already installed
in the server and the code is already built.

Execute the following commands to install upstart services for the cascade and slagsquare applications:
```shell
$ cd /home/kpv/catkin_ws/src/smelter-monitoring/src/smelter_bringup/scripts
$ ./smelter_monitoring_decoupled_upstart_install.sh
```

Remove services with:
```shell
$ ./smelter_monitoring_decoupled_upstart_uninstall.sh
```
