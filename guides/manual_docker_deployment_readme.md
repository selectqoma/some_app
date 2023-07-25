# How to deploy the docker container manually

## HOST SIDE (your laptop)
In the deployment pc you have to build the docker image first and save it inside the repository or pull it from the gitlab registry
```shell
$ cd /path/to/smelter-monitoring
$ docker-compose build
$ # or
$ docker pull registry.gitlab.com/kapernikov/umicore/smelter-monitoring/base_focal_x64
$ docker save --output=smelter-monitoring-docker-img.tar registry.gitlab.com/kapernikov/umicore/smelter-monitoring/base_focal_x64
```

## SERVER SIDE
1. Copy the new image to the server
```shell
$ scp -r smelter-monitoring-docker-img.tar kpv@192.168.1.30:~/catkin_ws/src/smelter-monitoring
```
2. Load the new image ON THE SERVER SIDE
**DO NOT confuse with docker import**
```shell
$ docker load -i smelter-monitoring-docker-img.tar
```
