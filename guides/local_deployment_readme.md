# Local Deployment
The following instructions guide you on how to deploy the application on your system for development purposes

1. Clone the repository
```shell
$ git clone git@gitlab.com:Kapernikov/Umicore/smelter-monitoring.git
```
2. Clone the datasets and models repository
```shell
$ git clone git@gitlab.com:Kapernikov/Umicore/smelter-datasets-and-models.git
```
3. Pull or build the docker image
```shell
$ docker pull registry.gitlab.com/kapernikov/umicore/smelter-monitoring/base_focal_x64:latest
```
or 
```shell
$ cd smelter-monitoring
$ docker-compose build
```
4. Run a docker container
```shell
$ docker-compose up -d
```
5. Connect to the docker container
```shell
$ ./docker_connect.sh
```
