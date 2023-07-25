# Deploy repositories using local git daemon
1. On the host side execute the following to start the git daemon. Take care to
replace the path to umicore repos with the actual path
```shell
$ git daemon --base-path=/path/to/umicore/repositories --export-all --reuseaddr --informative-errors --verbose
```
2. On the client side execute the following
If the repositories do not exist:
```shell
$ git clone git://192.168.1.222/smelter-monitoring
$ git clone git://192.168.1.222/smelter-datasets-and-models
$ ...
```
If the repositories already exist:
```shell
$ cd /path/to/smelter-monitoring \
  && git remote add local git://192.168.1.222/smelter-monitoring \
  && git pull local master
$ cd /path/to/smelter-datasets-and-models \
  && git remote add local git://192.168.1.222/smelter-datasets-and-models \
  && git pull local master
```

The rest of the repositories are already cloned inside the docker images, so
no need to clone them on the client side as well. You should clone them if you
want to run the application natively on the server instead of via docker.
