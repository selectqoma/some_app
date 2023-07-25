# Deploy the docker image of the application using a local docker registry

## HOST (Your Laptop):
1. Add your ip to /etc/openssl.conf
```shell
$ MYIP=192.168.1.222
$ sudo sed "/\[\ v3_ca\ \]/a subjectAltName=IP:$MYIP" /etc/ssl/openssl.cnf -i.bak
```
2. Generate a Certificate Authority (CA)
```shell
$ mkdir ~/certs
$ cd ~ && openssl req \
    -newkey rsa:4096 -nodes -sha256 -keyout certs/domain.key \
    -x509 -out certs/domain.crt
```
3. Add domain.crt to your docker certificates
```shell
$ mkdir -p /etc/docker/certs.d/$MYIP:5000
$ cp ~/certs/domain.crt /etc/docker/certs.d/$MYIP:5000/ca.crt
```
4. Pull the local docker registry image
```shell
$ docker run -d -p 5000:5000 --restart=always registry.gitlab.com/kapernikov/umicore/smelter-monitoring/local-docker-registry/amd64
```

## CLIENT (Umicore Server):
1. Copy the Certificate Authority (CA) to the client
```shell
$ MYIP=192.168.1.222 # yes this is correct
$ mkdir -p /etc/docker/certs.d/$MYIP:5000
$ scp yourusername@yourip:~/certs/domain.crt /etc/docker/certs.d/$MYIP:5000/ca.crt
$ echo { "insecure-registries":["192.168.1.222:5000"] } > /etc/docker/daemon.json
```
2. Pull the docker image of the application
```shell
$ docker pull 192.168.1.222:5000/smelter-monitoring/base_focal_x64
$ docker tag 192.168.1.222:5000/smelter-monitoring/base_focal_x64 registry.gitlab.com/kapernikov/umicore/smelter-monitoring/base_focal_x64
```
