# Deploy the smelter-monitoring app docker image using Ansible (For initial deployment after format)

1. Install openssh-server in the remote server
```shell
$ sudo apt install openssh-server
```
2. Copy your ssh key to the authorized\_keys of the machine, otherwise, connection is refused when running the playbook.
```shell
$ ssh-copy-id kpv@192.168.1.30
```
3. You must create a data partition with label **log** and auto-mount it to ~/.ros
- After you have created the partition open /etc/fstab and add the following
```
UUID=<UUID of partition> /home/kpv/.ros ext4 defaults,users
```
- Check the permissions of the partition and if it's root execute the following:
```shell
$ sudo chown kpv:kpv -R /home/kpv/.ros
```
4. In the deployment pc you have to build the docker image first and save it inside the repository or pull it from the gitlab registry
```shell
$ cd /path/to/smelter-monitoring
$ docker-compose build
$ # or
$ docker pull registry.gitlab.com/kapernikov/umicore/smelter-monitoring/base_focal_x64
$ docker save --output=smelter-monitoring-docker-img.tar registry.gitlab.com/kapernikov/umicore/smelter-monitoring/base_focal_x64
```
5. Execute the ansible script (requires [ansible](https://docs.ansible.com/ansible/latest/installation_guide/intro_installation.html#latest-releases-via-apt-ubuntu)>=2.8.0)  
Prerequisites:
- Execute the script from inside the repository
- The repository must be located in the same directory as the smelter-datasets-and-models repository.
- Update the ip of the remote server in /path/to/smelter-monitoring/hosts file if it's different from 192.168.1.30.
```shell
$ ansible-playbook smelter-monitoring-ansible-playbook.yml -v -i hosts -K --extra-vars "host=kpv_umicore_smelter_server"
```
