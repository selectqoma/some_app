# Setup networking to establish internet access
1. Your laptop's IP must be set to the static IP 192.168.1.222.
1. Set the static IP 192.168.1.30 on the server using netplan. Should already be the case.
2. For giving internet access to the server set the wired connection on the dev laptop to *Shared to other computers*.
  a. Open network editor `$ nm-connection-editor`  
  b. Open your wifi hotspot  
  c. Go to IPv4 tab  
  d. Choose Method "Shared to other computers"  

You should end up with something like the following:
```
network:
  version: 2
  renderer: networkd
  ethernets:

    # network interface for camera network
    eno1:
      dhcp4: no      
      addresses: [ 192.168.1.30/24 ]
      gateway4: 192.168.1.222
      # gateway4: 192.168.1.1
      nameservers:
          addresses: [192.168.1.222,8.8.8.8,8.8.4.4]
          # addresses: [8.8.8.8,8.8.4.4]

    # network interface for administrative network (enable communication with osi pi server)
    eno2:
      dhcp4: no      
      addresses: [ 10.62.243.5/24 ]
      gateway4: 10.62.243.1
      nameservers:
          addresses: [10.32.3.86,10.32.3.87]
```
3. If you don't have internet connection after the above steps, there might be another network interface that interferes. For example, the network interface for administrative network must be disabled for allowing internet access.
```shell
$ sudo ifconfig eno2 down
```
