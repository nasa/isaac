\page wifi_driver Wifi Driver

This package provides the driver for publishing wifi strength info. I is based on the library available in https://github.com/bmegli/wifi-scan, adapted to ROS for this project.

Because the function that scans all the networks needs network admin rights, there are 2 modes in which this driver can run. The modes can be set as:

    roslaunch astrobee astrobee.launch wifi:=node

There are three options: 
	none: no wifi reader is active (default)
	station: has the 'scan station' activated;
	all: has the 'scan all' activated

For the 'all' option, one needs to give the rights to the node executable, as:

    setcap cap_net_admin+ep /path/to/wifi/node
(as a default, the node executable is in the build folder in devel/lib/wifi/wifi_tool)

All the parameters are in astrobee/config/hw/wifi.config
General Parameters:

    interface_name: to find ou the interface name, one can find out the interface name by running ifconfig in the cmd line.

### Scan Station

Publishes information to the topic hw/wifi/station. It only includes signal strength of the wifi network to which the robot is currently connected to.
Scan Station Parameters:

    time_scan_station: update rate of the station scan, can be set upt to 50ms.

### Scan All

Publishes information to the topic hw/wifi/all.
Scan All Parameters:

    time_scan_station: update rate of the station scan, can be set up to 50ms.
    time_scan_all: time in between scans, note that even if the time is set to a low limit, information acquisition rate is limited.
    max_networks: maximum number of networks that will be acquired during 'all' scan.

All data is published as messages on ROS topics using the prefix hw/wifi.


### Lib mnl
This package is needed in hardware/wifi node, such that we can scan the wifi networks.
If you are missing this package, and have sudo rights, do:

    sudo apt-get install libmnl-dev

Otherwise, it can be installed from source as follows:

    mkdir $HOME/projects
    cd $HOME/projects
    wget https://www.netfilter.org/projects/libmnl/files/libmnl-1.0.4.tar.bz2
    tar xjfv libmnl-1.0.4.tar.bz2
    cd libmnl-1.0.4
    ./configure --prefix=$(pwd)/install
    make -j 10
    make install
    export PKG_CONFIG_PATH=$HOME/projects/libmnl-1.0.4/install/lib/pkgconfig

The last line above will tell catkin where to look for this library.

