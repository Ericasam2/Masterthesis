## Description
The file is an introduction to how to prepare the software and hardware of the rover remote control system. 

## Prerequisite
Ubuntu 16.04 \
ROS kinetic \
MAVROS 0.18.3 \
MAVLINK 2016.10.10 

## Install Ubuntu
The ubuntu can be ran on virtual machine (eg. VMWARE), we can follow the [install guide](https://linuxconcept.com/how-to-install-ubuntu-16-04-in-vmware-workstation/) to install the ubuntu. After installation, we can use `ctrl` + `alt` + `t` to open the terminal and enter:
```bash
lsb_release -a
```
The version of the ubuntu should be shown in the terminal. \
The reason we choose Ubuntu 16.04: Mainly because the compatiality of the MAVLINK and MAVROS version.

## Install ROS
The ROS can be installed based on the [Ubuntu install of ROS Kinetic](https://wiki.ros.org/kinetic/Installation/Ubuntu). After the installation, we can open the terminal and enter:
```bash
rosversion -d
```
The following should be shown on the terminal:
```bash
kinetic
```

## Install MAVLINK and MAVROS
The MAVROS and MAVLINK can be installed using `wget` webtool. The following installation guide are mainly based on the [PX4 USER GUIDE](https://docs.px4.io/main/en/ros/mavros_installation.html):

### Create a catkin workspace and initialize
1. Install the dependencies:
```bash
sudo apt-get install python-catkin-tools python-rosinstall-generator -y
```
2. The workspace in the guide is located in `~/catkin_ws`, change to your prefered folder. 
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
wstool init src
```
3. Retrive the version of MAVLink and MAVROS:
```bash
rosinstall_generator --rosdistro kinetic mavlink | tee /tmp/mavros.rosinstall
rosinstall_generator --upstream mavros | tee -a /tmp/mavros.rosinstall
```
4. Edit the file `/tmp/mavros.rosinstall`
```bash
- git:
    local-name: mavlink
    uri: https://github.com/mavlink/mavlink-gbp-release.git
    version: 2016.10.10

- git:
    local-name: mavros
    uri: https://github.com/mavlink/mavros.git
    version: 0.18.3
```
5. Create workspace & deps
```bash
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src -j4
rosdep install --from-paths src --ignore-src -y
```
6. Install [GeographicLib](https://geographiclib.sourceforge.io/) datasets:
```bash
./src/mavros/mavros/scripts/install_geographiclib_datasets.sh
```

7. Build source
```bash
catkin build
```

8. Make sure that you use setup.bash or setup.zsh from workspace
```
source devel/setup.bash
```

## Connect to ErleBrain
There are several ways to connect to the erlebrain: Hotspot, ethernet cable, wifi, monitor&keyboard, etc ... \
Here we will introduce how to use the cable and wifi to connect to the erleBrain. 
### Ethernet Cable:
* the Platform is windows 10.
1. connect the erlebrain to the computer with the ethernet cable. 
    * you should observe the ethernet Network port on erleBrain is flashing. 
    * If it is not flashing, you need use the monitor&keyboard. 
2. Share the network connection:
    * Access the “Network Connections”;
    * Right click on the Ethernet connection and select “Properties”;
    * Click on "Sharing Tab";
    * Enable sharing and choose the ethernet port the erleBrain is using.   
3. Check the ipadress of the erlebrain:
   * Open terminal in Windows and type `arp -a`
   * Find the ipaddress of the erlebrain 
   * the ipaddress of erlebrain normally should be a dynamic address. 
4. Using ssh to connect to the erlebrain
   ```bash
   user.name = "erle"
   user.password = "holaerle"
   ```
   We will have a terminal where we can interface with the erleBrain using ssh. 

### Connect the erleBrain to Wifi:
1. Create a wpa passphase based on the Wifi network you want to connect `wpa_passphrase HostPC holahost`, change the `HostPC` to the name of your Wifi, and change the `holahost` to the correspending password. 
   ```bash
    network={
            ssid="HostPC"
            #psk="holahost"
            psk=c80d81ddbd0c2fb867164fdc93bceb1a76075d7c4ba635e8faa08f4d8f394312
    }
   ```
   Then save the above generated-content to `/etc/wpa_supplicant/erle.conf`, change the name `erle.conf` to the the name you want, the file need to end with `.conf`
   Edit the file using:
   ```bash
   sudo nano /etc/wpa_supplicant/erle.conf
   ```
2. Edit the `interface` file using:
    ```bash
    sudo nano /etc/network/interfaces
    ```

    ```bash
    # Please note that this file is written to be used with dhcpcd.
    # For static IP, consult /etc/dhcpcd.conf and 'man dhcpcd.conf'.

    auto lo
    iface lo inet loopback

    auto wlan0
    allow-hotplug wlan0
    iface wlan0 inet manual
    wpa-conf /etc/wpa_supplicant/erle.conf
    ```
    Make sure to comment all the other network configuration options. 
3. Restart the erleBrain and the Wifi will be automatically connected. 
   
**Note that the erleBrain can connect to the ethernet and the Wifi at the same time.**

### Monitor&keyboard:

In some case when, 
 * the ethernet cable is not enabled on the erleBrain;
 * The network the laptop is using cannot be shared to the erleBrain;
 * There is no available Wifi network (eg. `eduroam`) you can use.

You need to connect the erleBrain with the monitor and keyboard with the HDMI cable and USB cable.

## Host machine and local network

### Host Machine
In VMWare, the host machine can be configured and connected to the network in the following way, the guide is based on [VMware Docs](https://docs.vmware.com/en/VMware-Workstation-Pro/17/com.vmware.ws.using.doc/GUID-826323AD-D014-475D-8909-DFA73B5A3A57.html):
1. Select virtual machine and select VM>Setting.
2. On the Hardware tab, select Network Adapter;
3. Select *Bridged: Connected directly to the physical network*
4. Select *Replicate physical network connection state*
5. Save the changes.
6. In your Virtual machine, type the command in the terminal
   `ifconfig`, you will see your IP address (normally in the form of `192.168.*.*`)

Note that some network do not allow the VMware to have a unique IP address (eg. `eduroam`). If this case happen, we can establish another local network instead. 


### Local Network
1. You need your personal modem to create a local network; 
2. Power up the modem and do the above process to connect the laptop and the erleBrain to the local network. 

### Tests
* On host machine (ubuntu 16.04 in Virual machine), run the following command to test the connection, change the `192.168.0.102` to your erleBrain's IP address.
```bash
ping 192.168.0.102
```
* On ErleBrain, run the following command to test the connection, change the `192.168.0.101` to your hostmachine's IP address.
```bash
ping 192.168.0.101
```
If the `ping` is successful both ways, the connection is established, and we can do the further MAVLink test. 

### Some further reading
* The reason we want to create a local network and connect all our devices `HostMachine` and `ErleBrain` to it is that, we want to make sure all devices can communicate with each other. 
* We try to put the device under the same network segmentation, we can refer to [what-is-LAN](https://www.cisco.com/c/en/us/products/switches/what-is-a-lan-local-area-network.html#~what-it-is) for some basic understanding. 

## ROS configurations
### Host machine
The IP address of the host machine need to be specified when we start the roscore:
```bash
sudo nano ~/.bashrc
```
And append the following context to the `.bashrc` file, change the `192.168.0.101` to your own HostMachine IP address:
```bash
#ROS
export ROS_MASTER_URI=http://192.168.0.101:11311/ # hostmachine IP
export ROS_IP=192.168.0.101 # hostmachine IP
export ROS_HOSTNAME=192.168.0.101
```
Save the file and exit, remember to source the `.bashrc` file:
```bash
source ~/.bashrc
```
Remember to source the ROS configuration file:
```bash
source /opt/ros/kinetic/setup.bash
```
### ErleBrain
```bash
sudo nano ~/.bashrc
```
And append the following context to the `.bashrc` file, change the `192.168.0.102` to your own ErleBrain IP address:
```bash
#ROS
export ROS_MASTER_URI=http://192.168.0.101:11311/ # hostmachine IP
export ROS_IP=192.168.0.102 # erlebrain IP
export ROS_HOSTNAME=192.168.0.102
```
Save the file and exit, remember to source the `.bashrc` file:
```bash
source ~/.bashrc
```
Change the settings of MAVROS using 
```bash
sudo nano /opt/ros/kinetic/setup-mavros.bash
```
change the content to 
```bash
#!/usr/bin/env bash
# generated from catkin/cmake/templates/setup.bash.in

CATKIN_SHELL=bash

# source setup.sh from same directory as this file
_CATKIN_SETUP_DIR=$(builtin cd "`dirname "${BASH_SOURCE[0]}"`" > /dev/null && pwd)
. "$_CATKIN_SETUP_DIR/setup.sh"

# wait a bit for roscore to stabilize
sleep 3

# Launch mavros with the udp bridge
# rosrun mavros mavros_node _fcu_url:="udp://:6001@" _gcs_url:="udp://10.0.0.1:9000@10.0.0.2:6000?ids=1,255,252"

roslaunch mavros apm.launch fcu_url:="udp://:6001@"

```
Remember to source the ROS configuration file:
```bash
source /opt/ros/kinetic/setup.bash
```

### Test
1. On hostMachine, open terminal and start ROS 
   ```bash
   roscore
   ```
2. On erleBrain, launch mavros using
   ```bash
   /opt/ros/kinetic/setup-mavros.bash
   ```
3. On erleBrain, open a new terminal and enable the apm setting
   ```bash
   sudo ~/apm.sh
   ```
4. To make sure if the devices are connected, you can run the command to test:
    ```bash
    rostopic echo -n1 /diagnostics
    ```
    If everything goes well the output would be the following:
    ```
    header: 
    seq: 17
    stamp: 
        secs: 1573737275
        nsecs: 972295800
    frame_id: ''
    status: 
    - 
        level: 0
        name: "leader_mavros: FCU connection"
        message: "connected"
        hardware_id: "udp://:6001@"
    ```

## Some hardware issue
![The seriel ports of ErleBrain](media/Erle-Brain2_dimensiones.jpg)
* Many funtions of the erleBrain are hardware-based, which means the seriel leads need to be connected to other hardware to enble the correspending functions.
* To enable the `Radio Control`, 
  * the `PPM-SUM input channel` need to be connected to `RC receiver`;
  * the `PPM-SUM input channel 1` need to be connected to front motor and the `PPM-SUM input channel 3` need to be connected to back motor;
  * the `RC transimitter` need to be powered and turned on;
  * When the `RC receiver` is flasing, the switch on the `receiver` need to be pressed and turned on.
* TBD
  
## Apm planner 2
The APM planner 2 can be used to interface with the ErleBrain with graphical interfaces. The software helps you with the calibration, monitoring and control of the erleBrain and rover. 

### Installation: 
We can follow the [Installing APM Planner for Linux](https://ardupilot.org/planner2/docs/installation-for-linux.html) to install the apm planner. 

### ErleBrain:
We need to edit the `opt/ros/kinetic/setup-mavros.bash`
```bash
sudo nano opt/ros/kinetic/setup-mavros.bash
```
We can append the following command to the file and comment other command start with `roslaunch`, here replace the `192.168.0.102` with your ErleBrain IP and `192.168.0.101` with your HostMachine IP:
```bash
roslaunch mavros apm.launch fcu_url:="udp://:6001@" gcs_url:="udp://192.168.0.102:9000@192.168.0.101:6000?ids=255,252,1"
# ErleBrain IP: 192.168.0.102
# HostMachine IP: 192.168.0.101
```
