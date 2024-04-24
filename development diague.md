## Description
The file is an introduction to how to prepare the software and hardware of the rover remote control system. 

## Table of Contents <a name="outline"></a>
1. [Table of Contents](#outline)
2. [Container](#container)
    - [Setup singularity](#set-up-singularity)
    - [Setup image](#setup-the-image)
3. [Prerequisite](#prerequisite)
4. [Install Ubuntu](#install-ubuntu)
5. [Install ROS](#install-ros)
6. [Install MAVLINK and MAVROS](#install-mavlink-and-mavros)
7. [Connecting to ErleBrain](#connecting-to-erlebrain)
8. [Host machine and local network](#host-machine-and-local-network)
9. [ROS configurations](#ros-configurations)
10. [Low-level Control and Communication](#low-level-control-and-communication)
      -  [Calibration](#calibration)
      -  [Send RC signal](#using-terminal-to-send-rc-signal)
      -  [Keyboard control](#keyboard-control)
      -  [Data Communication & Acquisition](#data-communication--acquisition)


## Container <a name="container"></a>
**Singularity** is a container platform. It allows you to create and run containers that package up pieces of software in a way that is portable and reproducible. 

### Set-up singularity <a name="singularity"></a>
The singularity provided here is `singularity-ce 3.10.2`, the files are available in the following GitHub repository:
[SingularityCE 3.10.2](https://github.com/sylabs/singularity/releases/tag/v3.10.2)

The instruction for singularity download and install is provided here:
[Installing SingularityCE](https://docs.sylabs.io/guides/latest/admin-guide/installation.html)

Depending on your OS, the downloading is provided as follows:

**Ubuntu 18.04 / Linux Mint 19 / Elementary OS 5** \
[singularity-ce_3.10.2-bionic_amd64.deb](https://github.com/sylabs/singularity/releases/download/v3.10.2/singularity-ce_3.10.2-bionic_amd64.deb)

**Ubuntu 20.04/ Linux Mint 20 / Elementary OS 6** \
[singularity-ce_3.10.2-focal_amd64.deb](https://github.com/sylabs/singularity/releases/download/v3.10.2/singularity-ce_3.10.2-focal_amd64.deb)

**Ubuntu 22.04 / Linux Mint 21 / Elementary OS 7** \
[singularity-ce_3.10.2-jammy_amd64.deb](https://github.com/sylabs/singularity/releases/download/v3.10.2/singularity-ce_3.10.2-jammy_amd64.deb)

**Install and Test** 
Install the `singularity` from source:
```bash
cd $HOME/Downloads
sudo dpkg -i ./singularity-ce_3.10.2-<your_distribution>_amd64.deb
sudo apt install -f
```

Open a new terminal and test if the installation is successful 
```bash
singularity --version
```

The output should be
```bash
singularity-ce version 3.10.2-XXXX
```

### Setup the image <a name="image"></a>

**Download the image** \
The image can be downloaded from the storage space in [Team - RC rover 2023](https://teams.microsoft.com/_?tenantId=096e524d-6929-4030-8cd3-8ab42de0887b#/school/FileBrowserTabApp/%E5%B8%B8%E8%A7%84?groupId=49107539-c634-4f20-9012-495b83cdc3aa&threadId=19:582ySj6s4WZVJOd7K0HRELjdM0qSSpRiVON46_yPR2o1@thread.tacv2&ctx=channel)

Save the image to local `/home/samgao1999/ubuntu_1604.sif` \
Convert the image to a writable sandbox directory:
```bash
sudo singularity build --sandbox ubuntu_1604 /home/samgao1999/ubuntu_1604.sif
```

**Option 1: Just open the container** \
To start the container, run the following command:

```bash
sudo singularity shell -w --hostname 192.168.0.100 kinetic.sif/ hostname
```
Substitute the argument `192.168.0.100` with your host machine IP address

**Option 2: Open the container and bind with Native OS** \
Only the `$home` directory (i.e. `/root`) is linked to the native OS by default, to be able to link to other files in the image change `~/catkin_ws/src/keyboard_control` to the location of the ROS code in your computer, run the following command:
```bash
sudo singularity shell -w --hostname 192.168.0.100 --bind ~/catkin_ws/src/keyboard_control:/app/catkin_ws/src/keyboard_control kinetic.sif/ hostname
```
Here we bind one folder `~/catkin_ws/src/keyboard_control` to the folder in the container `/app/catkin_ws/src/keyboard_control`

**Compile the Code**\
Go to the ROS workspace on the container:
```bash
cd /app/catkin_ws
catkin build
```

**Source the file** 
```bash
source /app/catkin_ws/devel/setup.bash
```


## Prerequisite <a name="prerequisite"></a>

Ubuntu 16.04 \
ROS kinetic \
MAVROS 0.18.3 \
MAVLINK 2016.10.10 

## Install Ubuntu <a name="ubuntu"></a>

1. Install Ubuntu 16.04: 
   * The ubuntu can be ran on virtual machine (eg. VMWARE)
   * you can follow the [install guide](https://linuxconcept.com/how-to-install-ubuntu-16-04-in-vmware-workstation/) to install the ubuntu. 
2. Verify Ubuntu Installation:
   * After installation, open the terminal with `ctrl` + `alt` + `t` 
   * Enter the following command to verify the Ubuntu version:
    ```bash
    lsb_release -a
    ```
    * The terminal should display the Ubuntu version.

The reason for choosing Ubuntu 16.04 is mainly due to compatibility with MAVLINK and MAVROS versions.

## Install ROS<a name="ros"></a>
1. Install ROS Kinetic:
   * The ROS can be installed based on the [Ubuntu install of ROS Kinetic](https://wiki.ros.org/kinetic/Installation/Ubuntu)
2. Verify ROS Installation:
   * After the installation, open the terminal and enter the following command:
    ```bash
    rosversion -d
    ```
   * The terminal should display `kinetic`, indicating a successful ROS installation.

## Install MAVLINK and MAVROS
The MAVROS and MAVLINK can be installed using `wget` webtool. The following installation guide are mainly based on the [PX4 USER GUIDE](https://docs.px4.io/main/en/ros/mavros_installation.html):

### Create a catkin workspace and initialize
1. Install the dependencies:
```bash
sudo apt-get install python-catkin-tools python-rosinstall-generator -y
```
2. The workspace in the guide is located in `~/catkin_ws`, change to your preferred folder. 
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
    version: release/kinetic/mavlink/2016.10.10-0

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

## Connecting to ErleBrain

There are several methods to connect to the ErleBrain: Hotspot, Ethernet cable, Wi-Fi, and using a monitor and keyboard. This guide will explain how to connect via cable and Wi-Fi.

### Using an Ethernet Cable

**Platform: Windows 10**

1. **Connect ErleBrain to Your Computer**:
   - Plug the Ethernet cable into the ErleBrain and your computer.
   - Observe the Ethernet Network port on the ErleBrain; it should be flashing. If it's not flashing, consider using the monitor and keyboard method (explained below).

2. **Share the Network Connection**:
   - Access the "Network Connections" on your Windows machine.
   - Right-click on the Ethernet connection and select "Properties."
   - Click on the "Sharing" tab.
   - Enable sharing and choose the Ethernet port that the ErleBrain is using.

3. **Check the IP Address of ErleBrain**:
   - Open the Windows terminal and enter the command: `arp -a`.
   - Find the IP address of the ErleBrain; typically, it's a dynamic address.

4. **Connecting via SSH**:
   - Use the following credentials to connect to the ErleBrain via SSH:
     ```bash
     Username: erle
     Password: holaerle
     ```
   - You will have access to a terminal to interact with the ErleBrain using SSH.

### Connecting the ErleBrain to Wi-Fi

1. **Create a WPA Passphrase**:
   - Generate a WPA passphrase for the Wi-Fi network you want to connect to. Replace `HostPC` with the network name and `holahost` with the corresponding password.
   ```bash
    network={
            ssid="HostPC"
            #psk="holahost"
            psk=c80d81ddbd0c2fb867164fdc93bceb1a76075d7c4ba635e8faa08f4d8f394312
    }
   ```
   - Save the content above to a file in `/etc/wpa_supplicant/`, naming it as desired but ending with `.conf`. You can edit the file using:

     ```bash
     sudo nano /etc/wpa_supplicant/your-chosen-name.conf
     ```
2. **Edit the `interfaces` File**:
   - Open the `interfaces` file for editing using the following command:

     ```bash
     sudo nano /etc/network/interfaces
     ```

   - Replace existing network configurations with the following lines:

     ```bash
     auto lo
     iface lo inet loopback

     auto wlan0
     allow-hotplug wlan0
     iface wlan0 inet manual
     wpa-conf /etc/wpa_supplicant/your-chosen-name.conf
     ```

   Make sure to comment out any other network configurations.

3. **Restart ErleBrain**:
   - After making these changes, restart the ErleBrain, and it will automatically connect to the Wi-Fi network.

**Note**: ErleBrain can connect to Ethernet and Wi-Fi simultaneously.

### Monitor&keyboard:

In some case when, 
 * the ethernet cable is not enabled on the erleBrain;
 * The network the laptop is using cannot be shared to the erleBrain;
 * There is no available Wifi network (eg. `eduroam`) you can use.

You need to connect the erleBrain with the monitor and keyboard with the HDMI cable and USB cable.

## Host machine and local network

### Host Machine
To configure your host machine for network connectivity in VMware, follow these steps, which are based on [VMware Docs](https://docs.vmware.com/en/VMware-Workstation-Pro/17/com.vmware.ws.using.doc/GUID-826323AD-D014-475D-8909-DFA73B5A3A57.html):
1. Select your virtual machine and go to VM > Settings.
2. On the Hardware tab, select Network Adapter.
3. Choose "Bridged: Connected directly to the physical network."
4. Select "Replicate physical network connection state."
5. Save the changes.
6. In your virtual machine, open the terminal and type the command `ifconfig`. You will see your IP address, typically in the form of `192.168.*.*`.

**Note**: Some networks, like `eduroam`, may not allow VMware to have a unique IP address. In such cases, you can establish a local network as described below.


### Local Network
To create a local network, follow these steps:

1. You need your personal modem to create a local network.
2. Power up the modem and connect both the laptop and the ErleBrain to the local network.

### Tests
To ensure that your network connection is working correctly, you can perform the following tests:

**On the Host Machine (Ubuntu 16.04 in the Virtual Machine)**:
Run the following command, replacing `192.168.0.101` with your ErleBrain's IP address:
```bash
ping 192.168.0.101
```

**On ErleBrain**: Run the following command, replacing `192.168.0.102` with your ErleBrain's IP address:
```bash
ping 192.168.0.102
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

## Low-level Control and Communication 
### Calibration 
* Follow the instruction from the `Apm Planner 2` to do the calibration;

### Using terminal to send RC signal 
* We can send to Remote Control through `rostopic`
    ```bash
    rostopic pub -r 10 /mavros/rc/override mavros_msgs/OverrideRCIn "channels: [1500, 0, 1500, 0, 0, 0, 0, 0]" 
    ```
* We can use the command to **arm** the rover to enable the accelaration of the rover:
    ```bash
    rosservice call /mavros/cmd/arming True
    ```
* `channel 1` is used to control the steering angle and `channel 3` is used to control the accelaration. 
* While using the `terminal` to control the rover, the **transmitter** will be disabled, we can run the following command to enable the **transmitter**:
  ```bash
  rostopic pub -1 /mavros/rc/override mavros_msgs/OverrideRCIn "channels: [0,0,0,0,0,0,0,0]"
  ```

### Keyboard Control
Run the keyboard control node
```bash
rosrun keyboard_control keyboard_control_node
```

Call ROS service to activate the motor of rover
```
rosservice call /mavros/cmd/arming True
```

Use `w,a,s,d` to send the control signal
  * `w` to speed up
  * `s` to speed down
  * `a` to steer left
  * `d` to steer right
  * `q` to reset the steering angle and velocity to 0
  * `ctrl C` to stop

### Data Communication & Acquisition
Launch the node to receive the localization data from MOCAP, record the localization data to file `data_output.csv`, use **rviz** to visualize the rover state and the coordination system.  
```bash
roslaunch rviz_car_model test.launch
```

## Observer
TBD

## High-level Controller
TBD
