# Dialogue 18/9/2023

## 1. 18 channels → 8 channels

I change the code in .msg file and rebuild the MAVROS, then I use the following test command:

```
rostopic pub -r 10 /maos/rc/override mavros_msgs/OverrideRCIn "channels: [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]"
```

the ErleBrain give me the following feedback: *[ WARN] [1693328496.698021710]: RC override not supported by this FCU!*

![Send RC signal and get warning RC override not supported by this FCU!](/home/samgao1999/桌面/thesis/2023-09-18%2016-15-05%20的屏幕截图.png)

I might consider reinstall the older version of MAVROS

In *mavros/mavros_msgs/SHANGELOG.rst*, there is an update suggests that Mavlink v2.0 accept RC_CHANNELS_OVERRIDE up to 18 channels. If the FCU_protocol version is 2.0,


# Dialogue 20/9/2023
## 1. Simulation launch
[The launch procedure](https://docs.px4.io/main/en/simulation/ros_interface.html) shows that the simulation with MAVROS can be initialized with with the launch file:
```
cd <PX4-Autopilot_clone>
roslaunch px4 mavros_posix_sitl.launch
```
Then the [takeoff procedure](https://masoudir.github.io/mavros_tutorial/Chapter1_ArduRover_with_CLI/Step2_How_to_Arm_and_Disarm/#arm-and-disarm)

Terminal 1:
```
rosservice call /mavros/set_mode "custom_mode: 'OFFBOARD"
rostopic pub /mavros/setpoint_position/local geometry_msgs/PoseStamped -r 20 "pose:
  position:
    x: 0.0
    y: 0.0
    z: 10.0"
```
Terminal 2:
```
rosservice call /mavros/cmd/arming True
```  

## 2. The version collision
I try to subscribe the topic "/mavros/states", and the system give me the following warning:

```
[ERROR] [1693341196.359086204]: Client [/rostopic_20707_1695290724770] wants topic /mavros/state to have datatype/md5sum [mavros_msgs/State/ce783f756cab1193cb71ba9e90fece50], but our version has [mavros_msgs/State/9e3d873fae342c8f48a8bd64c53d991e]. Dropping connection.
```

Following the [Solution from Github](https://github.com/mavlink/mavros/issues/1517), it is important to downgrade the version of mavros/ mavlink/ mavros_msgs on my laptop. By checking the version on the erlebrain, I Find that:
```
erle@erle-brain:~ $ rosversilogErroron mavros
0.18.3
erle@erle-brain:~ $ rosversion mavlink
2016.9.9
erle@erle-brain:~ $ rosversion mavros_msgs
0.18.3
```

The definition on **\<erlebrain\>**/mavros_msgs/msg/State.msg:
```
# Current autopilot state
#
# Known modes listed here:
# http://wiki.ros.org/mavros/CustomModes

std_msgs/Header header
bool connected
bool armed
bool guided
string mode
```

The definition on **\<laptop\>**/mavros_msgs/msg/State.msg:
```
# Current autopilot state
#
# Known modes listed here:
# http://wiki.ros.org/mavros/CustomModes
#
# For system_status values
# see https://mavlink.io/en/messages/common.html#MAV_STATE
#

std_msgs/Header header
bool connected
bool armed
bool guided
bool manual_input
string mode
uint8 system_status
```

I compare the code of MAVROS 0.19.0 and MAVROS 0.26.0 (the earliest version of mavros after melodic mavlink released), the main problem is that the log function is redefined in "#include \<mavconn/console_bridge_compat.h\>", if the code is regenerated, the current phase "catkin-build" problem may get fixed. 


# Dialogue 8/10/2023
## Ubuntu 16.04
I used the virtual machine to install the ubuntu 16.04 and install the history version of mavlink and mavros.
```bash
samgao1999@ubuntu:~$ rosversion mavlink
2016.10.10
samgao1999@ubuntu:~$ rosversion mavros
0.18.3
```
## Network and connection
And I changed some internet setup in both */etc/network/interfaces* and */etc/wpa_supplicant/home.conf*:
```bash
erle@erle-brain:~ $ cat /etc/network/interfaces
# Please note that this file is written to be used with dhcpcd.
# For static IP, consult /etc/dhcpcd.conf and 'man dhcpcd.conf'.

auto lo
iface lo inet loopback

auto wlan0
allow-hotplug wlan0
iface wlan0 inet manual
wpa-conf /etc/wpa_supplicant/home.conf

```

```bash
erle@erle-brain:~ $ cat /etc/wpa_supplicant/home.conf 
network={
	ssid="TP-LINK_ECE5"
	#psk="setapassword"
	psk=5b3ee93e6881321a6c5f00db4fe820c0993bd021e29e0ee8449de4a4f7ccd804
}
```

One problem I want to keep note of is that the erleBrain3 only has **2.4Gz** wifi network connection, the prevailing 5Gz signal cannot be used by it. Also, under school network **eduroam**, the network cannot be shared through *bridge mode* in VMware. The ideal way is to use a private network. 
## PreArm issue
In previous setup, I find there is version collision so that I cannot check the */mavros/state*, now I fix the problem and I can check the state of the erleBrain, now new problem arise, even if I publish command to topic */mavros/rc/override*, the */mavros/rc/in* still do not give me the ideal signal. Now I think it may related to the *arming* issue, the controller may needed to be armed to take off or receive signal, when I try to arm the controller, it gives me the error:
```bash
[ERROR] [1696607025.001099516]: FCU: PreArm: 3D Accel calibration needed
```
In the following experiment, I will try to use the GroundStation to control it. 

# Dialogue 12/10/2023
## some error report
Every time I try to ssh connect to the device after I turn on the ```~/apm.sh```, the terminal will report:
```bash
samgao1999@ubuntu:~$ ssh erle@192.168.1.106
ssh: connect to host 192.168.1.106 port 22: No route to host
```
I need to restart the erlebrain to reconnect to it. 

# Dialogue 19/10/2023
## APM_planner2
The basic workflow of connecting the erleBrain3 with host Machine:
**On erleBrain3**
```bash
sudo ~/apm.sh
```

In */opt/ros/kinetic/setup-mavros.bash* add the last line:
```
roslaunch mavros apm.launch fcu_url:="udp://:6001@" gcs_url:="udp://192.168.1.106:9000@192.168.1.105:6000?ids=255,252,1"
```
Where *192.168.1.106* is erleBrain ip address and *192.168.1.105* is host machine ip address. 
```bash
/opt/ros/kinetic/setup-mavros.bash
```
Start the apm_planner:
```
~/apm_planner/apm_planner/release/apmplanner2 
```
 ## The RC is not started
 When I try to calibrate the RC, I find there is an error *Radio Control is not active or turned on*. And at the same time the `3D Accelerometer Calibration` cannot be done without the rover. 
 After checking some sources online [APM_Planner2 set up for erle-robotics](https://erlerobotics.gitbooks.io/erle-robotics-erle-brain-a-linux-brain-for-drones/content/en/GCS/apmplanner.html#configuring-simple-mode) and [APM 2.5 Ardupilot controller- setup guide](https://www.youtube.com/watch?v=QAFdHnoae0s&ab_channel=75echo), I find we may need the hardware eg. *Radio-Control Receiver* and *Radio-Control Sender*. Also, the network problem in the lab shall be fixed. 

# Dialogue 26/10/2023
## Environment Setup
I set another modem in the lab, and connected the laptop, virtue machine, and erleBrain to the modem, This way I created a local network. 
The Segment and IP address are:
```bash
Host machine: 192.168.0.103
ErleBrain: 192.168.0.101
```
## the bound Remote Control signal
if we publish data on the topic `/mavros/rc/override mavros_msgs/OverriRCIn`, and then we subscribe the topic `/mavros/rc/override mavros_msgs/out`, we can observe that only two channels are valid:
**Prerequisite: Armed and Manual**
* ch1 is bounded by [1100, 1900]
* ch3 is bounded by [1208, 1900] 
Use `rostopic pub -1 /mavros/rc/override mavros_msgs/OverrideRCIn "channels: [0,0,0,0,0,0,0,0]"` to enable the remote control 

# Dialogue 8/11/2023
## Software
Now I am adopting the `teleop_twist_keyboard_cpp` method to control the rover with the keyboard. 
I am required to take the **sampling time** into consideration. So Even if the sampling time is changing, I still need to control the wheel to go through the whole control span with a certain amount of time. 
```
T = 1[s]
lower bound = 1100
upper bound = 1900
```
Now I am trying to add `arming` into the control program, I can call arming with the keyboard "," or "."

# Dialogue 1/12/2023
## The mapping from PWM to steering angle / longitudinal velocity
* The mapping from PWM to steering angle can be directly measured:  pick 10 different PWM signals and corresponding steering angle to check the relation.
* The mapping from PWM to longitudinal velocity: we need the Motion Capture System to measure that.

## The lateral controller design:
For the path following, the lateral controller is needed
* objective:
  * minimize the heading error and cross-track error
  * The limitations of the vehicle need to be satisfied: steering angle, lateral dynamics ...
* error:
  * heading error: the difference between the vehicle heading angle and the target path orientation. (both measured in global coordinates)
  * cross-track error: the distance between the reference point on the vehicle and the target path.
* Controller:
  * Pure-pursuit controller
  * Stanly controller
  * MPC controller
    
# Dialogue 3/12/2023
## Singularity
Install singularity:

Build an image from docker:
Follow the instructions from the [official guide](https://docs.sylabs.io/guides/3.0/user-guide/build_a_container.html)
```sudo singularity build --sandbox kinetic.sif docker://yabin/ros_kinetic_desktop_full```

To start the container, run the following command:

```
sudo singularity shell -w --hostname 192.168.0.100 kinetic.sif/ hostname
```
Substitute the argument `192.168.0.100` with your host machine IP address

```
sudo singularity shell -w --hostname 192.168.0.100 --bind ~/to_vm/:/root/from_host/ kinetic.sif/ hostname
```
Here we bind one folder `~/to_vm/` to the folder in the container `/root/from_host/`

```bash
source ~/catkin_ws/devel/setup.bash

```

```
cp -r ~/from_host/keyboard_control/ ~/catkin_ws/src/
```
Use the command to update the file in the ROS workspace

# Dialogue 8/12/2023
## Simulator
When building the simulation environment with `PX4 model` and `Gazebo-11` environment using the command:
```
make px4_sitl gazebo_rover
```
I always encounter a **linker** problem:
```bash
fatal error: opencv2/aruco.hpp: no such file or directory
```
the way to solve it is through:

In file `/home/samgao1999/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/CMakeLists.txt`
Add the path of the opencv4 `/usr/local/include/opencv4` to the configuration, make sure the `include_directories` look like this:
```
include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}/eigen3	# Workaround for Eigen3
  ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_MSG_INCLUDE_DIRS}
  ${MAVLINK_INCLUDE_DIRS}
  ${MAVLINK_INCLUDE_DIRS}/mavlink/v2.0 # Workaround for "fatal error: development/mavlink.h: No such file or directory"
  ${OGRE_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}/Paging		# Workaround for "fatal error: OgrePagedWorldSection.h: No such file or directory"
  # ${OpenCV_INCLUDE_DIRS}
  /usr/local/include/opencv4
  ${OpticalFlow_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
  )
```

# Dialogue 18/12/2023
## Simulator
We can open the simulator in the following way from [ROS with Gazebo Classic Simulation](https://docs.px4.io/main/en/simulation/ros_interface.html)
```
cd <PX4-Autopilot_clone>
DONT_RUN=1 make px4_sitl_default gazebo-rover
source ~/catkin_ws/devel/setup.bash    # (optional)
source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic
roslaunch px4 mavros_posix_sitl.launch
```
Open a new terminal and run the QGroundControl:
```
cd <Download>
./QGroundControl.AppImage
```
The reason is that the version of `opencv2` and `opencv4` is colliding, the cmake cannot find the right header file under `opencv4/opencv2`. 
