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