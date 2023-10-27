## Description
The software platform need to include some functions:
1. The calibration for the remote control.
2. A program to control the steering and acceleration of the rover.
3. A program to monitor the speed/acceleration of the rover.
4. A program to set the target state(speed/location) and control the rover to reach the target.

## Calibration
This can be done by apm_planner. 

## Steering Control
The Remote Control can be done by 
```bash
rostopic pub -1 /mavros/rc/override mavros_msgs/OverrideRCIn "channels: [value_1,0,value_2,0,0,0,0,0]"
```
the `value_1` is used to control the steering angle, and the `value_2` is used to control the acceleration. 
We need a function `RCsteeringControl` that can do the following:
* Use the keyboard to perform the control
* `w`--> go straight, `s`--> go backward `d`--> turn right `a`--> turn left.
* the steering logic:
  * if only `a` is pressed: the value_1 keep go small (> 1100);
  * if only `d` is pressed: the value_1 keep go big (< 1900);
  * if neither `a` nor `d` is pressed: the value_1 return to neutral (1500).
  * else: do nothing. 
