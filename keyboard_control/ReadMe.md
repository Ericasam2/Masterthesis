**Version 1.0**
Original by: Sam Gao (S.Gao-9@student.tudelft.nl) on 25/03/2024

| Edited by: | To version: | on Date: | Notes                                                        |
| ---------- | ----------- | -------- | ------------------------------------------------------------ |
| Sam Gao       | 1.0      | 25/03/2024  | Add the framework of the data collecting, rover keyboard control and simulation|
| NAME       | ...         |          |                                                              |
|            |             |          |                                                              |

### Open Issues

- TASK, While collecting data, the MOCAP is sending wrong position data due to communication delay
- TASK, ...

# Folder Structure

| File/Folder name | Description                                                  |
| ---------------- | ------------------------------------------------------------ |
| keyboard_control | Send the PWM value to "rc/override" topic to control the rover with keyboard inputs |
| mocap_optitrack | Get the localization data from MOCAP |
| rviz_car_model | Perform the data collection, rviz simulation while rover localization |
|  |  |
|  |  |

# Anything

## How to use

### Keyboard Control
**Important:**\
the `/app/catkin_ws/devel/setup.bash` in every new bash need to be sourced. 


**Step 1:**\
Start a new bash and run the **singularity** container.
```bash
sudo singularity shell -w --hostname 192.168.0.100 --bind ~/catkin_ws/src/keyboard_control:/app/catkin_ws/src/keyboard_control kinetic.sif/ hostname
``` 
In the **singularity** container, source the code and run:
```bash
source /app/catkin_ws/devel/setup.bash
roscore
```
If the `/app/catkin_ws/devel/setup.bash` is reported missing or not exist, perform the step2 first. 

**Step 2:**\
Start a new bash and run the **singularity** container, substitute the address `192.168.0.100` with `<your host machine ip address>` (Here is a recap of the setup procedure introduced in [wiki](../../Wiki/HowtoInstallNewRover.md))
```bash
sudo singularity shell -w --hostname 192.168.0.100 --bind ~/catkin_ws/src/keyboard_control:/app/catkin_ws/src/keyboard_control kinetic.sif/ hostname
``` 

Source the ROS workspace before running the code
```bash
source /app/catkin_ws/devel/setup.bash
```

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

### rviz_car_model
Launch the node to receive the localization data from MOCAP, record the localization data to file `data_output.csv`, use **rviz** to visualize the rover state and the coordination system.  
```bash
roslaunch rviz_car_model test.launch
```

- ANY OTHER IMPORTANT INFORMATION
  (If there is any other information that needs to be known by the user of a folder, please include it too)