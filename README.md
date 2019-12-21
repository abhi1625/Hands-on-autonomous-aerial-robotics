<p><h1>FlyingNimbus <img src="./images/FlyingNimbus.gif" width=15% height=15%> </h1></p>
Final project for ENAE788M

## Overview
The final project in the ENAE788M course requires us to complete the following course autonomously 

![track](images/Track.png)

This is to be done using the hardware setup provided in the course, the details of which can be found [here](https://github.com/prgumd/PRGFlyt/wiki/PRGHusky). Once you setup the hacked bebop quadrotor follow the instructions to run the course
## Required Dependencies

## Using this repo

## Other useful info for playing around with the parrot bebop 
### Connecting to bebop shell
* press the power button of bebop 4 times in succession
* connect to bebop wifi and use the following command:
  ```
  telnet 192.168.42.1
  ```

You should be able to login into the shell

### Hacking Bebop's internal PID
Before you begin to tune the PID, remount the disk with read and write permissions using:
```
mount -o remount, rw /
```
Now you can change parameters in any of the Bebop's internal file.

The .cfg file which contains all the gains for different modes in Bebop are present in `/etc/colibry/common/controller.cfg`.
- Tune the gains under `ctrlGainPosition`


### Sending Commands to Bebop
1. Connect to Bebop 2 Wifi

2. `roscore`

3. `roslaunch bebop_driver bebop_node.launch`

4. Use the following commands to do specific tasks:
Takeoff:
``rostopic pub --once /bebop/takeoff std_msgs/Empty``
Land:
``rostopic pub --once /bebop/land std_msgs/Empty``
Movement:
```
rostopic pub --once /bebop/cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" 
```
Change values of x,y,z as needed.


5. To enable camera without stabilization
```
~/bebop_ws/./video_stabil.sh
```

### Other useful links
* [Leapord camera drivers](https://github.com/chahatdeep/ubuntu-for-robotics/tree/master/Drivers)
* [Bebop Hacking Guide](http://fargesportfolio.com/wp-content/uploads/2018/01/BeebopHackingGuide1_7_2.pdf)
* [Bebop_autonomy documentation](https://buildmedia.readthedocs.org/media/pdf/bebop-autonomy/latest/bebop-autonomy.pdf)
* [bebop ros package](https://bebop-autonomy.readthedocs.io/en/latest/installation.html)
* [Wifi driver](https://github.com/chahatdeep/rtl8812au-wifi-driver)

