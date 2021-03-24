
## Quick tutorial

Download 
[px4 + gazebo](https://docs.px4.io/master/en/dev_setup/dev_env_linux_ubuntu.html)
then in px4 folder
```bash
make px4_sitl_default gazebo
```

Download [QGroundControl](https://docs.qgroundcontrol.com/master/en/releases/daily_builds.html) and:
```bash
chmod +x QGroundControl.AppImage
```
Download [ROS Melodic desktop only version](http://wiki.ros.org/melodic/Installation/Ubuntu)

Download [mavsdk newest version](https://github.com/mavlink/MAVSDK/releases)

Make catkin_ws folder, add this repo to src and catkin_ws folder:
```bash
catkin_make
```
if everything is okay
```bash 
roscore
rosrun mavsdk_interface node udp://:14540
source devel/setup.bash
```