# 1.Test service and actionlib procedure

### Install necessary dependencies

```sh
$ sudo apt-get install -y ros-${ROS_DISTRO}-desktop-full
$ sudo apt-get install -y ros-${ROS_DISTRO}-navigation
$ sudo apt-get install -y ros-${ROS_DISTRO}-gmapping 
$ sudo apt-get install -y ros-${ROS_DISTRO}-joy 
$ sudo apt-get install -y ros-${ROS_DISTRO}-laser-proc 
$ sudo apt-get install -y ros-${ROS_DISTRO}-urg-c 
$ sudo apt-get install -y ros-${ROS_DISTRO}-ecl 
$ sudo apt-get install -y ros-${ROS_DISTRO}-rosserial-python 
$ sudo apt-get install -y ros-${ROS_DISTRO}-rosbridge-suite 
$ sudo apt-get install -y ros-${ROS_DISTRO}-robot-upstart 
$ sudo apt-get install -y ros-${ROS_DISTRO}-bfl 
```
### In your catkin workspace

```sh
$ cd ~/investhkrobot_ros
$ catkin_make
$ echo "source ~/investhkrobot_ros/devel/setup.bash" >> ~/.bashrc  (optional)
$ source ~/.bashrc
```

### Make the Python node executable: 

```sh
$ chmod +x scripts/ar_rotate_handler_client.py
$ chmod +x scripts/ar_rotate_handler_server.py
```

### Test the service, returning type is float64[]
- `Service` can listen to the response and make sure the result can be heard, but service will send only one response normally.

		$ roscore
		$ rosrun robot_control ar_rotate_handler_server.py
		$ rosrun robot_control ar_rotate_handler_client.py

### Test the actionlib, returning type is float64[]
- `Action` can keep watching the whole process, but you have to listen to the ACTION/result topic, and the result will publish once only.

		$ roscore
		$ rosrun robot_control ArRotateHandler_server
		$ rosrun robot_control ArRotateHandler_client
