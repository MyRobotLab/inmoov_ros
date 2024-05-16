### Forked from https://github.com/alansrobotlab/inmoov_ros

Thank you Alan for your work!

**The xenial-kinetic branch is currently being restructured to make it easier to adapt to custom InMoov designs.**  

Please check the xenial-kinetic milestones to see the direction we're heading and our status
https://github.com/alansrobotlab/inmoov_ros/milestones/xenial_kinetic

These are massive changes to the framework, but once it's all in, it will be alot easier to set up ROS with your specific robot.  Stay tuned!

Until then, once you have the packages installed, the following commands should work:
 - roslaunch inmoov_description display.launch (to pull up the rviz urdf model)
 - (the rest is currently hard coded to one robot, we're working on that)

## Alan's InMoov ROS Introduction
![enter image description here](http://i.imgur.com/bweApZH.png)


---------

### What Is It?
This is a ROS software stack that connects a dedicated PC to an InMoov robot.  
It currently implements the following:

 - a fully articulated URDF model of InMoov
 - inmoov firmware that integrates rossserial_arduino for communication
 - inmoov_msgs that define communication between host pc and arduino
 - trainer module to set arduino eeprom values and calibrate each servo

### What You Need To Get Started
> This works with a specific technology stack.  It can be run natively on a PC, or in VMWare Player.  Everything is tied to compatibility with MoveIt!.  
>  
>  MoveIt! is currently available for ROS Indigo, and Indigo is tied to Ubuntu 14.04 LTS, so everything fits together from there.  
>   
>   As soon as they release MoveIt! packages for ROS Kinetic, the stack will be updated to Ubuntu 16.04 + ROS Kinetic + MoveIt!

#### What you'll need:

 - VMWare Player
 (https://my.vmware.com/en/web/vmware/free#desktop_end_user_computing/vmware_workstation_player/12_0)
 - Ubuntu 14.04 LTS (http://www.ubuntu.com/download/alternative-downloads) 
 - ROS Indigo (http://wiki.ros.org/indigo/Installation/Ubuntu)
 - MoveIt! (http://moveit.ros.org/install/)

### How to Install It:
Copy these packages into your {inmoov_ros}/src folder
Run the following commands from the root of your {inmoov_ros} folder:
  
    catkin_make              #build/rebuild all projects
    source devel/setup.bash  #let ROS know about all of your new packages

### How to use it:
Run the following commands:

    rosrun inmoov_tools set_parameters.py        #loads parameters into server
    roslaunch inmoov_tools servobus.launch       #launches arduino interfaces
    roslaunch inmoov_description display.launch  #launches rviz
    rosrun inmoov_tools trainer.py               #launches trainer module


### Next Steps (todo list)
 - urdf:  fix right shoulder out rotation
 - urdf:  fix right wrist rotation
 - trainer:  move all arduino communication to service calls
 - trainer:  only publish joint commands to joint_command topic
 - node:  write node that sends joint commands to arduino through service calls
 - pose:  migrate pose module to pyqt4
 - headdemo:  migrate headdemo module to pyqt4


### Current system
- Ubuntu 20.04 LTS
- ROS Noetic
- MoveIt! 
- Python 3.8
- OpenCV 4.2.0


## ROS2 setup
```bash

# Setup Sources and Keys

sudo apt update && sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg


# Then, add the repository to your sources list:
echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2
sudo apt update
sudo apt install ros-foxy-desktop

# Environment Setup
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
source ~/.bashrc

# workspace
mkdir -p ~/ros2_workspace/src
cd ~/ros2_workspace/
colcon build
source ~/ros2_workspace/install/setup.bash



```


### kinect v2 dependencies
```bash
sudo apt-get update
sudo apt-get install build-essential cmake pkg-config python3-rosdep2 \
libusb-1.0-0-dev libturbojpeg0-dev libglfw3-dev libopenni2-dev

cd ~
git clone https://github.com/OpenKinect/libfreenect2.git
cd libfreenect2
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/freenect2
make
sudo make install
sudo cp ../platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/





```


```bash

# this shell
source /opt/ros/noetic/setup.bash

# all shells
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# for catkin_ws
source ~/catkin_ws/devel/setup.bash

# install ros packages
# FIXME diff between desktop-full and this project
# rossserial_arduino thread ... makeit at least 3 others
sudo apt install ros-noetic-rosserial-python
sudo apt install ros-noetic-rosserial-arduino
sudo apt install ros-noetic-kinect2

cd  ~/catkin_ws
catkin_make

# generate ros libraries for arduino
rosrun rosserial_arduino make_libraries.py ~/Arduino/libraries

roslaunch inmoov_description display.launch

# terminate all nodes if master is still running
rosnode kill -a

pkill -f ros

```

roslaunch kinect-demo
launch files

```bash
roslaunch inmoov_bringup bringup.launch # initializes and ui with joints
roslaunch inmoov_tools servobus.launch # requires real serial port
roslaunch inmoov_tools trainer-stack.launch 
# ERROR: cannot launch node of type [inmoov_tools/trainer.py]: Cannot locate node of type [trainer.py] in package [inmoov_tools]. Make sure file exists in package path and permission is set to executable (chmod +x)
roslaunch inmoov_tools kinect-full-demo.launch # needs kinect2_bridge
roslaunch inmoov_tools webcams.launch
roslaunch inmoov_tools dev.launch # needs real serial port
roslaunch inmoov_tools demo.launch # "all the things" paths are not quite right requires serial
roslaunch inmoov_tools righteye.launch
roslaunch inmoov_tools videoserver.launch
roslaunch inmoov_tools sergei-help.launch
roslaunch inmoov_tools makercon-demo.launch
roslaunch inmoov_tools kinect-demo.launch
roslaunch inmoov_description moveit-display.launch
roslaunch inmoov_description gazebo.launch
roslaunch inmoov_description display.launch
```

```bash
# quick envs
cd  ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```


kinect2 challenges
* need to hack the CMakelists.txt and add 
```
# PCL gcc error
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# PCL vs OpenCV 4.2.0 - PCL wants 3.4.0 - this site has the fix
https://github.com/paul-shuvo/iai_kinect2_opencv4

```


```
sudo apt install ros-humble-moveit
```

rosserial is gone ...

New fancy
https://micro.ros.org/