# Quickly Setting Up ROS2 Workspace

## Setup BILLEE_PKG
There are two files to quickly setup a ros2 workspace. Under [billee_pkg_setup](billee_pkg_setup) there is a bash script that setups all scripts that are saved inside of the bille_pkg branch in the BILL-EE repo. That way incase an accident happens on the local storage of either Jetson or Raspberry pi it can quickly be recovered.

## Setting up a new workspace
Under [ros2_setup](ros2_setup) there is another bash script so you can immediately create a ros2 workspace with a publisher and subscriber script just to extra verify if the package built properly.

# Steps Before Running Bash Script
## Install colcon build and your ros2 distro
To install colcon tools run the following command:
```
sudo apt install python3-colcon-common-extensions
```

