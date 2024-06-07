# amerigo_control
Hardware Interface in ROS.

## 1. Prerequisites
* Ubuntu 20.04.
* ROS Noetic.
* Install [amerigo_description](https://github.com/0nhc/amerigo_description).

## 2. Install this package.
```sh
cd <your_ws>/src
git clone https://github.com/0nhc/amerigo_control.git
cd ..
rosdep install --from-path src --ignore-src -r -y
catkin_make
```
