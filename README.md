# Mobile-Robots course exercise

Repository containing the exercises for the *"Mobile Robots for Critical Mission"* course

* Francesco Rosa, frosa@unisa.it
* Marco Preziosi, mpreziosi@unisa.it

## How to run
```bash
cd ~
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/ros-planning/robot_pose_ekf.git
cd ../
catkin build
source devel/setup.bash
roslaunch exercises demo_[NUM_EXERCISE].launch
```