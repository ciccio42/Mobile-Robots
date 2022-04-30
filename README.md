# Mobile-Robots course exercise

Repository containing the exercises for the *"Mobile Robots for Critical Mission"* course

* Francesco Rosa, frosa@unisa.it
* Marco Preziosi, mpreziosi@unisa.it

# Dependencies
Follow the rules [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/) to install turtlebot packages

## How to run
1. Create a ROS workspace
2. Read *Dependencies*
3. Move in *src* folder
```bash
git clone https://github.com/ciccio42/Mobile-Robots.git # or copy the exercises directory into the src folder
cd Mobile-Robots && mv exercises ../ && cd ../ && rm -rf Mobile-Robots
cd ../
catkin build
source devel/setup.bash
roslaunch exercises demo_[NUM_EXERCISE].launch # NUM_EXERCISE = 1,2,3,4
```