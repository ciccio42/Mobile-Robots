#! /usr/bin/python3

from asyncio import FastChildWatcher
import rospy                                  # Import the Python library for ROS
from sensor_msgs.msg import LaserScan         # Import the Twist Message from the std_msgs package
from geometry_msgs.msg import Twist

# CONSTANT DEFINITION
LASER_SCAN_TOPIC = "scan"
COLLISION_DIRECTIONS = [0, 90, 180]
COLLISION_THRESHOLD = 0.80
LINEAR_VELOCITY = 0.26
ANGULAR_VELOCITY = 0.26

def move(pub, turn=False):
    cmd_vel = Twist()
    if turn:
        # apply only angular velocity
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = ANGULAR_VELOCITY
    else:
        # apply only linear velocity
        cmd_vel.linear.x = LINEAR_VELOCITY
        cmd_vel.angular.z = 0.0

    pub.publish(cmd_vel)

def stop(pub):
    cmd_vel = Twist()
    cmd_vel.linear.x = 0.0
    cmd_vel.angular.z = 0.0
    pub.publish(cmd_vel)

def check_for_obstacle():
    rospy.loginfo("\nCheck for obstable")
    
    # get message
    laser_scan_msg = rospy.wait_for_message(LASER_SCAN_TOPIC, LaserScan)
    obstacles_dir = [] 
    for i, angle in enumerate(COLLISION_DIRECTIONS):
        # check for obstacle at given angle
        range = laser_scan_msg.ranges[angle]
        if range <= COLLISION_THRESHOLD:
            obstacles_dir.append(True)
        
        obstacles_dir.append(False)

    return obstacles_dir
    
def main():
    rospy.init_node('random walker')                          
    cmd_vel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
    
    while(True):
        # check for obstacle
        obstacle_flag_list = check_for_obstacle()
        for dir, obstacle_flag in enumerate(obstacle_flag_list):
            if obstacle_flag:
                stop(cmd_vel_publisher)
                rospy.loginfo("\nObstacle found, at {}".format(dir*90))
                if dir == 0:
                    # obstacle in front of the robot
                    # is there any obstacle left or right?
                    if obstacle_flag_list[1]==True or obstacle_flag_list[2]==True:
                        move(cmd_vel_publisher, turn=True)
                if dir == 1 or dir == 2:
                    # there is an obstacle left or right
                    # is there an obstacle in front of the robot?
                    if obstacle_flag_list[0]==True:
                        move(cmd_vel_publisher, turn=False)
                break
            else:
                rospy.loginfo("\nObstacle not found, keep going")
                move(cmd_vel_publisher)

if __name__ == '__main__':
    rospy.loginfo("\nStarting random walker")
    main()