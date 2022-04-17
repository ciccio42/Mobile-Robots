#! /usr/bin/python3

import rospy                               # Import the Python library for ROS
from std_msgs.msg import String            # Import the String message from the std_msgs package
from geometry_msgs.msg import Twist         # Import the Twist Message from the geometry_msgs package

def talker():
  rospy.init_node('vel_publisher')                          # Initiate a Node named 'vel_publisher'     
  pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)  # Create a Publisher object
  move = Twist()
  rate = rospy.Rate(100)                                      #set a publish rate of 1 Hz
  
  while not rospy.is_shutdown():
    rospy.loginfo('START MOVING STRAIGHT FORWARD!....')
    move.linear.x=0.26
    move.angular.z=0
    pub.publish(move)
    rate.sleep()
  
try:
  talker()
except rospy.ROSInterruptException:
  pass

