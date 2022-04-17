#! /usr/bin/python3

import rospy                                  # Import the Python library for ROS
from sensor_msgs.msg import LaserScan         # Import the Twist Message from the std_msgs package


def callback(msg):            
  rospy.loginfo('>>>>>>>>>>')
  rospy.loginfo('s1[0]')                      #The sensor returns a vector of 360 elements. 
  rospy.loginfo(msg.ranges[0])                #We simply print the values at 0, 90, 180 and 270 degrees

  rospy.loginfo('s2[90]')
  rospy.loginfo(msg.ranges[90])

  rospy.loginfo('s3[180]')
  rospy.loginfo(msg.ranges[180])

  rospy.loginfo('s4[270]')
  rospy.loginfo(msg.ranges[270])


rospy.init_node('laser_data')                           # Initiate a Node named 'laser_data'     
sub = rospy.Subscriber('scan', LaserScan, callback)     # Create a Subscriber object to the /scan topic
rospy.spin()

  
    
 



