import math
import os, sys
import rospy, rospkg
from std_srvs.srv import Empty
from visualization_msgs.msg import MarkerArray
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, PoseArray, PointStamped, Pose
from sensor_msgs.msg import Imu,LaserScan
from move_base_msgs.msg import MoveBaseAction
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
import tf.transformations 

def timer_elapsed(event=None):
    rospy.loginfo("Cleanning coastmap")
    clear_costmaps_client()
   
if __name__ == '__main__':

    rospy.init_node("coastmap_cleaner")
    
    # create service client to clean up the local coastmap
    rospy.loginfo("Waiting for clear_costmaps...")
    rospy.wait_for_service("/move_base/clear_costmaps")
    clear_costmaps_client = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)
    rospy.Timer(rospy.Duration(secs=10),timer_elapsed, oneshot=False)
    rospy.spin()