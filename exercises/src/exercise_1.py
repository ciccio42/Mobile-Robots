#! /usr/bin/python3

# ROS import
from turtle import pos
import rospy, rospkg
import tf
from tf import TransformListener
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PointStamped, Pose, Twist

import os, sys
import math
import numpy as np

# import utils
package_path = rospkg.RosPack().get_path('exercises')
sys.path.append(os.path.join(package_path, "scripts"))
import utils
from utils import V_MAX, OMEGA_MAX, TIME, USER_INPUT_VALID, REACHED_WP, ALIGNMENT_COMPLETE, \
                  ALIGNMENT_COMPLETE, STD_DEV, MEAN, MEAN_ROT, STD_DEV_ROT, MEAN_LASER_X, STD_DEV_LASER_VAR_X, \
                  MEAN_LASER_Y, STD_DEV_LASER_VAR_Y, MEAN_ORIENTATION_IMU, STD_DEV_ORIENTATION_IMU, CLIP_ON_VARIATION_MOTION_MODEL

# create cmd_vel publisher
cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

def timer_elapsed(event=None):
    # stop the robot
    cmd_vel = Twist()
    cmd_vel.linear.x = 0.0
    cmd_vel.linear.y = 0.0
    cmd_vel.linear.z = 0.0
    cmd_vel.angular.x = 0.0
    cmd_vel.angular.y = 0.0
    cmd_vel.angular.z = 0.0
    cmd_vel_pub.publish(cmd_vel)
    global REACHED_WP, USER_INPUT_VALID
    REACHED_WP = True
    USER_INPUT_VALID = True

def alignment_elapsed(event=None):
    # stop the robot
    cmd_vel = Twist()
    cmd_vel.linear.x = 0.0
    cmd_vel.linear.y = 0.0
    cmd_vel.linear.z = 0.0
    cmd_vel.angular.x = 0.0
    cmd_vel.angular.y = 0.0
    cmd_vel.angular.z = 0.0
    cmd_vel_pub.publish(cmd_vel)
    global ALIGNMENT_COMPLETE
    ALIGNMENT_COMPLETE = True

def move_robot(cmd_vel_pub: rospy.Publisher, delta_x, delta_y, delta_theta):
    global REACHED_WP
    REACHED_WP = False

    
    if abs(delta_theta) < 10**-3: # delta_theta is equal to zero
        # the robot is alligned with the desired orientation
        # go straight
        rospy.loginfo("\nGoing straight with trapezoidal motion.....")
        utils.trapezoidal_motion(cmd_vel_pub, delta_x)
        timer_elapsed()
    else:
        # the robot is not aligned with the target orientation
        rospy.loginfo("\nAligning with next wp...")
        omega = delta_theta / TIME
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0
        cmd_vel.linear.z = 0.0
        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = omega
        cmd_vel_pub.publish(cmd_vel)
        rospy.Timer(rospy.Duration(secs=TIME),alignment_elapsed, oneshot=True)
        # wait until alignment is not complete
        global ALIGNMENT_COMPLETE
        ALIGNMENT_COMPLETE = False
        while not ALIGNMENT_COMPLETE:
            pass
        
        # Rotate delta
        tRz = tf.transformations.rotation_matrix(delta_theta, (0,0,1))[:-1,:-1].transpose()
        new_displacement = np.matmul(tRz, np.array([[delta_x, delta_y, 0.0]]).transpose())
        rospy.loginfo("\nCommand afer alignment: delta_x {} - delta_theta {}".format(new_displacement[0][0], new_displacement[1][0], delta_theta))
        # set new command
        rospy.sleep(1)
        v_x = new_displacement[0][0] / TIME
        v_y = new_displacement[1][0] / TIME
        utils.trapezoidal_motion(cmd_vel_pub, new_displacement[0][0])
        timer_elapsed()

    

def convert_wp_to_pose(waypoint):
    pose = Pose()
    pose.position.x = waypoint[0]
    pose.position.y = waypoint[1]
    pose.position.z = 0.0
    orientation = tf.transformations.quaternion_from_euler(0.0, 0.0, waypoint[2])
    pose.orientation.x = orientation[0]
    pose.orientation.y = orientation[1]
    pose.orientation.z = orientation[2]
    pose.orientation.w = orientation[3]
    rospy.loginfo("\nWaypoint pose {}".format(pose))
    return pose

def get_current_pose(tf_listener: tf.listener, start_frame:str, end_frame:str) -> Pose:
    try:
        t = tf_listener.getLatestCommonTime(start_frame, end_frame)
        position, quaternion = tf_listener.lookupTransform(start_frame, end_frame, t)
        rospy.logdebug("\nCurrent robot pose with respect to {}:\n position {}\n Quaternion {}".format(start_frame, position, quaternion))    
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr("Lookup Transform failed")

    pose = Pose()
    pose.position.x = position[0]
    pose.position.y = position[1]
    pose.position.z = position[2]
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]
    return pose

def compute_pose_difference(current_pose: Pose, desired_pose: Pose):
    # current pose A{base_footprint}_{odom}
    # desired pose A{odom}_{wp}

    # create the homogeneous transformation from base_footprint to odom
    A_base_footprint_to_odom = np.zeros((4,4))
    A_base_footprint_to_odom[:3, :3] = tf.transformations.quaternion_matrix([current_pose.orientation.x, current_pose.orientation.y,
                                                                     current_pose.orientation.z, current_pose.orientation.w])[:3, :3]
    A_base_footprint_to_odom[0][3] = current_pose.position.x
    A_base_footprint_to_odom[1][3] = current_pose.position.y
    A_base_footprint_to_odom[2][3] = current_pose.position.z
    A_base_footprint_to_odom[3][3] = 1

    # create the homogeneous transformation from odom to waypoint
    A_odom_to_wp = np.zeros((4,4))
    A_odom_to_wp[:3, :3] = tf.transformations.quaternion_matrix([desired_pose.orientation.x, desired_pose.orientation.y,
                                                                     desired_pose.orientation.z, desired_pose.orientation.w])[:3, :3]
    A_odom_to_wp[0][3] = desired_pose.position.x
    A_odom_to_wp[1][3] = desired_pose.position.y
    A_odom_to_wp[2][3] = desired_pose.position.z
    A_odom_to_wp[3][3] = 1
    
    # define the wp with respect to current pose
    A_base_footprint_to_wp = np.matmul(A_base_footprint_to_odom, A_odom_to_wp)

    """
    rospy.loginfo("\nA_base_footprint_to_odom:\n{}".format(A_base_footprint_to_odom))
    rospy.loginfo("\nA_odom_to_wp:\n{}".format(A_odom_to_wp))
    rospy.loginfo("\nvA_base_footprint_to_wp:\n{}".format(A_base_footprint_to_wp))
    """

    delta_x = A_base_footprint_to_wp[0][3]
    delta_y = A_base_footprint_to_wp[1][3]

    r,p,y = tf.transformations.euler_from_matrix(A_base_footprint_to_wp[:3,:3])
    delta_theta = y

    rospy.loginfo("\nCommand delta_x {} - delta_y {} - delta_theta {}".format(delta_x, delta_y, delta_theta))
    
    return delta_x, delta_y, delta_theta

def main():
    rospy.init_node("exe_1_node")
    rate = rospy.Rate(0.2)
    
    # marker publisher
    marker_array_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)

    # create transform listener
    tf_listener = TransformListener()

    tf_listener.waitForTransform("/odom", "/imu_link", rospy.Time(), rospy.Duration(20.0))
    
    # get the path
    waypoints = utils.create_path()
    marker_array = utils.create_markers(waypoints)
    rospy.loginfo("\nPublishing markers")
    marker_array_pub.publish(marker_array)

    rospy.loginfo("\nStart following the planned trajectory")
    for i in range(len(waypoints)):
        rospy.loginfo("\n###########################################\n")

        key = input("Press any key to continue: ")

        rospy.loginfo("\nWaypoint {} - {}".format(i+1, waypoints[i]))
        # get current robot pose
        current_pose = get_current_pose(tf_listener, "base_footprint", "odom")
        # get desired robot pose
        desired_pose = convert_wp_to_pose(waypoints[i])
        # compute the difference between the current pose and the desired one
        delta_x, delta_y, delta_theta = compute_pose_difference(current_pose=current_pose, desired_pose=desired_pose)
        rospy.loginfo("\n")
        # compute the time needed to reach the desired pose with a given velocity
        rospy.loginfo("\nMove toward the waypoint....")
        move_robot(cmd_vel_pub, delta_x, delta_y, delta_theta)

        while not REACHED_WP:
            pass
        
        rospy.loginfo("\nWhere the robot is: \n{}".format(get_current_pose(tf_listener, 'odom', 'base_footprint')))
        rospy.loginfo("\nWhere the robot is supposed to be, based on motion model: \n{}".format(desired_pose))

if __name__ == '__main__':
    main()