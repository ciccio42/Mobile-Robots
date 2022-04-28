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

# SUPPOSE THAT THE STARTING POSITION IS KNOWN
start_position = Pose()
start_position.position.x = 0.0
start_position.position.y = 0.0
start_position.position.z = 0.0
start_position.orientation.w = 1.0

# create cmd_vel publisher
cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

# MAX LINEAR SPEED
V_MAX = 0.20
# MAX ANGULAR SPEED
OMEGA_MAX = 0.20
# TIME TO REACH DESIDER POSITION
TIME = 3 # s
# Flag to consider the user input valid
USER_INPUT_VALID = True
REACHED_WP = True
ALIGNMENT_COMPLETE = False

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

def alignement_elapsed(event=None):
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

def create_path():
    # create a sequence of waypoits [x,y,theta], with respect to map
    waypoints = []
    """
    waypoints.append([0.5, 0, 0])
    
    waypoints.append([1.0, 0.0, 0.0])
    
    waypoints.append([1.5, 0.0, 0])
    
    waypoints.append([2.0, 0.0, 0.0])
    
    waypoints.append([2.5, 0.0, 0.0])
    
    waypoints.append([3, 0.0, 0.0])"""

    
    # move forward of 0.5 m along the x
    waypoints.append([0.5, 0, 0])
    
    # turn right
    waypoints.append([0.5, -0.5, -math.pi/2])
    
    # turn left
    waypoints.append([1.0, -0.5, 0])
    
    # move forward
    waypoints.append([1.0, 0.0, math.pi/2])
    
    # turn left
    waypoints.append([0.5, 0.0, math.pi])
    
    # go to starting point
    waypoints.append([0, 0.0, math.pi])

    return waypoints

def spawn_robot():
    pass

def move_robot(cmd_vel_pub: rospy.Publisher, delta_x, delta_y, delta_theta):
    """This function is used to move the robot

    Parameters
    ----------
        cmd_vel_pub: rospy.Publisher
            Command publisher

        delta_x:
            Displacement to cover along x axis
        delta_y:
            Displacement to cover along y axis

        delta_theta:
            Displacement to cover about z axis

    Returns
    -------
        
    """
    global REACHED_WP
    REACHED_WP = False
    
    if abs(delta_theta) < 10**-3: # delta_theta is equal to zero
        # the robot is alligned with the desired orientation
        # go straight
        rospy.loginfo("Going straight.....")
        v_x = delta_x / TIME
        v_y = delta_y / TIME
        cmd_vel = Twist()
        cmd_vel.linear.x = abs(v_x)
        cmd_vel.linear.y = abs(v_y)
        cmd_vel.linear.z = 0.0
        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = 0.0
        cmd_vel_pub.publish(cmd_vel)
        rospy.Timer(rospy.Duration(secs=TIME),timer_elapsed, oneshot=True)         
    else:
        # the robot is not aligned with the target orientation
        rospy.loginfo("Aligning with next wp...")
        omega = delta_theta / TIME
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0
        cmd_vel.linear.z = 0.0
        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = omega
        cmd_vel_pub.publish(cmd_vel)
        rospy.Timer(rospy.Duration(secs=TIME),alignement_elapsed, oneshot=True)
        # wait until alignement is not complete
        global ALIGNMENT_COMPLETE
        ALIGNMENT_COMPLETE = False
        while not ALIGNMENT_COMPLETE:
            pass
        
        # Rotate delta
        tRz = tf.transformations.rotation_matrix(delta_theta, (0,0,1))[:-1,:-1].transpose()
        new_displacement = np.matmul(tRz, np.array([[delta_x, delta_y, 0.0]]).transpose())
        rospy.loginfo("Command afer alignment: delta_x {} - delta_y {} - delta_theta {}".format(new_displacement[0][0], new_displacement[1][0], delta_theta))
        # set new command
        rospy.sleep(1)
        v_x = new_displacement[0][0] / TIME
        v_y = new_displacement[1][0] / TIME
        cmd_vel = Twist()
        cmd_vel.linear.x = abs(v_x)
        cmd_vel.linear.y = abs(v_y)
        cmd_vel.linear.z = 0.0
        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = 0.0
        cmd_vel_pub.publish(cmd_vel)
        rospy.Timer(rospy.Duration(secs=TIME),timer_elapsed, oneshot=True)
    

def convert_wp_to_pose(waypoint):
    """This function is used to convert the waypoint = [x,y,theta] in a pose

    Parameters
    ----------
        waypoint: list
            Waypoint in input, in the format [x,y,theta]
    Returns
    -------
        pose: Pose
            Corresponding waypoint pose, defined with respect to odom
            
    """
    pose = Pose()
    pose.position.x = waypoint[0]
    pose.position.y = waypoint[1]
    pose.position.z = 0.0
    orientation = tf.transformations.quaternion_from_euler(0.0, 0.0, waypoint[2])
    pose.orientation.x = orientation[0]
    pose.orientation.y = orientation[1]
    pose.orientation.z = orientation[2]
    pose.orientation.w = orientation[3]
    rospy.loginfo("Waypoint pose {}".format(pose))
    return pose

def get_current_pose(tf_listener: tf.listener, start_frame:str, end_frame:str) -> Pose:
    """This function is used to get the current pose of the robot

    Parameters
    ----------
        tf_listener: tf.listener
            The current robot pose. It is the position and orientation from base_footprint to odom frame

        start_frame: str
            Frame name with respect to which describe the pose

        end_frame: str
            Name of the frame that we want to know the pose
    Returns
    -------
        pose: Pose
            The pose of the end_frame with respect to the start_frame (e.g. odom -> base_footprint)
            
    """
    try:
        t = tf_listener.getLatestCommonTime(start_frame, end_frame)
        position, quaternion = tf_listener.lookupTransform(start_frame, end_frame, t)
        rospy.loginfo("Current robot pose with respect to {}:\n position {}\n Quaternion {}".format(end_frame, position, quaternion))    
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
    """This function computes the the difference in position and orientation between the current pose and the desired one, 
    in order to generate the command

    Parameters
    ----------
        current_pose: Pose
            The current robot pose. It is the position and orientation from base_footprint to odom frame

        desired_pose: Pose
            Desired robot pose. It is the position and the orientation of the waypoiny with respect the odom frame
    Returns
    -------
        delta_x: float
            The distance between current position and desired one, along x axis
        delta_y: float
            The distance between current position and desired one, along y axis
        delta_theta: float
            The difference in orientation between the current position and desired one, about x axis
            
    """
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

    
    rospy.logdebug("\nA_base_footprint_to_odom:\n{}".format(A_base_footprint_to_odom))
    rospy.logdebug("\nA_odom_to_wp:\n{}".format(A_odom_to_wp))
    rospy.logdebug("\nvA_base_footprint_to_wp:\n{}".format(A_base_footprint_to_wp))
    

    delta_x = A_base_footprint_to_wp[0][3]
    delta_y = A_base_footprint_to_wp[1][3]

    r,p,y = tf.transformations.euler_from_matrix(A_base_footprint_to_wp[:3,:3])
    delta_theta = y

    rospy.loginfo("Command delta_x {} - delta_y {} - delta_theta {}".format(delta_x, delta_y, delta_theta))

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
    waypoints = create_path()
    marker_array = utils.create_markers(waypoints)
    rospy.loginfo("Publishing markers")
    marker_array_pub.publish(marker_array)

    rospy.loginfo("Start following the planned trajectory")
    for i in range(len(waypoints)):
        
        key = input("Press any key to continue: ")

        rospy.loginfo("Waypoint {} - {}".format(i+1, waypoints[i]))
        # get current robot pose
        current_pose = get_current_pose(tf_listener, "base_footprint", "odom")
        # get desired robot pose
        desired_pose = convert_wp_to_pose(waypoints[i])
        # compute the difference between the current pose and the desired one
        delta_x, delta_y, delta_theta = compute_pose_difference(current_pose=current_pose, desired_pose=desired_pose)
        rospy.loginfo("\n")
        # compute the time needed to reach the desired pose with a given velocity
        rospy.loginfo("Move toward the waypoint....")
        move_robot(cmd_vel_pub, delta_x, delta_y, delta_theta)

        while not REACHED_WP:
            pass
        
if __name__ == '__main__':
    main()