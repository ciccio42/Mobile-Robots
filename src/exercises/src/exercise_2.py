#! /usr/bin/python3

# ROS import
import rospy, rospkg
import tf
from tf import TransformListener
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PointStamped, Pose, Twist, PoseWithCovariance, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


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

# Robot State based on motion model
motion_model_estimated_state = PoseWithCovarianceStamped() 

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
        rospy.loginfo("\nGoing straight with trapezoidal motion.....")
        # where actually I go due to the noise in the movement model
        new_displacement_w_noise = [delta_x, delta_y] + np.clip(np.random.normal(MEAN, STD_DEV, size=2), -CLIP_ON_VARIATION_MOTION_MODEL, CLIP_ON_VARIATION_MOTION_MODEL)
        rospy.loginfo("\nCommand afer noise: delta_x {} - delta_theta {}".format(new_displacement_w_noise[0], 0.0))
        utils.trapezoidal_motion(cmd_vel_pub, new_displacement_w_noise[0])    
        timer_elapsed()     
    else:
        # the robot is not aligned with the target orientation
        rospy.loginfo("\nAligning with next wp...")
        # add noise to ration
        theta_with_noise = delta_theta + np.random.normal(MEAN_ROT, STD_DEV_ROT)
        rospy.loginfo("\nCommand rotation, with added noise: delta_theta {}".format(theta_with_noise))
        omega = theta_with_noise / TIME
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
        # new_displacement_wo_noise -> my belief where I think to be
        new_displacement_wo_noise = np.matmul(tRz, np.array([[delta_x, delta_y, 0.0]]).transpose())
        rospy.loginfo("\nCommand after alignment: delta_x {} - delta_theta {}".format(new_displacement_wo_noise[0][0], 0.0))
        # set new command
        rospy.sleep(1)
        # where actually I go due to the noise in the movement model
        noise = np.clip(np.random.normal(MEAN, STD_DEV, size=2), -CLIP_ON_VARIATION_MOTION_MODEL, CLIP_ON_VARIATION_MOTION_MODEL)
        rospy.loginfo("\nNoise: {}".format(noise))
        new_displacement_w_noise = [new_displacement_wo_noise[0][0], new_displacement_wo_noise[1][0]] + noise
        rospy.loginfo("\nCommand after alignment, with added noise noise: delta_x {} - delta_theta {}".format(new_displacement_w_noise[0], 0.0))
        v_x = new_displacement_w_noise[0] / TIME
        v_y = new_displacement_w_noise[1] / TIME
        utils.trapezoidal_motion(cmd_vel_pub, new_displacement_w_noise[0])
        timer_elapsed()
    
def convert_wp_to_pose(waypoint):
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
    pose = Pose()
    pose.position.x = waypoint[0]
    pose.position.y = waypoint[1]
    pose.position.z = 0.0
    orientation = tf.transformations.quaternion_from_euler(0.0, 0.0, waypoint[2])
    pose.orientation.x = orientation[0]
    pose.orientation.y = orientation[1]
    pose.orientation.z = orientation[2]
    pose.orientation.w = orientation[3]
    #rospy.logdebug("Waypoint pose {}".format(pose))
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
        rospy.logdebug("Current robot pose with respect to {}:\n position {}\n Quaternion {}".format(start_frame, position, quaternion))    
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

    
    rospy.logdebug("A_base_footprint_to_odom:\n{}".format(A_base_footprint_to_odom))
    rospy.logdebug("A_odom_to_wp:\n{}".format(A_odom_to_wp))
    rospy.logdebug("vA_base_footprint_to_wp:\n{}".format(A_base_footprint_to_wp))
    

    delta_x = A_base_footprint_to_wp[0][3]
    delta_y = A_base_footprint_to_wp[1][3]

    r,p,y = tf.transformations.euler_from_matrix(A_base_footprint_to_wp[:3,:3])
    delta_theta = y

    rospy.loginfo("\nCommand delta_x {} - delta_y {} - delta_theta {}".format(delta_x, delta_y, delta_theta))
    
    return delta_x, delta_y, delta_theta

def initialize_robot_state(tf_listener: tf.listener):
    """Initialize the robot state by setting the motion_model_estimated_state global variable

    Parameters
    ----------
        tf_listener: tf_listener
            Transform listener
    Returns
    -------
    
    """

    # get the initial pose from base_footprint to odom
    global initial_pose
    initial_pose = get_current_pose(tf_listener=tf_listener, start_frame="base_footprint", end_frame="odom")
    # Initialize the motion_model_estimated_state with the supposed known initial position
    global motion_model_estimated_state
    #----INITIAL POSE----#
    motion_model_estimated_state.pose = PoseWithCovariance()
    motion_model_estimated_state.pose.pose = initial_pose
    # The covariance is zero during the initialization, since we suppose to know the initial position
    motion_model_estimated_state.pose.covariance = np.zeros(36, np.float64)
    for i in range(6):
        diagonal_index = (i*6)+i
        if i < 3:
            if i == 0:
                motion_model_estimated_state.pose.covariance[diagonal_index] = motion_model_estimated_state.pose.covariance[diagonal_index] + 10**-5
            elif i == 1:
                motion_model_estimated_state.pose.covariance[diagonal_index] = motion_model_estimated_state.pose.covariance[diagonal_index] + 10**-5
            elif i == 2:
                motion_model_estimated_state.pose.covariance[diagonal_index] = motion_model_estimated_state.pose.covariance[diagonal_index] + 1000000000000.0

        else:
            if i == 5:
                motion_model_estimated_state.pose.covariance[diagonal_index] = motion_model_estimated_state.pose.covariance[diagonal_index] + 0.001
            else:
                motion_model_estimated_state.pose.covariance[diagonal_index] = motion_model_estimated_state.pose.covariance[diagonal_index] + 1000000000000.0
    
    #---- Header ----#
    motion_model_estimated_state.header.stamp = rospy.Time.now()
    motion_model_estimated_state.header.frame_id='odom'

    rospy.loginfo("\nInitialization state: {}".format(motion_model_estimated_state))  

def update_state_motion_model(waypoint):
    """Update the robot state by setting the motion_model_estimated_state global variable

    Parameters
    ----------
        tf_listener: tf_listener
            Transform listener
    Returns
    -------
    
    """
    # The estimate pose is equal to the previous waypoint
    # where I exepect to be, in case of noise-free environment
    pose_motion_model = convert_wp_to_pose(waypoint)
    rospy.logdebug("Previous wp with odom: {}".format(pose_motion_model))    
    
    global motion_model_estimated_state
    #---- POSE----#
    motion_model_estimated_state.pose.pose = pose_motion_model
    # update the covariance matrix
    # we suppose that the error on the different axis is i.i.d (indipendent and equally distributed)
    for i in range(6):
        diagonal_index = (i*6)+i
        if i < 3:
            if i == 0:
                motion_model_estimated_state.pose.covariance[diagonal_index] = motion_model_estimated_state.pose.covariance[diagonal_index] + STD_DEV**2
            elif i == 1:
                motion_model_estimated_state.pose.covariance[diagonal_index] = motion_model_estimated_state.pose.covariance[diagonal_index] + STD_DEV**2
            elif i == 2:
                motion_model_estimated_state.pose.covariance[diagonal_index] = motion_model_estimated_state.pose.covariance[diagonal_index] + 1000000000000.0

        else:
            if i == 5:
                motion_model_estimated_state.pose.covariance[diagonal_index] = motion_model_estimated_state.pose.covariance[diagonal_index] + STD_DEV_ROT**2
            else:
                motion_model_estimated_state.pose.covariance[diagonal_index] = motion_model_estimated_state.pose.covariance[diagonal_index] + 1000000000000.0
   
    motion_model_estimated_state.header.stamp = rospy.Time.now()
    motion_model_estimated_state.header.frame_id='odom'
    rospy.loginfo("\nUpdate state based on motion model: {}".format(motion_model_estimated_state))    
 
def main():
    rospy.init_node("exe_2_node")
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
        if i > 0:
            # Update the robot state
            # Note: The estimated position through the motion model is the previous waypoint
            update_state_motion_model(waypoint=waypoints[i-1])
            current_pose = motion_model_estimated_state.pose.pose
            # convert current_pose_odom into current_pose_base_footprint_odom
            # create homogeneous transformation matrix from odom
            A_odom = np.zeros((4,4))
            A_odom[:3, :3] = tf.transformations.quaternion_matrix([current_pose.orientation.x, current_pose.orientation.y,
                                                                            current_pose.orientation.z, current_pose.orientation.w])[:3, :3]
            A_odom[0][3] = current_pose.position.x
            A_odom[1][3] = current_pose.position.y
            A_odom[2][3] = current_pose.position.z
            A_odom[3][3] = 1
            
            A_base_footprint_odom = np.zeros((4,4))
            A_base_footprint_odom[:3,:3] = A_odom[:3,:3].transpose()
            A_base_footprint_odom[3][3] = 1

            quaternion_base_footprint_to_odom = tf.transformations.quaternion_from_matrix(matrix = A_base_footprint_odom)
            current_pose.orientation.x = quaternion_base_footprint_to_odom[0]
            current_pose.orientation.y = quaternion_base_footprint_to_odom[1]
            current_pose.orientation.z = quaternion_base_footprint_to_odom[2]
            current_pose.orientation.w = quaternion_base_footprint_to_odom[3]

            position_base_footprint_to_odom = -np.matmul(A_odom[:3,:3].transpose(), 
                                                        np.array([current_pose.position.x, current_pose.position.y,current_pose.position.z]).transpose())
            current_pose.position.x = position_base_footprint_to_odom[0]
            current_pose.position.y = position_base_footprint_to_odom[1]
            current_pose.position.z = position_base_footprint_to_odom[2]
            rospy.logdebug("Current pose base_footprint to odom: {}".format(current_pose))
        
        elif i == 0:
            # at the beginning, initialize the state
            initialize_robot_state(tf_listener=tf_listener)
            current_pose = motion_model_estimated_state.pose.pose

        # get desired robot pose
        desired_pose = convert_wp_to_pose(waypoints[i])
        # compute the difference between the current pose and the desired one
        delta_x, delta_y, delta_theta = compute_pose_difference(current_pose=current_pose, desired_pose=desired_pose)
        # compute the time needed to reach the desired pose with a given velocity
        rospy.loginfo("\nMove toward the waypoint....")
        move_robot(cmd_vel_pub, delta_x, delta_y, delta_theta)

        while not REACHED_WP:
            pass
        rospy.loginfo("\nWhere the robot is: \n{}".format(get_current_pose(tf_listener, 'odom', 'base_footprint')))
        rospy.loginfo("\nWhere the robot is supposed to be, based on motion model: \n{}".format(desired_pose))
        
if __name__ == '__main__':
    main()