#! /usr/bin/python3

# ROS import
from cmath import nan
import rospy, rospkg
import tf
from tf import TransformListener
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PointStamped, Pose, Twist
from sensor_msgs.msg import Imu, LaserScan

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

# set the mean and std. var for guassian noise
MEAN = 0.0 # m
STD_VAR = 0.3 # m 

# set the mean and std.var for rotation
MEAN_ROT = 0.0 # rad
STD_VAR_ROT = 0.06 # rad

# Initial Pose
initial_pose = Pose()
# Distance values at beginning
initial_measure_cartesian_odom = None
# Angles of interest
angles_of_interest = [0, 90, 180, 270] # front, left, behind, right


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
    global REACHED_WP
    REACHED_WP = False
    
    if abs(delta_theta) < 10**-3: # delta_theta is equal to zero
        # the robot is alligned with the desired orientation
        # go straight
        rospy.loginfo("Going straight.....")
        # where actually I go due to the noise in the movement model
        new_displacement_w_noise = [delta_x, delta_y] + np.random.normal(MEAN, STD_VAR, size=2)
        rospy.loginfo("Command afer noise: delta_x {} - delta_y {} - delta_theta {}".format(new_displacement_w_noise[0], new_displacement_w_noise[1], 0.0))
        v_x = new_displacement_w_noise[0] / TIME
        v_y = new_displacement_w_noise[1] / TIME
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
        # add noise to ration
        theta_with_noise = delta_theta + np.random.normal(MEAN_ROT, STD_VAR_ROT)
        rospy.loginfo("Command rotation after noise: delta_theta {}".format(theta_with_noise))
        omega = theta_with_noise / TIME
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
        # new_displacement_wo_noise -> my belief where I think I am going
        new_displacement_wo_noise = np.matmul(tRz, np.array([[delta_x, delta_y, 0.0]]).transpose())
        rospy.loginfo("Command after alignemnt: delta_x {} - delta_y {} - delta_theta {}".format(new_displacement_wo_noise[0][0], new_displacement_wo_noise[1][0], 0.0))
        # set new command
        rospy.sleep(1)
        # where actually I go due to the noise in the movement model
        noise = np.random.normal(MEAN, STD_VAR, size=2)
        rospy.loginfo("Noise: {}".format(noise))
        new_displacement_w_noise = [new_displacement_wo_noise[0][0], new_displacement_wo_noise[1][0]] + noise
        rospy.loginfo("Command after alignemnt noise: delta_x {} - delta_y {} - delta_theta {}".format(new_displacement_w_noise[0], new_displacement_w_noise[1], 0.0))
        v_x = new_displacement_w_noise[0] / TIME
        v_y = new_displacement_w_noise[1] / TIME
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
    rospy.loginfo("A_base_footprint_to_odom:\n{}".format(A_base_footprint_to_odom))
    rospy.loginfo("A_odom_to_wp:\n{}".format(A_odom_to_wp))
    rospy.loginfo("vA_base_footprint_to_wp:\n{}".format(A_base_footprint_to_wp))
    """

    delta_x = A_base_footprint_to_wp[0][3]
    delta_y = A_base_footprint_to_wp[1][3]

    r,p,y = tf.transformations.euler_from_matrix(A_base_footprint_to_wp[:3,:3])
    delta_theta = y

    rospy.loginfo("Command delta_x {} - delta_y {} - delta_theta {}".format(delta_x, delta_y, delta_theta))

    return delta_x, delta_y, delta_theta

def get_laser_scan(laser_scan_topic: str):
    
    laser_scan_msg = rospy.wait_for_message(laser_scan_topic, LaserScan)

    return laser_scan_msg

def convert_laser_measure_polar_to_cartesian(measures: np.array):
    
    measure_cartesian = []

    for angle, ray in enumerate(measures):
        measure_cartesian.append(utils.convert_polar_to_cartesian(ray=ray, angle=((angle*2*math.pi)/(360))))

    measure_cartesian = np.array(measure_cartesian)

    return measure_cartesian

def covert_laser_scan_to_frame(tf_listener: tf.listener , measure_base_scan: np.array, frame: str):
    t = tf_listener.getLatestCommonTime(frame, "base_scan")
    position, quaternion = tf_listener.lookupTransform(frame, "base_scan", t)

    A_odom_base_scan = np.zeros((4,4))
    
    A_odom_base_scan[:3,:3] = tf.transformations.quaternion_matrix(quaternion)[:3, :3]
    A_odom_base_scan[0][3] = position[0]
    A_odom_base_scan[1][3] = position[1]
    A_odom_base_scan[2][3] = position[2]
    A_odom_base_scan[3][3] = 1
    rospy.logdebug("{} to base scan {}".format(frame, A_odom_base_scan))
    
    measaure_cartesian_odom = []
    for measure in  measure_base_scan:
        pos_base_scan = np.array([[measure[0], measure[1], 0, 1]]).transpose()
        pos_odom = np.matmul(A_odom_base_scan, pos_base_scan)
        pos_odom = [pos_odom[0][0], pos_odom[1][0], pos_odom[2][0]]
        measaure_cartesian_odom.append(pos_odom)

    measaure_cartesian_odom = np.array(measaure_cartesian_odom)
    return measaure_cartesian_odom

def measure_world(laser_scan_topic: str, imu_topic: str, tf_listener: tf.listener):
    """Gets and prints the spreadsheet's header columns

    Parameters
    ----------
        laser_scan_topic: str
            name of laser scan topic, to get the position of the robot

        imu_topic: str
            name of the imu topic, to get the orientation of the robot
    Returns
    -------
    
        robot_pose: Pose
            Estimated robot pose

    """
    robot_pose = Pose()

    laser_scan_msg = rospy.wait_for_message(laser_scan_topic, LaserScan)
    imu_msg = rospy.wait_for_message(imu_topic, Imu) 

    # get the orientation from the imu msg
    if imu_msg.header.frame_id == "base_footprint" and imu_msg.orientation_covariance[0] == -1:
        robot_pose.orientation.x = imu_msg.orientation.x  
        robot_pose.orientation.y = imu_msg.orientation.y
        robot_pose.orientation.z = imu_msg.orientation.z
        robot_pose.orientation.w = imu_msg.orientation.w

    # convert polar to cartesian
    measures_cartesian = convert_laser_measure_polar_to_cartesian(laser_scan_msg.ranges)
    # convert base_scan to odom
    measures_cartesian_base_footprint = covert_laser_scan_to_frame(tf_listener=tf_listener, measure_base_scan=measures_cartesian, frame="base_footprint")
    rospy.loginfo("\nCurrent measure  \nfront: {} \nleft: {} \nbehind: {} \nright: {}".format(measures_cartesian_base_footprint[0],
                                                                                            measures_cartesian_base_footprint[90],
                                                                                            measures_cartesian_base_footprint[180],
                                                                                            measures_cartesian_base_footprint[270]))

    front_x_base_footprint = measures_cartesian_base_footprint[0] # front is always aligned to x_base_footprint 
    left_y_base_footprint = measures_cartesian_base_footprint[90] # left is always aligned to y_base_footprint

    _,_, y = tf.transformations.euler_from_quaternion([imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w])
    if (y*360)/(2*math.pi) < 0:     
        yaw = int(359 - abs((y*360)/(2*math.pi)))
    else:
        yaw = int(abs((y*360)/(2*math.pi)))

    rospy.logdebug("\nYaw: {}".format(yaw))
    global initial_measure_cartesian_odom
    if initial_measure_cartesian_odom[yaw][0] == np.inf or initial_measure_cartesian_odom[yaw][0] == nan:
        pass
    else:
        # compute the position with respect odom
        rot_matrix = tf.transformations.quaternion_matrix([imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w])[:3,:3]
        rospy.logdebug("\nRotation {}".format(rot_matrix))
        front_x_odom = np.matmul(rot_matrix, np.array([front_x_base_footprint]).transpose())
        front_x_odom = np.array([front_x_odom[0][0],front_x_odom[1][0],front_x_odom[2][0]])
        pos_odom_to_base_footprint = initial_measure_cartesian_odom[yaw] - front_x_odom
        rospy.logdebug("\nInitial measure odom {}: Front x odom {}".format(initial_measure_cartesian_odom[yaw], front_x_odom))
        rospy.logdebug("\nPosition base footprint to odom {}".format(pos_odom_to_base_footprint))
        
        # compute the position base_footprint to odom
        pos_base_footprint_to_odom = np.matmul(rot_matrix.transpose(), np.array([pos_odom_to_base_footprint]).transpose())
        pos_base_footprint_to_odom = np.array([pos_base_footprint_to_odom[0][0],pos_base_footprint_to_odom[1][0],pos_base_footprint_to_odom[2][0]])
            
    robot_pose.position.x = pos_base_footprint_to_odom[0]
    robot_pose.position.y = pos_base_footprint_to_odom[1]
    robot_pose.position.z = pos_base_footprint_to_odom[2]
    
    return robot_pose

def main():
    rospy.init_node("exe_3_node")
    rate = rospy.Rate(1)
    
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
    
    # get the initial pose from base_footprint to odom
    global initial_pose
    initial_pose = get_current_pose(tf_listener=tf_listener, start_frame="base_footprint", end_frame="odom")
    # get initial measure
    initial_measure_polar = get_laser_scan("/scan")
    # convert from polar to cartesian
    rospy.logdebug("Initial Measure polar \nfront: {} \nleft: {} \nbehind: {} \nright: {}".format(initial_measure_polar.ranges[0],
                                                                                      initial_measure_polar.ranges[90],
                                                                                      initial_measure_polar.ranges[180],
                                                                                      initial_measure_polar.ranges[270]))
    initial_measure_cartesian = convert_laser_measure_polar_to_cartesian(initial_measure_polar.ranges) # defined with respect to laser scan
    rospy.logdebug("Initial Measure cartesian \nfront: {} \nleft: {} \nbehind: {} \nright: {}".format(initial_measure_cartesian[0],
                                                                                      initial_measure_cartesian[90],
                                                                                      initial_measure_cartesian[180],
                                                                                      initial_measure_cartesian[270]))
    # convert from laser scan to odom
    global initial_measure_cartesian_odom
    initial_measure_cartesian_odom = covert_laser_scan_to_frame(tf_listener, initial_measure_cartesian, "odom")
    rospy.logdebug("Initial Measure odom \nfront: {} \nleft: {} \nbehind: {} \nright: {}".format(initial_measure_cartesian_odom[0],
                                                                                      initial_measure_cartesian_odom[90],
                                                                                      initial_measure_cartesian_odom[180],
                                                                                      initial_measure_cartesian_odom[270]))
    
    for i in range(len(waypoints)):
        
        rospy.loginfo("Estimate robot state with sensor")
        measured_robot_pose = measure_world("/scan", "/imu", tf_listener)
                
        rate.sleep()

        key = input("Press any key to continue: ")

        rospy.loginfo("Estimate robot state with sensor")
        measured_robot_pose = measure_world("/scan", "/imu", tf_listener)
        
        rospy.loginfo("Estimated robt pose {}".format(measured_robot_pose))


        rospy.loginfo("Waypoint {} - {}".format(i+1, waypoints[i]))
        
        # get desired robot pose
        desired_pose = convert_wp_to_pose(waypoints[i])
        # compute the difference between the current pose and the desired one
        delta_x, delta_y, delta_theta = compute_pose_difference(current_pose=measured_robot_pose, desired_pose=desired_pose)
        rospy.loginfo("")
        # compute the time needed to reach the desired pose with a given velocity
        rospy.loginfo("Move toward the waypoint....")
        move_robot(cmd_vel_pub, delta_x, delta_y, delta_theta)

        while not REACHED_WP:
            pass
    
if __name__ == '__main__':
    main()