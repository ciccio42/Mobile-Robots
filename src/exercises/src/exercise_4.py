#! /usr/bin/python3

# ROS import
from cmath import nan
from ossaudiodev import SNDCTL_SEQ_GETINCOUNT
from typing import List
import rospy, rospkg
import tf
from tf import TransformListener
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PointStamped, Pose, Twist, PoseWithCovariance, TwistWithCovariance, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry
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
STD_VAR_ROT = 0.006 # rad

# set the mean and std. var for laser scan estimation
MEAN_X = 0.0
STD_VAR_X = 0.3
MEAN_Y = 0.0
STD_VAR_Y = 0.05
MEAN_ORIENTATION_IMU = 0.0
STD_VAR_ORIENTATION_IMU = 0.01

# Initial Pose
initial_pose = Pose()
# Distance values at beginning
initial_measure_cartesian_odom = None
initial_measure_polar = None
# counter for estimated pose with sensor
seq_cnt = 0

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

def trapezoidal_motion(cmd_vel_pub: rospy.Publisher,delta_x):
    vel_c = 0.2
    t_f = 5.0
    pos_i = 0
    pos_f = delta_x

    t_c = (pos_i-pos_f + (vel_c * t_f))/(vel_c)
    acc_c = (vel_c**2 ) /(pos_i-pos_f + (vel_c * t_f))
    t_c = rospy.Duration(t_c).to_sec()
    t_f = rospy.Duration(t_f).to_sec()
    
    rospy.logdebug(f"t_c:{t_c} - acc_c: {acc_c}")

    
    start = rospy.Time.now().to_sec()
    v_k_prec = 0 
    v_k = 0
    t_prec = rospy.Time.now().to_sec()
    
    r = rospy.Rate(30)
    while (rospy.Time.now().to_sec() - start) < t_f:

        if 0 < (rospy.Time.now().to_sec() - start) < (t_c):
            v_k = v_k_prec + (acc_c * (rospy.Time.now().to_sec() - t_prec))
            rospy.logdebug(f"acc con v: {v_k} -- {rospy.Time.now().to_sec() - t_prec}")
            t_prec = rospy.Time.now().to_sec()
            v_k_prec = v_k
        elif (t_c) < (rospy.Time.now().to_sec() - start) < (t_f - t_c):
            v_k = v_k_prec
            t_prec = rospy.Time.now().to_sec()
            rospy.logdebug(f"cost con v: {v_k}")

        elif ((t_f - t_c) < (rospy.Time.now().to_sec() - start) < (t_f)):
            v_k = v_k_prec - (acc_c * (rospy.Time.now().to_sec()- t_prec))
            t_prec = rospy.Time.now().to_sec()
            rospy.logdebug(f"dec con v: {v_k}")
            v_k_prec = v_k
    
        cmd_vel = Twist()
        cmd_vel.linear.x = abs(v_k)
        cmd_vel.linear.y = 0.0
        cmd_vel.linear.z = 0.0
        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = 0.0
        cmd_vel_pub.publish(cmd_vel)
        
        r.sleep()
    timer_elapsed()

def move_robot(cmd_vel_pub: rospy.Publisher, delta_x, delta_y, delta_theta, angle_x_base_footprint_displacement):
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
        rospy.loginfo("Going straight with trapezoidal motion.....")
        # where actually I go due to the noise in the movement model
        new_displacement_w_noise = [delta_x, delta_y] + np.random.normal(MEAN, STD_VAR, size=2)
        rospy.loginfo("Command afer noise: delta_x {} - delta_y {} - delta_theta {}".format(new_displacement_w_noise[0], new_displacement_w_noise[1], 0.0))
        v_x = new_displacement_w_noise[0] / TIME
        v_y = new_displacement_w_noise[1] / TIME
        trapezoidal_motion(cmd_vel_pub, new_displacement_w_noise[0])        
    else:
        #----STEP 1----#
        # align with the displacement vector
        rospy.loginfo("Aligning with displacement vector..")
        angle_x_base_footprint_displacement = angle_x_base_footprint_displacement + np.random.normal(MEAN_ROT, STD_VAR_ROT)
        rospy.loginfo("Command rotation angle_x_base_footprint_displacement after noise:  {}".format(angle_x_base_footprint_displacement))
        # the robot is not aligned with the target orientation
        rospy.loginfo("Aligning with next wp...")
        omega = angle_x_base_footprint_displacement / TIME
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

        #---STEP 2---#
        # move toward the wp
        # Rotate delta
        tRz = tf.transformations.rotation_matrix(angle_x_base_footprint_displacement, (0,0,1))[:-1,:-1].transpose()
        # new_displacement_wo_noise -> my belief where I think I am going
        new_displacement_wo_noise = np.matmul(tRz, np.array([[delta_x, delta_y, 0.0]]).transpose())
        rospy.loginfo("Command after alignemnt: delta_x {} - delta_y {} - delta_theta {}".format(new_displacement_wo_noise[0][0], new_displacement_wo_noise[1][0], 0.0))
        # set new command
        rospy.sleep(1)
        # where actually I go due to the noise in the movement model
        noise = np.random.normal(MEAN, STD_VAR, size=2)
        rospy.loginfo("Noise: {}".format(noise))
        new_displacement_w_noise = [new_displacement_wo_noise[0][0], new_displacement_wo_noise[1][0]] + noise
        rospy.loginfo("Command after alignemnt, with noise: delta_x {} - delta_y {} - delta_theta {}".format(new_displacement_w_noise[0], new_displacement_w_noise[1], 0.0))
        v_x = new_displacement_w_noise[0] / TIME
        v_y = new_displacement_w_noise[1] / TIME
        trapezoidal_motion(cmd_vel_pub, new_displacement_w_noise[0])
        while not REACHED_WP:
            pass

        #---STEP 3---#
        # reach wp orientation
        theta_with_noise = (delta_theta-angle_x_base_footprint_displacement) + np.random.normal(MEAN_ROT, STD_VAR_ROT)
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
        ALIGNMENT_COMPLETE = False
        while not ALIGNMENT_COMPLETE:
            pass
           
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

    
    rospy.logdebug("A_base_footprint_to_odom:\n{}".format(A_base_footprint_to_odom))
    rospy.logdebug("A_odom_to_wp:\n{}".format(A_odom_to_wp))
    rospy.logdebug("vA_base_footprint_to_wp:\n{}".format(A_base_footprint_to_wp))
    

    delta_x = A_base_footprint_to_wp[0][3]
    delta_y = A_base_footprint_to_wp[1][3]

    r,p,y = tf.transformations.euler_from_matrix(A_base_footprint_to_wp[:3,:3])
    delta_theta = y

    # compute the yaw that align the robot with the displacement vector
    angle_x_base_footprint_displacement = math.atan2(delta_y, delta_x)
    rospy.loginfo("Angle angle_x_base_footprint_displacement {}".format(angle_x_base_footprint_displacement))

    rospy.loginfo("Command delta_x {} - delta_y {} - delta_theta {}".format(delta_x, delta_y, delta_theta))

    return delta_x, delta_y, delta_theta, angle_x_base_footprint_displacement

def get_laser_scan(laser_scan_topic: str):
    """Get the measure from laser scan
    
    Parameters
    ----------
        laser_scan_topic: str
            Laser scan topic name
    Returns
    -------
        laser_scan_msg: sensor_msgs.LaserScan
            Laser scan message
    """    
    laser_scan_msg = rospy.wait_for_message(laser_scan_topic, LaserScan)

    return laser_scan_msg

def convert_laser_measure_polar_to_cartesian(measures: np.array):
    """Convert laser ranges defined in polar coordinates into the corresponding cartesian coordinates
    
    Parameters
    ----------
        measures: np.array
            array of range measures
    Returns
    -------
        measure_cartesian: list
            list of tuples (x,y)
    """     
    measure_cartesian = []

    for angle, ray in enumerate(measures):
        measure_cartesian.append(utils.convert_polar_to_cartesian(ray=ray, angle=((angle*2*math.pi)/(360))))

    measure_cartesian = np.array(measure_cartesian)

    return measure_cartesian

def covert_laser_scan_to_frame(tf_listener: tf.listener , measure_base_scan: np.array, frame: str):
    """Convert measures defined in /scan frame, into the target frame frame
    
    Parameters
    ----------
        tf_listener: tf_listener
            Transform listener
        measure_base_scan: np.array
            Measures defined in /scan frame
        frame:
            Target frame, with respect to which define the measures
    Returns
    -------
        measaure_cartesian_target: list
            list of tuples (x,y) with respect to target frame
    """ 
    t = tf_listener.getLatestCommonTime(frame, "base_scan")
    position, quaternion = tf_listener.lookupTransform(frame, "base_scan", t)

    A_target_base_scan = np.zeros((4,4))
    
    A_target_base_scan[:3,:3] = tf.transformations.quaternion_matrix(quaternion)[:3, :3]
    A_target_base_scan[0][3] = position[0]
    A_target_base_scan[1][3] = position[1]
    A_target_base_scan[2][3] = position[2]
    A_target_base_scan[3][3] = 1
    rospy.logdebug("{} to base scan {}".format(frame, A_target_base_scan))
    
    measaure_cartesian_target = []
    for measure in  measure_base_scan:
        pos_base_scan = np.array([[measure[0], measure[1], 0, 1]]).transpose()
        pos_target = np.matmul(A_target_base_scan, pos_base_scan)
        pos_target = [pos_target[0][0], pos_target[1][0], pos_target[2][0]]
        measaure_cartesian_target.append(pos_target)

    measaure_cartesian_target = np.array(measaure_cartesian_target)
    return measaure_cartesian_target

def measure_world(laser_scan_topic: str, imu_topic: str, odom_topic: str, tf_listener: tf.listener):
    """Measure the robot pose through sensors

    Parameters
    ----------
        laser_scan_topic: str
            name of laser scan topic, used to get the position of the robot

        imu_topic: str
            name of the imu topic, used to get the orientation of the robot

        tf_listener: tf_listener
            Transform listener
    Returns
    -------
    
        robot_pose: Pose
            Estimated robot pose

    """

    def estimate_robot_pose_laser_scan()->Pose:
        #---- ESTIMATE CURRENT POSITION WITH LASER SCAN ----#
        robot_pose = Pose()
        # get the orientation from the imu msg
        # the goal is to obtain the orientation of odom with respect to base_footprint
        if imu_msg.header.frame_id == "base_footprint" and imu_msg.orientation_covariance[0] != -1:
            robot_pose.orientation.x = imu_msg.orientation.x  + np.random.normal(MEAN_ORIENTATION_IMU, STD_VAR_ORIENTATION_IMU)
            robot_pose.orientation.y = imu_msg.orientation.y  + np.random.normal(MEAN_ORIENTATION_IMU, STD_VAR_ORIENTATION_IMU)
            robot_pose.orientation.z = imu_msg.orientation.z  + np.random.normal(MEAN_ORIENTATION_IMU, STD_VAR_ORIENTATION_IMU)
            robot_pose.orientation.w = imu_msg.orientation.w  + np.random.normal(MEAN_ORIENTATION_IMU, STD_VAR_ORIENTATION_IMU)
        
        # convert polar to cartesian
        measures_cartesian = convert_laser_measure_polar_to_cartesian(laser_scan_msg.ranges)
        # convert base_scan to odom
        measures_cartesian_base_footprint = covert_laser_scan_to_frame(tf_listener=tf_listener, measure_base_scan=measures_cartesian, frame="base_footprint")
        rospy.logdebug("\nCurrent measure  \nfront: {} \nleft: {} \nbehind: {} \nright: {}".format(measures_cartesian_base_footprint[0],
                                                                                                measures_cartesian_base_footprint[90],
                                                                                                measures_cartesian_base_footprint[180],
                                                                                                measures_cartesian_base_footprint[270]))
        
        # get the position
        _,_, y = tf.transformations.euler_from_quaternion([imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w])
        
        if y < 0: # rotate in clockwise 
            # the value of interest is in counter-clockwise direction 
            yaw_index = int(abs((y*360)/(2*math.pi)))
        else:   # rotate in counter-clockwise
            # the value of interest is in clockwise direction
            yaw_index = 359 - int(abs((y*360)/(2*math.pi)))


        global initial_measure_cartesian_odom
        # get first measure
        measure_front_first_odom = initial_measure_cartesian_odom[0]
        measure_left_first_odom = initial_measure_cartesian_odom[90]
        # get current measure aligned with odom
        measure_front_current = measures_cartesian_base_footprint[yaw_index]
        measure_left_current = measures_cartesian_base_footprint[(90+yaw_index)%359]
        # align measures with odom
        rot_matrix = tf.transformations.quaternion_matrix([imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w])[:3,:3]
        measure_front_current_odom = np.matmul(rot_matrix, np.array([measure_front_current]).transpose())
        measure_front_current_odom = [measure_front_current_odom[0][0], measure_front_current_odom[1][0], measure_front_current_odom[2][0]]
        measure_left_current_odom = np.matmul(rot_matrix, np.array([measure_left_current]).transpose())
        measure_left_current_odom = [measure_left_current_odom[0][0], measure_left_current_odom[1][0], measure_left_current_odom[2][0]]

        if (measure_front_current_odom[0] == np.inf or measure_front_current_odom[0] == nan) or \
            (measure_left_current_odom[0] == np.inf or measure_left_current_odom[0] == nan):
            pass
        else:
            # compute the difference
            rospy.logdebug("\nFirst measure front: {}\nFirst measure left: {}\n".format(measure_front_first_odom, measure_left_first_odom))
            rospy.logdebug("\nCurrent measure front: {}\nCurrent measure left: {}\n".format(measure_front_current_odom, measure_left_current_odom))
            x_pos_odom = (measure_front_first_odom - measure_front_current_odom)[0] + np.random.uniform(MEAN_X, STD_VAR_X)
            y_pos_odom = (measure_left_first_odom - measure_left_current_odom)[1] + np.random.uniform(MEAN_X, STD_VAR_X)
            pos_odom = np.array([x_pos_odom, y_pos_odom, 0.0])
            rospy.logdebug("\nPosition odom: {}".format(pos_odom))
            
            """
            # compute the position from base_footprint to odom
            rot_matrix = tf.transformations.quaternion_matrix([imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w])[:3,:3]
            pos_base_footprint_to_odom = np.matmul(rot_matrix.transpose(), np.array([pos_odom]).transpose())
            pos_base_footprint_to_odom = np.array([pos_base_footprint_to_odom[0][0],pos_base_footprint_to_odom[1][0],pos_base_footprint_to_odom[2][0]])       
            rospy.logdebug("\nPosition base_footprint: {}".format(pos_base_footprint_to_odom))
            """

        
        robot_pose.position.x = pos_odom[0]
        robot_pose.position.y = pos_odom[1]
        robot_pose.position.z = pos_odom[2]

        return robot_pose

    laser_scan_msg = rospy.wait_for_message(laser_scan_topic, LaserScan)
    imu_msg = rospy.wait_for_message(imu_topic, Imu) 
    odom_msg = rospy.wait_for_message(odom_topic, Odometry)
    
    # get estimated robot pose with laser scan
    pose_laser_scan = estimate_robot_pose_laser_scan() # odom -> base_footprint
    rospy.loginfo("Estimated robot pose with laser scan: {}".format(pose_laser_scan))
    robot_pose_with_covariance = PoseWithCovariance()
    # set the position of robot_pose_with_covariance
    robot_pose_with_covariance.pose.position.x = pose_laser_scan.position.x
    robot_pose_with_covariance.pose.position.y = pose_laser_scan.position.y
    robot_pose_with_covariance.pose.position.z = pose_laser_scan.position.z
    # the orientation between odom and estimated with laser scan is the same, 
    # since the robot spawn in (0,0,0) with the same orientation of odom
    # we keep the same orientation of pose_laser_scan
    robot_pose_with_covariance.pose.orientation = pose_laser_scan.orientation
    robot_pose_with_covariance.covariance = list(np.zeros(36))
    for i in range(6):
        diagonal_index = (i*6)+i
        if i < 3:
            robot_pose_with_covariance.covariance[diagonal_index] = robot_pose_with_covariance.covariance[diagonal_index] + (STD_VAR_X**2)
        else:
            robot_pose_with_covariance.covariance[diagonal_index] = robot_pose_with_covariance.covariance[diagonal_index] + (STD_VAR_ORIENTATION_IMU**2)

    rospy.loginfo("Combined pose with covariance: {}".format(robot_pose_with_covariance))

    return robot_pose_with_covariance, odom_msg.twist

def update_with_ekf(robot_pose_with_covariance: PoseWithCovariance, robot_twist_with_covariance: TwistWithCovariance, odom_combined_pub: rospy.Publisher):
    # create odometry message
    odom_combined_msg = Odometry()
    # Header
    global seq_cnt
    odom_combined_msg.header.seq = seq_cnt
    seq_cnt += 1
    odom_combined_msg.header.stamp = rospy.Time.now()
    odom_combined_msg.header.frame_id = "odom"
    # child_frame_id
    odom_combined_msg.child_frame_id = "base_footprint"
    # pose
    odom_combined_msg.pose = robot_pose_with_covariance
    # twist
    odom_combined_msg.twist = robot_twist_with_covariance
    # PUBLISH
    odom_combined_pub.publish(odom_combined_msg)

    ekf_pose = rospy.loginfo("Waiting for ekf filter...")
    ekf_pose = rospy.wait_for_message("/robot_pose_ekf/odom_combined", PoseWithCovarianceStamped)
    rospy.loginfo("EKF Pose {}".format(ekf_pose))
    
    # convert from odom_combined to base footprint
    estimated_robot_pose = Pose()

    R_base_footprint_odom_combined = np.zeros((4,4))
    R_base_footprint_odom_combined[3][3] = 1
    R_base_footprint_odom_combined[:3, :3] = np.array(tf.transformations.quaternion_matrix([ekf_pose.pose.pose.orientation.x, ekf_pose.pose.pose.orientation.y,
                                                                                            ekf_pose.pose.pose.orientation.z, ekf_pose.pose.pose.orientation.w]))[:3, :3]
    R_base_footprint_odom_combined[:3,:3] = R_base_footprint_odom_combined[:3,:3].transpose()
    
    # position
    p_base_footprint_odom_combined = -np.matmul(R_base_footprint_odom_combined[:3, :3], 
                                             np.array([ekf_pose.pose.pose.position.x, ekf_pose.pose.pose.position.y, ekf_pose.pose.pose.position.z]).transpose())
    rospy.loginfo("p: {}".format(p_base_footprint_odom_combined))

    estimated_robot_pose.position.x = p_base_footprint_odom_combined[0]
    estimated_robot_pose.position.y = p_base_footprint_odom_combined[1]
    estimated_robot_pose.position.z = p_base_footprint_odom_combined[2]
    # orientataion
    quaternion_base_footprint_odom_combined = tf.transformations.quaternion_from_matrix(R_base_footprint_odom_combined)
    estimated_robot_pose.orientation.x = quaternion_base_footprint_odom_combined[0]
    estimated_robot_pose.orientation.y = quaternion_base_footprint_odom_combined[1]
    estimated_robot_pose.orientation.z = quaternion_base_footprint_odom_combined[2]
    estimated_robot_pose.orientation.w = quaternion_base_footprint_odom_combined[3]

    return estimated_robot_pose

def main():
    rospy.init_node("exe_4_node")
    rate = rospy.Rate(1)
    
    # marker publisher
    marker_array_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)

    # create transform listener
    tf_listener = TransformListener()

    tf_listener.waitForTransform("/odom", "/imu_link", rospy.Time(), rospy.Duration(20.0))
    
    # combined odom publisher 
    odom_combined_pub = rospy.Publisher('turtlebot/laser_scan_pose', Odometry, queue_size=10)

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
    global initial_measure_polar
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
        robot_pose_with_covariance, robot_twist_with_covariance = measure_world("/scan", "/imu", "/odom",tf_listener)
        robot_pose = update_with_ekf(robot_pose_with_covariance, robot_twist_with_covariance, odom_combined_pub)
        rospy.loginfo("Estimated robot pose {}".format(robot_pose))
        key = input("Press any key to continue: ")
        rospy.loginfo("Waypoint {} - {}".format(i+1, waypoints[i]))
        
        # get desired robot pose
        desired_pose = convert_wp_to_pose(waypoints[i])
        # compute the difference between the current pose and the desired one
        delta_x, delta_y, delta_theta,  angle_x_base_footprint_displacement = compute_pose_difference(current_pose=robot_pose, desired_pose=desired_pose)
        
        # compute the time needed to reach the desired pose with a given velocity
        rospy.loginfo("Move toward the waypoint....")
        move_robot(cmd_vel_pub, delta_x, delta_y, delta_theta, angle_x_base_footprint_displacement)

        while not REACHED_WP:
            pass
    
if __name__ == '__main__':
    main()