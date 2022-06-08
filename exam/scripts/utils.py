from typing import List, Tuple
import rospy
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Pose, Twist, PoseStamped, PoseWithCovarianceStamped, PoseWithCovariance
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseGoal
from actionlib_msgs.msg import GoalID
import tf
import math
import yaml
import numpy as np
import csv

SEQ = 0

##---- CONSTANT DEFINITION ----#
# MAX LINEAR SPEED
V_MAX = 0.20
# MAX ANGULAR SPEED
OMEGA_MAX = 0.10
# TIME TO REACH DESIDER POSITION
TIME = 5 # s
# Flag to consider the user input valid
USER_INPUT_VALID = True
REACHED_WP = True
ALIGNMENT_COMPLETE = False

# set the mean and std.var for rotation in motion model
MEAN_ROT = 0.0 # rad
STD_DEV_ROT = 0.006 # rad

# set the mean and std. var for laser scan estimation
MEAN_LASER_X = 0.0
STD_DEV_LASER_VAR_X = 0.3
MEAN_LASER_Y = 0.0
STD_DEV_LASER_VAR_Y = 0.3
MEAN_ORIENTATION_IMU = 0.0
STD_DEV_ORIENTATION_IMU = 0.01


def read_csv_file(path_file) -> Tuple[list, PoseWithCovarianceStamped]:
    waypoints = []

    def get_initial_pose_csv(wp:list) -> PoseWithCovarianceStamped:
        pose = [float(wp[0]), float(wp[1]), float(wp[2])]
        pose_stamped_with_covariance = PoseWithCovarianceStamped()
        #---- Header ----#
        pose_stamped_with_covariance.header.seq = 0
        rospy.loginfo(rospy.Time.now())
        pose_stamped_with_covariance.header.stamp = rospy.Time.now()
        pose_stamped_with_covariance.header.frame_id = "map"
        #---- Pose With Covariance ----#
        pose_with_covariance = PoseWithCovariance()
        pose_with_covariance.pose.position.x = pose[0]
        pose_with_covariance.pose.position.y = pose[1]
        pose_with_covariance.pose.position.z = 0.0
        orientation = tf.transformations.quaternion_from_euler(0.0, 0.0, pose[2])
        pose_with_covariance.pose.orientation.x = float(orientation[0])
        pose_with_covariance.pose.orientation.y = float(orientation[1])
        pose_with_covariance.pose.orientation.z = float(orientation[2])
        pose_with_covariance.pose.orientation.w = float(orientation[3])
        pose_with_covariance.covariance = list(np.zeros(36, np.float))
        for i in range(6):
            diagonal_index = (i*6)+i
            if i < 3:
                if i == 2:
                    pose_with_covariance.covariance[diagonal_index] = 1000000000000.0 # since the robot is planar and the z term is not estimated
                else:
                    pose_with_covariance.covariance[diagonal_index] = 10**-5 # the initial pose is supposed known 
            else:
                if i == 3 or i == 4:
                    pose_with_covariance.covariance[diagonal_index] = 1000000000000.0 # since the robot is planar and the roll and pitch are not estimated
                else:
                    pose_with_covariance.covariance[diagonal_index] = 0.001 # the initial orientation is supposed known        
        
        pose_stamped_with_covariance.pose = pose_with_covariance
        return pose_stamped_with_covariance

    def compute_wp_orientation(current_wp:list, next_wp:list) -> float:
        # compute the difference vector between the current and next wp
        diff_vector = np.subtract(np.array(next_wp), np.array(current_wp))
        # compute the orientation between the current wp and the next one
        return np.arctan2(diff_vector[1], diff_vector[0])
    
    previous_wp = []
    with open(path_file) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=' ')
        for i, row in enumerate(csv_reader):
            # start position
            if i == 0:
                waypoints.append([float(row[0]), float(row[1]), 0.0])
                previous_wp = [float(row[0]), float(row[1])]
            # goal position
            elif i == 1:
                goal_wp = [float(row[0]), float(row[1]), 0.0]
            else:
                # compute the orientation for the previous wp based on the current wp
                wp_angle = compute_wp_orientation(previous_wp, [float(row[0]), float(row[1])])
                waypoints[-1][2] = wp_angle
                waypoints.append([float(row[0]), float(row[1]), 0.0])
                previous_wp = [float(row[0]), float(row[1])]
        
        # compute the orientation for the last wp before the goal
        wp_angle = compute_wp_orientation(previous_wp, [goal_wp[0],goal_wp[1]])
        waypoints[-1][2] = wp_angle
        waypoints.append(goal_wp)
        waypoints[-1][2] = wp_angle
        rospy.loginfo(waypoints)
        # get the initial pose
        initial_pose = get_initial_pose_csv(waypoints[0])
    return waypoints, initial_pose
                
def read_configuration_file(path_file):
    
    def get_wps(file_reader):
        # create a sequence of waypoits [x,y,theta], with respect to map
        waypoints = []
        for wp in file_reader['waypoints']:
            waypoints.append(wp)

        return waypoints

    def get_initial_pose(file_reader):
        pose = file_reader['initial_pose']
        pose_stamped_with_covariance = PoseWithCovarianceStamped()
        #---- Header ----#
        pose_stamped_with_covariance.header.seq = 0
        rospy.loginfo(rospy.Time.now())
        pose_stamped_with_covariance.header.stamp = rospy.Time.now()
        pose_stamped_with_covariance.header.frame_id = "map"
        #---- Pose With Covariance ----#
        pose_with_covariance = PoseWithCovariance()
        pose_with_covariance.pose.position.x = pose[0]
        pose_with_covariance.pose.position.y = pose[1]
        pose_with_covariance.pose.position.z = 0.0
        orientation = tf.transformations.quaternion_from_euler(0.0, 0.0, pose[2])
        pose_with_covariance.pose.orientation.x = float(orientation[0])
        pose_with_covariance.pose.orientation.y = float(orientation[1])
        pose_with_covariance.pose.orientation.z = float(orientation[2])
        pose_with_covariance.pose.orientation.w = float(orientation[3])
        pose_with_covariance.covariance = list(np.zeros(36))        
        pose_stamped_with_covariance.pose = pose_with_covariance
        return pose_stamped_with_covariance

    with open(path_file, 'r') as f:
        file_reader = yaml.load(f, Loader=yaml.FullLoader)


    waypoints = get_wps(file_reader)
    initial_pose = get_initial_pose(file_reader=file_reader)
    return waypoints, initial_pose
    
def create_markers(waypoints):
    marker_array = MarkerArray()
    marker_array.markers = []

    for i in range(len(waypoints)):
        # marker header 
        marker = Marker()
        marker.header.frame_id = "map"
        
        # marker field
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        #---- define marker pose ----#
        # position 
        marker.pose.position.x = waypoints[i][0]
        marker.pose.position.y = waypoints[i][1]
        marker.pose.position.z = 0
        # pose
        marker.pose.orientation.w = 1

        marker.color.r, marker.color.g, marker.color.b = (0, 255, 0)
        marker.color.a = 0.5
        marker.scale.x, marker.scale.y, marker.scale.z = (0.06, 0.06, 0.06)
        marker_array.markers.append(marker)

    # set markers id
    id = 0
    for m in marker_array.markers:
        m.id = id
        id += 1
    return marker_array

def create_goal_msg(wp)->MoveBaseActionGoal:
    stamp = rospy.Time.now()
    #---- GOAL ----#
    move_base_action_goal = MoveBaseActionGoal()
    # header
    global SEQ
    move_base_action_goal.header.seq = SEQ
    SEQ += 1 
    move_base_action_goal.header.stamp = stamp
    move_base_action_goal.header.frame_id = 1
    # goal id
    move_base_action_goal.goal_id.stamp = stamp
    move_base_action_goal.goal_id.id = "wp_goal"
    # move base goal
    move_base_goal = MoveBaseGoal()
    #---- Pose Stamped ----#
    pose_stamped = PoseStamped()
    #---- Header ----#
    pose_stamped.header.stamp = stamp
    pose_stamped.header.frame_id = "map"
    #---- Pose ----#
    pose = Pose()
    pose.position.x = wp[0]
    pose.position.y = wp[1]
    pose.position.z = 0.0
    orientation = tf.transformations.quaternion_from_euler(0.0, 0.0, wp[2])
    pose.orientation.x = float(orientation[0])
    pose.orientation.y = float(orientation[1])
    pose.orientation.z = float(orientation[2])
    pose.orientation.w = float(orientation[3])
    pose_stamped.pose = pose
    move_base_goal.target_pose = pose_stamped
    move_base_action_goal.goal = move_base_goal
    
    return move_base_action_goal

def convert_polar_to_cartesian(ray: float, angle: float):
    
    x = ray * math.cos(angle)
    y = ray * math.sin(angle)
    return [x,y]

def trapezoidal_motion(cmd_vel_pub: rospy.Publisher,delta):
    t_f = 10.0
    vel_c = 2 * (delta) / t_f
    pos_i = 0
    pos_f = delta

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
    