import math
import os, sys
from threading import currentThread
import rospy, rospkg
from visualization_msgs.msg import MarkerArray
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, PoseArray, PointStamped, Pose
from sensor_msgs.msg import Imu,LaserScan
from move_base_msgs.msg import MoveBaseAction
from gazebo_msgs.msg import ModelState

import numpy as np 
from datetime import datetime
import os
import tf

# get the path to the current package
rospack = rospkg.RosPack()
pkg_path = rospack.get_path('exam')
utils_lib_path = os.path.join(pkg_path, "scripts")
sys.path.append(utils_lib_path)
import utils
from utils import V_MAX, OMEGA_MAX, TIME, USER_INPUT_VALID, REACHED_WP, ALIGNMENT_COMPLETE, \
                  ALIGNMENT_COMPLETE, MEAN_ORIENTATION_IMU, STD_DEV_ORIENTATION_IMU
import argparse
 
parser = argparse.ArgumentParser(description="Navigation Node parameters",
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument("-num_path", "--path-file-number", default="1", help="Number of path file")
parser.add_argument("-run_aip", "--run-automatic-initialization-procedure", default="False", help="Whether run the automatic initialization procedure or not")
args, unknown = parser.parse_known_args()

# Mean initialization error
initialization_error_mean = 0.0
# STD DEV initialization error
initialization_error_std_dev_x = 15 # m
initialization_error_std_dev_y = 10 # m
initialization_error_std_dev_yow = math.pi/4 # rad

# Covariance threshold for initial localization
COVARIANCE_X_THRESHOLD = 1 # m
COVARIANCE_Y_THRESHOLD = 1 # m
COVARIANCE_YAW_THRESHOLD = 0.1 # rad
TIME_LIMIT_FOR_INITIAL_LOCALIZATION = (2*math.pi)*2 # The time required to complete one rotation around the z-axis

# waypoints file path
path_file_path = os.path.join(pkg_path, f"config/path_")
cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

ERROR_X = 0.10
ERROR_Y = 0.10

def go_to_next_wp(wp: list, move_base_client: actionlib.SimpleActionClient, time):
    goal = utils.create_goal_msg(wp)
    rospy.loginfo(f"Sending goal {goal}")
    move_base_client.send_goal(goal=goal.goal)
    rospy.loginfo(f"Waiting for result....")
    move_base_client.wait_for_result()
    result = move_base_client.get_state()
 
    if result == GoalStatus.SUCCEEDED:
        rospy.loginfo("Waypoint reached")
        curr_time = rospy.Time.now().to_sec()
        log_file.write(f" ---- Waypoint reached in: {round(curr_time - time, 3)} s")
        return curr_time
        #utils.compute_difference_current_pose_desired_pose(wp)

def initialize_pose(initial_pose: PoseWithCovarianceStamped) -> PoseWithCovarianceStamped:
    # at the beginning the robot is supposed to be in the starting wp
    # however the related variance is high, since there is a huge uncertainty at the beginning
    initial_pose.pose.covariance[0] = initialization_error_std_dev_x
    initial_pose.pose.covariance[7] = initialization_error_std_dev_y
    initial_pose.pose.covariance[35] = initialization_error_std_dev_yow
    return initial_pose

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

def automatic_initialization_procedure():
    
    """Perform the initial localization
    """
    rate = rospy.Rate(10)
    # Start by rotating
    rate = rospy.Rate(10)
    cmd_vel_msg = Twist()
    cmd_vel_msg.angular.z = 1.0 # rad/s
    start_time = rospy.Time.now()
    rospy.loginfo("Start localization rotation procedure")
    time_elapsed = False
    
    # count number of particles
    particle_cloud = rospy.wait_for_message("/particlecloud", PoseArray)
    rospy.loginfo(f"Number of particles {len(particle_cloud.poses)}")
    localization = False

    while True:
        rate.sleep()
        current_time = rospy.Time.now()
        elapsed_time = (current_time.secs + (current_time.nsecs * 10**-9)) - (start_time.secs + (start_time.nsecs * 10**-9))
        rospy.loginfo(f"Localization phase - Elapsed time {elapsed_time}")
        try:
            # get the last estimated pose
            estimated_pose = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped)
            # rospy.loginfo (f"Estimated pose: {estimated_pose}")
            covariance_x = estimated_pose.pose.covariance[0]
            covariance_y = estimated_pose.pose.covariance[7]
            covariance_yow = estimated_pose.pose.covariance[35]
            cmd_vel_pub.publish(cmd_vel_msg)
            cmd_vel_pub.publish(cmd_vel_msg)
            if covariance_x < COVARIANCE_X_THRESHOLD and covariance_y < COVARIANCE_Y_THRESHOLD  and elapsed_time >= TIME_LIMIT_FOR_INITIAL_LOCALIZATION:
                cmd_vel_msg.angular.z = 0
                cmd_vel_pub.publish(cmd_vel_msg)
                break
            
        
        except:
            rospy.loginfo("AMCL Pose not updated")

    estimated_pose = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped)
    rospy.loginfo (f"Estimated pose: {estimated_pose}")
    covariance_x = estimated_pose.pose.covariance[0]
    covariance_y = estimated_pose.pose.covariance[7]
    covariance_yow = estimated_pose.pose.covariance[35]
    if covariance_yow < COVARIANCE_YAW_THRESHOLD:
        localization = True
        return True
    else:
        localization = False
        rospy.loginfo("Localization not completed")



    if localization == False:
        input("Press any key to start to move into open space")
        while True:
            laser_values = utils.get_laser_scan("/scan").ranges

            # take the maximum value and the degree
            max_measure = max(laser_values)
            degree = laser_values.index(max_measure)
            rospy.loginfo (f"max: {max_measure}, degree: {degree}")
            input("press")
            # rotate to max measure 
            omega = math.radians(degree)/ TIME
            rospy.loginfo(f"Omega: {omega}")
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0
            cmd_vel.linear.y = 0.0
            cmd_vel.linear.z = 0.0
            cmd_vel.angular.x = 0.0
            cmd_vel.angular.y = 0.0
            cmd_vel.angular.z = omega
            cmd_vel_pub.publish(cmd_vel)
            rospy.Timer(rospy.Duration(secs=TIME),alignment_elapsed, oneshot=True)

            # take a neighbourhood [-10 10] degrees of the maximum point
            N_NEIGHBOURS = 10
            start_indx = (degree - N_NEIGHBOURS)%359
            end_indx = (degree + N_NEIGHBOURS + 1)%359
            neighbourhood = []
            if end_indx > start_indx:
                neighbourhood = laser_values[start_indx : end_indx]
            else:
                neighbourhood_until_end = laser_values[start_indx :]
                neighbourhood = neighbourhood_until_end + laser_values[0 : end_indx]
            rospy.loginfo (f"neighbourhood: {neighbourhood}, start: {start_indx}, end: {end_indx}")
            min_measure = min(neighbourhood)
            degree = neighbourhood.index(min_measure)
            rospy.loginfo (f"min: {min_measure}, degree: {degree}")
            input("press2")
            if min_measure == math.inf:
                min_measure = 3.15
            # move to maximum_value/2
            utils.trapezoidal_motion(cmd_vel_pub, (min_measure/2))

            # check covariance information 
            try:
                # get the last estimated pose
                estimated_pose = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped)
                covariance_x = estimated_pose.pose.covariance[0]
                covariance_y = estimated_pose.pose.covariance[7]
                covariance_yow = estimated_pose.pose.covariance[35]
                cmd_vel_pub.publish(cmd_vel_msg)
                if covariance_x < COVARIANCE_X_THRESHOLD and covariance_y < COVARIANCE_Y_THRESHOLD and covariance_yow < COVARIANCE_YAW_THRESHOLD:
                    cmd_vel_msg.angular.z = 0
                    cmd_vel_pub.publish(cmd_vel_msg)
                    input ("Localization completed, press any key to reach next waypoint")
                    return True
            except:
                rospy.loginfo("AMCL Pose not updated")

if __name__ == '__main__':

    rospy.init_node("navigation_node")
    rate = rospy.Rate(0.2)
    
    # complete the name of the path file
    path_file_path = path_file_path + args.path_file_number + ".csv"
    rospy.loginfo(f"Path under test {path_file_path}")
    
    
    # create log file
    now = datetime.now()
    dt_string = now.strftime("%d%m%Y_%H:%M")
    folder = dt_string.split("_")[0]
    name = dt_string.split("_")[1]
    directory = str(pkg_path) + "/log/"+str(folder)
    if not os.path.exists(directory):
        os.makedirs(directory)
    log_file = open(str(directory)+"/"+str(name)+f"_path_{args.path_file_number}"+".txt","w")

    # initialize the robot pose
    initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
    gazebo_initial_pose_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)

    # create a move_base client
    move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    # Waits until the action server has started up and started
    # listening for goals.
    rospy.loginfo("Waiting for action server....")
    move_base_client.wait_for_server()

    # get the path waypoints
    waypoints, initial_pose = utils.read_csv_file(path_file_path)
    # publish waypoint markers
    markers_pub = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=100)
    markers = utils.create_markers(waypoints)
    markers_pub.publish(markers_pub)
    # log_file.close()
    # sys.exit()
    rospy.loginfo("Publishing initial pose....")
    rospy.loginfo(f"Initial Pose {initial_pose}")
    if args.run_automatic_initialization_procedure == "True":
        initial_pose = initialize_pose(initial_pose)

    rospy.loginfo(f"Initial Pose {initial_pose}")
    initial_pose_pub.publish(initial_pose)
    rospy.loginfo(f"Waypoints: {waypoints}")
    rospy.loginfo("###########")
    
    if args.run_automatic_initialization_procedure == "True":
        input("Press any key to start the initialization of localization:")
        automatic_initialization_procedure()

        # check if the robot have reached the first wp
        '''estimated_pose = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped)
        
        wp_x = waypoints[0][0]
        wp_y = waypoints[0][1]

        diff_x = abs(wp_x - estimated_pose.pose.pose.position.x)
        diff_y = abs(wp_y - estimated_pose.pose.pose.position.y)
        if diff_x < ERROR_X and diff_y < ERROR_Y:
            rospy.loginfo(f"Waypoint reached")
        else:
            _ = go_to_next_wp(wp=waypoints[0], move_base_client=move_base_client, time = 0)'''
        _ = go_to_next_wp(wp=waypoints[0], move_base_client=move_base_client, time = 0)

    #rate.sleep()
    
    input("Press any key to start the navigation:")
    log_file.write(f"Start Simulation. \nStart point: {waypoints[0]} - Goal point: {waypoints[-1]}")
    curr_time = rospy.Time.now().secs + (rospy.Time.now().nsecs * 10**-9)
    start_time = curr_time
    log_file.write(f"\n\n\nStart time: {curr_time}")
    for i, wp in enumerate(waypoints[1:]):
        rospy.loginfo(f"Waypoint number {i}\n{wp}")
        log_file.write(f"\n\nWaypoint number: {i}\n{wp}")
        curr_time = go_to_next_wp(wp=wp, move_base_client=move_base_client, time = curr_time)
        #input("Press any key to continue:")
        rate.sleep()
        
    minutes, sec = divmod(int(curr_time) - start_time, 60)
    log_file.write(f"\n\nGoal reached in: {minutes} m {int(sec)} s")
    log_file.close()
    
    """
    while(not rospy.is_shutdown()):
        rate.sleep()
    """