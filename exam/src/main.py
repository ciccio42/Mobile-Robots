import math
import os, sys
from threading import currentThread
import rospy, rospkg
from visualization_msgs.msg import MarkerArray
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, PoseArray
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseAction
from gazebo_msgs.msg import ModelState

import numpy as np 
from datetime import datetime
import os


# get the path to the current package
rospack = rospkg.RosPack()
pkg_path = rospack.get_path('exam')
utils_lib_path = os.path.join(pkg_path, "scripts")
sys.path.append(utils_lib_path)
import utils

import argparse
 
parser = argparse.ArgumentParser(description="Navigation Node parameters",
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument("-num_path", "--path-file-number", default="1", help="Number of path file")
parser.add_argument("-run_aip", "--run-automatic-initialization-procedure", default="False", help="Whether run the automatic initialization procedure or not")
args, unknown = parser.parse_known_args()

# Mean initialization error
initialization_error_mean = 0.0
# STD DEV initialization error
initialization_error_std_dev_x = 30 # m
initialization_error_std_dev_y = 20 # m
initialization_error_theta = 0.0 # rad

# Covariance threshold for initial localization
COVARIANCE_X_THRESHOLD = 1 # m
COVARIANCE_Y_THRESHOLD = 1 # m
TIME_LIMIT_FOR_INITIAL_LOCALIZATION = 2*math.pi # The time required to complete one rotation around the z-axis

# waypoints file path
path_file_path = os.path.join(pkg_path, f"config/path_")

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
    return initial_pose
    
def automatic_initialization_procedure():
    
    def move_towards_empty_space():        
        pass
    
    """Perform the initial localization
    """
    
    # Start by rotating
    rate = rospy.Rate(10)
    cmd_vel_msg = Twist()
    cmd_vel_msg.angular.z = 1.0 # rad/s
    # cmd_vel publisher
    cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    start_time = rospy.Time.now()
    rospy.loginfo("Start localization procedure")
    time_elapsed = False
    
    # count number of particles
    particle_cloud = rospy.wait_for_message("/particlecloud", PoseArray)
    rospy.loginfo(f"Number of particles {len(particle_cloud.poses)}")
    
    while True:
        rate.sleep()
        current_time = rospy.Time.now()
        elapsed_time = (current_time.secs + (current_time.nsecs * 10**-9)) - (start_time.secs + (start_time.nsecs * 10**-9))
        rospy.loginfo(f"Localization phase - Elapsed time {elapsed_time}")
        try:
            # get the last estimated pose
            estimated_pose = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped)
            covariance_x = estimated_pose.pose.covariance[0]
            covariance_y = estimated_pose.pose.covariance[7]
            cmd_vel_pub.publish(cmd_vel_msg)
            if covariance_x < COVARIANCE_X_THRESHOLD and covariance_y < COVARIANCE_Y_THRESHOLD and elapsed_time >= TIME_LIMIT_FOR_INITIAL_LOCALIZATION:
                cmd_vel_msg.angular.z = 0
                cmd_vel_pub.publish(cmd_vel_msg)
                return True
        except:
            rospy.loginfo("AMCL Pose not updated")
        
        if elapsed_time >= TIME_LIMIT_FOR_INITIAL_LOCALIZATION:
            cmd_vel_msg.angular.z = 0
            cmd_vel_pub.publish(cmd_vel_msg)
            time_elapsed = True
            break
    
    if time_elapsed == True:
        # check the axis with the highest variance
        estimated_pose = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped)
        covariance_x = estimated_pose.pose.covariance[0]
        covariance_y = estimated_pose.pose.covariance[7]
        rospy.loginfo(f"Covariance along x-axis {covariance_x}, y-axis {covariance_y}")
        if covariance_x > covariance_y:
            rospy.loginfo("Move along the x-axis to gain information")
            # get measures
            laser_scan = rospy.wait_for_message("", LaserScan)
        else:
            pass

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

    #rate.sleep()
    
    input("Press any key to start the navigation:")
    log_file.write(f"Start Simulation. \nStart point: {waypoints[0]} - Goal point: {waypoints[-1]}")
    curr_time = rospy.Time.now().secs + (rospy.Time.now().nsecs * 10**-9)
    start_time = curr_time
    log_file.write(f"\n\n\nStart time: {curr_time}")
    for i, wp in enumerate(waypoints):
        rospy.loginfo(f"Waypoint number {i}\n{wp}")
        log_file.write(f"\n\nWaypoint number: {i}\n{wp}")
        curr_time = go_to_next_wp(wp=wp, move_base_client=move_base_client, time = curr_time)
        input("Press any key to continue:")
        rate.sleep()
        
    minutes, sec = divmod(int(curr_time) - start_time, 60)
    log_file.write(f"\n\nGoal reached in: {minutes} m {int(sec)} s")
    log_file.close()
    
    """
    while(not rospy.is_shutdown()):
        rate.sleep()
    """