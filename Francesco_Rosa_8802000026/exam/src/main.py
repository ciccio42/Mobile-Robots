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

import numpy as np 
from datetime import datetime
import os
import tf
import sys

# get the path to the current package
rospack = rospkg.RosPack()
pkg_path = rospack.get_path('exam')
utils_lib_path = os.path.join(pkg_path, "scripts")
sys.path.append(utils_lib_path)
import utils
from utils import TIME

import argparse
 
parser = argparse.ArgumentParser(description="Navigation Node parameters",
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument("-num_path", "--path-file-number", default="1", help="Number of path file")
parser.add_argument("-run_aip", "--run-automatic-initialization-procedure", default="False", help="Whether run the automatic initialization procedure or not")
args, unknown = parser.parse_known_args()

# Mean initialization error
initialization_error_mean = 0.0
# STD DEV initialization error
initialization_error_std_dev_x = 0.05 # m^2
initialization_error_std_dev_y = 0.05 # m^2
initialization_error_std_dev_yaw = math.pi/4 # rad

# Covariance threshold for initial localization
COVARIANCE_X_THRESHOLD = 1e-1 # m^2
COVARIANCE_Y_THRESHOLD = 1e-1 # m^2
COVARIANCE_YAW_THRESHOLD = 0.1 # rad^2
TIME_LIMIT_FOR_INITIAL_LOCALIZATION = (2*math.pi)#*2 # The time required to complete one rotation around the z-axis
WRITE_WP_TIME = False
# waypoints file path
path_file_path = os.path.join(pkg_path, f"config/path_")
cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

ERROR_X = 0.20
ERROR_Y = 0.20

TIME_ELAPSED = False

def publish_pose_gazebo(service_proxy: rospy.ServiceProxy, wp: list):
    """Publish the initial pose in gazebo
        
        Parameters
        ----------
            service_proxy: rospy.ServiceProxy
                Service to call 
            wp: list
                Source waypoint
        Returns
        -------
    """          
        
    # create service message
    model_state_req = ModelState()

    # model name
    model_state_req.model_name = "turtlebot3_waffle_pi"

    # reference frame
    model_state_req.reference_frame = "world"

    # position
    model_state_req.pose.position.x = wp[0]
    model_state_req.pose.position.y = wp[1]
    model_state_req.pose.position.z = 0.0
    
    # orientation
    orientation = tf.transformations.quaternion_about_axis(wp[2], (0, 0, 1))
    model_state_req.pose.orientation.x = orientation[0]
    model_state_req.pose.orientation.y = orientation[1]
    model_state_req.pose.orientation.z = orientation[2]
    model_state_req.pose.orientation.w = orientation[3]

    # call service
    service_proxy(model_state_req)

def publish_initial_pose_amcl(initial_pose: PoseWithCovarianceStamped):
    """Publish the initial pose for amcl
        
        Parameters
        ----------
            initial_pose: PoseWithCovarianceStamped
                Initial pose to publish
        Returns
        -------
    """    
    def initialize_pose(initial_pose: PoseWithCovarianceStamped) -> PoseWithCovarianceStamped:
        """Initialize the covariance of initial pose
        
        Parameters
        ----------
            initial_pose: PoseWithCovarianceStamped
                Initial pose to publish
            
        Returns
        -------
            PoseWithCovarianceStamped
                Initial pose with covariance
        """
        initial_pose.header.stamp = rospy.Time.now()
        # at the beginning the robot is supposed to be in the starting wp
        # however the related variance is high, since there is a huge uncertainty at the beginning
        initial_pose.pose.covariance[0] = initialization_error_std_dev_x
        initial_pose.pose.covariance[7] = initialization_error_std_dev_y
        initial_pose.pose.covariance[35] = initialization_error_std_dev_yaw
        return initial_pose
   
    rospy.loginfo("Publishing initial pose....")
    rospy.loginfo(f"Initial Pose {initial_pose}")
    if args.run_automatic_initialization_procedure == "True":
        initial_pose = initialize_pose(initial_pose)

    rospy.loginfo(f"Initial Pose {initial_pose}")
    initial_pose_pub.publish(initial_pose)

def init_amcl():
    """Initialize amcl with initial pose
        
        Parameters
        ----------
            
        Returns
        -------
    """
    if rospy.get_param("sim") == True:
        publish_initial_pose_amcl(initial_pose)     
        rospy.Rate(0.3).sleep()
        # wait for gazebo service
        rospy.wait_for_service('/gazebo/set_model_state')
        gazebo_initial_pose_service_proxy = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        publish_pose_gazebo(gazebo_initial_pose_service_proxy, waypoints[0])
        rospy.Rate(0.3).sleep()
        publish_initial_pose_amcl(initial_pose) 
        rospy.Rate(0.3).sleep()
    else:
        publish_initial_pose_amcl(initial_pose) 

def go_to_next_wp(wp: list, move_base_client: actionlib.SimpleActionClient, time):
    """Navigate towards the next wp 
        
        Parameters
        ----------
            wp: list
                Waypoint to reach
            move_base_client: actionlib.SimpleActionClient
                Move_base client
            time: rospy.Time.now()
                Starting time to reach waypoint
        Returns
        -------
            curr_time: rospy.Time.now()
                Time of arrival 
            flag: boolean
                True for waypoint reached, False for failure

    """
    goal = utils.create_goal_msg(wp)
    rospy.loginfo(f"Sending goal {goal}")
    move_base_client.send_goal(goal=goal.goal)
    rospy.loginfo(f"Waiting for result....")
    move_base_client.wait_for_result()
    result = move_base_client.get_state()
    curr_time = rospy.Time.now().secs + (rospy.Time.now().nsecs * 10**-9)
    if result == GoalStatus.SUCCEEDED:
        rospy.loginfo("Waypoint reached")
        if WRITE_WP_TIME:
            log_file.write(f" ---- Waypoint reached in: {round(curr_time - time, 3)} s")
        return curr_time, True
    else:
        log_file.write(f"---- Error: {result}")
        rospy.logerr(f"Action error {result}")
        return curr_time, False

def timer_elapsed(event=None):
    """Funtion to stop the robot after the end of a Timer 
        
        Parameters
        ----------
        Returns
        -------
    """
    # stop the robot
    cmd_vel = Twist()
    cmd_vel.linear.x = 0.0
    cmd_vel.linear.y = 0.0
    cmd_vel.linear.z = 0.0
    cmd_vel.angular.x = 0.0
    cmd_vel.angular.y = 0.0
    cmd_vel.angular.z = 0.0
    cmd_vel_pub.publish(cmd_vel)
    global TIME_ELAPSED
    TIME_ELAPSED = True

def get_amcl_pose():
    amcl_update_client()
    while True:
        try:
            estimated_pose = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped)
            return estimated_pose
        except:
            rospy.loginfo("AMCL Pose not updated")
            return None

def rotation_procedure(open_space):
    """Function to spin the robot for a set number of seconds 
        
        Parameters
        ----------
            open_space: boolean
                Flag to indicate the stage of localization
        Returns
        -------
    """
    cmd_vel_msg = Twist()
    cmd_vel_msg.angular.z = 1.0 # rad/s
    start_time = rospy.Time.now()
    rate = rospy.Rate(10)
    rospy.loginfo("Start localization rotation procedure")
    while True:
        rate.sleep()
        current_time = rospy.Time.now()
        elapsed_time = (current_time.secs + (current_time.nsecs * 10**-9)) - (start_time.secs + (start_time.nsecs * 10**-9))
        rospy.logdebug(f"Localization phase-\nElapsed time {elapsed_time}")
        cmd_vel_pub.publish(cmd_vel_msg)
        estimated_pose = get_amcl_pose()
        covariance_x = estimated_pose.pose.covariance[0]
        covariance_y = estimated_pose.pose.covariance[7]
        covariance_yaw = estimated_pose.pose.covariance[35]
            
        if ((abs(covariance_x) < COVARIANCE_X_THRESHOLD and abs(covariance_y) < COVARIANCE_Y_THRESHOLD) and open_space == False)  or elapsed_time >= TIME_LIMIT_FOR_INITIAL_LOCALIZATION:
            if (abs(covariance_x) < COVARIANCE_X_THRESHOLD and abs(covariance_y) < COVARIANCE_Y_THRESHOLD):
                rospy.loginfo("Covariance under threshold")
            else:
                rospy.loginfo("Rotating Time elapsed")
            cmd_vel_msg.angular.z = 0
            cmd_vel_pub.publish(cmd_vel_msg)
            break

def automatic_localization_precedure():
    """Perform the initial localization procedure, at the start-up
    """

    def check_estimated_pose_variance():
        """Check whether the estimated pose covariance is under a certain threshold
        Returns
        -------
            result: bool
                True, if the estimated pose covariance is under a certain threshold, False otherwise
        """
        estimated_pose = get_amcl_pose()
        rospy.loginfo (f"Estimated pose:\n{estimated_pose}")
        covariance_x = estimated_pose.pose.covariance[0]
        covariance_y = estimated_pose.pose.covariance[7]
        covariance_yaw = estimated_pose.pose.covariance[35]

        if abs(covariance_x) < COVARIANCE_X_THRESHOLD and abs(covariance_y) < COVARIANCE_Y_THRESHOLD and abs(covariance_yaw) < COVARIANCE_YAW_THRESHOLD:
            localization = True
            return True
        else:
            localization = False
            rospy.loginfo("Localization not completed")
            return False  
                        
    def automatic_localization_procedure_step_1():
        """Locating procedure that moves the robot in the direction of the maximum distance read by the laser scan.
           The displacement is equal to the minimum distance in the neighourhood of the maximum.
        
        Returns
        -------
            result: bool
                True, if the estimated pose covariance is under a certain threshold, False otherwise
        """
        
        input("Press any key to start to move into open space")
        while True:
            laser_values = utils.get_laser_scan("/scan")
            # take the maximum value and the degree
            max_measure_indx = np.nanargmax(laser_values)
            max_value = laser_values[max_measure_indx]
            rospy.loginfo (f"max: {max_value}, degree: {max_measure_indx}")
            input("Press any key to start to align with moving direction")
            # rotate to max measure 
            omega = math.radians(max_measure_indx)/ TIME
            rospy.loginfo(f"Omega: {omega}")
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0
            cmd_vel.linear.y = 0.0
            cmd_vel.linear.z = 0.0
            cmd_vel.angular.x = 0.0
            cmd_vel.angular.y = 0.0
            cmd_vel.angular.z = omega
            cmd_vel_pub.publish(cmd_vel)
            rospy.Timer(rospy.Duration(secs=TIME),timer_elapsed, oneshot=True)
            global TIME_ELAPSED
            TIME_ELAPSED = False
            while not TIME_ELAPSED:
                cmd_vel_pub.publish(cmd_vel)
            cmd_vel.angular.z = 0.0
            cmd_vel_pub.publish(cmd_vel)
            # take a neighbourhood [-10 10] degrees of the maximum point
            laser_values = utils.get_laser_scan("/scan")
            N_NEIGHBOURS = 10
            neighbourhood_left = laser_values[:N_NEIGHBOURS]
            neighbourhood_right = laser_values[359-N_NEIGHBOURS:359]
            neighbourhood = np.concatenate((neighbourhood_left,neighbourhood_right))
            rospy.loginfo (f"neighbourhood: {neighbourhood} - size: {np.size(neighbourhood)}")

            try:
                min_measure_indx = np.nanargmin(neighbourhood)
                min_value = neighbourhood[min_measure_indx]
                # min value could be inf or numerical
                rospy.loginfo (f"min: {min_value}, degree: {min_measure_indx}")
                input("Press any key to start to move")
                if min_value == 10:
                    min_value = 3.5

                # move to min_value - security threshold
                utils.trapezoidal_motion(cmd_vel_pub, (min_value-0.40))
            except ValueError:
                # all nans
                # do nothing
                pass

            rotation_procedure(True)

            if check_estimated_pose_variance():
                # convergence reached
                input ("Localization completed, press any key to reach next waypoint")
                return True
    
    return automatic_localization_procedure_step_1()
    
def align_with_source_wp(theta):
    """Function to align the robot to next waypoint
        
        Parameters
        ----------
            theta: float
                Angle in radians of next waypoint
        Returns
        -------
    """
    # reach wp orientation
    estimated_pose = get_amcl_pose()
    _,_, y = tf.transformations.euler_from_quaternion([estimated_pose.pose.pose.orientation.x, 
                                                       estimated_pose.pose.pose.orientation.y, 
                                                       estimated_pose.pose.pose.orientation.z, 
                                                       estimated_pose.pose.pose.orientation.w])

    delta_theta = (theta-y)
    rospy.loginfo(f"Source wp:\n{theta}\nY:\n{y}")
    rospy.loginfo("\nAlignment with source waypoint orientation:\ntheta {}".format(delta_theta))
    omega = delta_theta / TIME
    rospy.loginfo (f"\n\nOmega alignment: {omega}")
    cmd_vel = Twist()
    cmd_vel.linear.x = 0.0
    cmd_vel.linear.y = 0.0
    cmd_vel.linear.z = 0.0
    cmd_vel.angular.x = 0.0
    cmd_vel.angular.y = 0.0
    cmd_vel.angular.z = omega
    cmd_vel_pub.publish(cmd_vel)
    rospy.Timer(rospy.Duration(secs=TIME),timer_elapsed, oneshot=True)
    global TIME_ELAPSED
    TIME_ELAPSED = False
    while not TIME_ELAPSED:
        cmd_vel_pub.publish(cmd_vel)
        pass
    cmd_vel.angular.z = 0.0
    cmd_vel_pub.publish(cmd_vel)
    rospy.loginfo ("Alignment Completed")

if __name__ == '__main__':

    rospy.init_node("navigation_node")
    
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
    initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=100)

    # marker array publisher
    markers_pub = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=100)
    
    # create a move_base client
    move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # create a amcl update client 
    rospy.loginfo("Waiting for request_nomotion_update service...")
    rospy.wait_for_service("request_nomotion_update")
    amcl_update_client = rospy.ServiceProxy("request_nomotion_update", Empty)

    # create service client to clean up the local coastmap
    rospy.loginfo("Waiting for clear_costmaps...")
    rospy.wait_for_service("/move_base/clear_costmaps")
    clear_costmaps_client = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)

    # Waits until the action server has started up and started
    # listening for goals.
    rospy.loginfo("Waiting for move base action server....")
    move_base_client.wait_for_server()

    # get the path waypoints
    waypoints, initial_pose = utils.read_csv_file(path_file_path)
    rospy.loginfo(f"Waypoints:\n{waypoints}")
    rospy.loginfo("###########")
    rate = rospy.Rate(1)
    # publish waypoint markers
    markers = utils.create_markers(waypoints)
    for i in range(5):
        markers_pub.publish(markers)
        rate.sleep()

    init_amcl()
    
    if args.run_automatic_initialization_procedure == "True":
        
        input("Press any key to start the localization:")
        if automatic_localization_precedure() == True:
            rospy.loginfo("Initial localization has reached convergence")
            rospy.Rate(0.2).sleep()
        
        # check if the robot has reached the first wp
        estimated_pose = get_amcl_pose()
        wp_x = waypoints[0][0]
        wp_y = waypoints[0][1]

        diff_x = abs(wp_x - estimated_pose.pose.pose.position.x)
        diff_y = abs(wp_y - estimated_pose.pose.pose.position.y)
        log_file.write(f"Localization Error--> x: {diff_x} - y: {diff_y}")
        if diff_x < ERROR_X and diff_y < ERROR_Y:
            rospy.loginfo(f"Source reached")
            align_with_source_wp(waypoints[0][2])
        else:
            _, _ = go_to_next_wp(wp=waypoints[0], move_base_client=move_base_client, time = 0)

    input("Press any key to start the navigation:")
    log_file.write(f"Start Simulation. \nStart point: {waypoints[0]} - Goal point: {waypoints[-1]}")
    curr_time = rospy.Time.now().secs + (rospy.Time.now().nsecs * 10**-9)
    start_time = curr_time
    log_file.write(f"\n\n\nStart time: {curr_time}")
    for i, wp in enumerate(waypoints[1:]):
        clear_costmaps_client()
        rospy.loginfo(f"Waypoint number:\n{i}\n{wp}")
        if WRITE_WP_TIME:
            log_file.write(f"\n\nWaypoint number: {i}\n{wp}")
        curr_time, res = go_to_next_wp(wp=wp, move_base_client=move_base_client, time = curr_time)
        if res == False:
            rospy.logerr("Go to next wp failed. EXIT")
            break
        # input("Press any key to continue:")
        # rate.sleep()
        
    minutes, sec = divmod(int(curr_time) - start_time, 60)
    log_file.write(f"\n\nGoal reached in: {minutes} m {int(sec)} s")
    log_file.close()
