import os, sys
import rospy, rospkg
from visualization_msgs.msg import MarkerArray
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
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

# Mean initialization error
initialization_error_mean = 0.0
# STD DEV initialization error
initialization_error_std_dev_x = 50 # m
initialization_error_std_dev_y = 30 # m
initialization_error_theta = 0.0 # rad

# waypoints file path
conf_file_path = os.path.join(pkg_path, "config/conf.csv")

# log file 



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
    """Perform the initial localization
    """
    # 

    pass

if __name__ == '__main__':
    rospy.init_node("navigation_node")
    rate = rospy.Rate(0.2)
    now = datetime.now()
    dt_string = now.strftime("%d%m%Y_%H:%M")
    folder = dt_string.split("_")[0]
    name = dt_string.split("_")[1]
    directory = str(pkg_path) + "/log/"+str(folder)
    if not os.path.exists(directory):
        os.makedirs(directory)
    log_file = open(str(directory)+"/"+str(name)+".txt","w")

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
    waypoints, initial_pose = utils.read_csv_file(conf_file_path)
    log_file.write(f"Start Simulation. \nStart point: {waypoints[0]} - Goal point: {waypoints[-1]}")
    
    # log_file.close()
    # sys.exit()
    rospy.loginfo("Publishing initial pose....")
    rospy.loginfo(f"Initial Pose {initial_pose}")
    # initial_pose = initialize_pose(initial_pose)
    rospy.loginfo(f"Initial Pose {initial_pose}")
    initial_pose_pub.publish(initial_pose)
    rospy.loginfo(f"Waypoints: {waypoints}")
    rospy.loginfo("###########")
    
    automatic_initialization_procedure()

    #rate.sleep()
    input("Press any key to start the navigation:")
    
    curr_time = rospy.Time.now().secs + (rospy.Time.now().nsecs * 10**-9)
    start_time = curr_time
    log_file.write(f"\n\n\nStart time: {curr_time}")
    for i, wp in enumerate(waypoints):
        rospy.loginfo(f"Waypoint number {i}\n{wp}")
        log_file.write(f"\n\nWaypoint number: {i}\n{wp}")
        curr_time = go_to_next_wp(wp=wp, move_base_client=move_base_client, time = curr_time)
        rate.sleep()
    minutes, sec = divmod(int(curr_time) - start_time, 60)
    log_file.write(f"\n\nGoal reached in: {minutes} m {int(sec)} s")
    log_file.close()
    while(not rospy.is_shutdown()):
        rate.sleep()