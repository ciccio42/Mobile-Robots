import os, sys
import rospy, rospkg
from visualization_msgs.msg import MarkerArray
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction

# get the path to the current package
rospack = rospkg.RosPack()
pkg_path = rospack.get_path('exam')
utils_lib_path = os.path.join(pkg_path, "scripts")
sys.path.append(utils_lib_path)
import utils

# waypoints file path
conf_file_path = os.path.join(pkg_path, "config/conf.yaml")

def go_to_next_wp(wp: list, move_base_client: actionlib.SimpleActionClient):
    goal = utils.create_goal_msg(wp)
    rospy.loginfo(f"Sending goal {goal}")
    move_base_client.send_goal(goal=goal.goal)
    rospy.loginfo(f"Waiting for result....")
    move_base_client.wait_for_result()
    result = move_base_client.get_result()
    rospy.loginfo(f"Result {result}")

def initialize_pose():
    pass

if __name__ == '__main__':
    rospy.init_node("navigation_node")
    rate = rospy.Rate(0.2)
    
    # initialize the robot pose
    initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)

    # create a move_base client
    move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    # Waits until the action server has started up and started
    # listening for goals.
    rospy.loginfo("Waiting for action server....")
    move_base_client.wait_for_server()

    # get the path waypoints
    waypoints, initial_pose = utils.read_configuration_file(conf_file_path)
    rospy.loginfo(f"Waypoints: {waypoints}")

    rospy.loginfo("Publishing initial pose....")
    rospy.loginfo(f"Initial Pose {initial_pose}")
    initial_pose_pub.publish(initial_pose)
    
    rate.sleep()
    
    for i, wp in enumerate(waypoints):
        rospy.loginfo(f"Waypoint number {i}\n{wp}")
        go_to_next_wp(wp=wp, move_base_client=move_base_client)
        rate.sleep()
        
    while(not rospy.is_shutdown()):
        rate.sleep()