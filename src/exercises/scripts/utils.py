import rospy
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Pose

def create_markers(waypoints):
    marker_array = MarkerArray()
    marker_array.markers = []

    for i in range(len(waypoints)):
        # marker header 
        marker = Marker()
        marker.header.frame_id = "odom"
        
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