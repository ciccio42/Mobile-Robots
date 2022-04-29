from turtle import distance
from numpy import angle
import rospy
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Pose, Twist
import math



def create_path():
    # create a sequence of waypoits [x,y,theta], with respect to map
    waypoints = []
       
    # move forward of 0.5 m along the x
    # waypoints.append([0.5, 0, 0])
    
    # turn right
    waypoints.append([0.0, -1.0, -math.pi/2])
    
    # turn left
    waypoints.append([1.0, -1.0, 0])
    
    # move forward
    waypoints.append([1.0, 0.0, math.pi/2])

    # move forward
    waypoints.append([1.0, 1.0, math.pi/2])
    
    # turn left
    waypoints.append([0.0, 1.0, math.pi])
    
    # go to starting point
    waypoints.append([0, 0.0, -math.pi/2])

    return waypoints
    

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

def convert_polar_to_cartesian(ray: float, angle: float):
    
    x = ray * math.cos(angle)
    y = ray * math.sin(angle)
    return [x,y]

def trapezoidal_motion(cmd_vel_pub: rospy.Publisher,delta_x):
    t_f = 10.0
    vel_c = 2 * (delta_x) / t_f
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
    