from typing import List, Tuple
import rospy
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Pose, Twist, PoseStamped, PoseWithCovarianceStamped, PoseWithCovariance
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseGoal
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from tf import TransformListener
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
                    pose_with_covariance.covariance[diagonal_index] = 1000000000.0 # since the robot is planar and the z term is not estimated
                else:
                    pose_with_covariance.covariance[diagonal_index] = 10**-5 # the initial pose is supposed known 
            else:
                if i == 3 or i == 4:
                    pose_with_covariance.covariance[diagonal_index] = 1000000000.0 # since the robot is planar and the roll and pitch are not estimated
                else:
                    pose_with_covariance.covariance[diagonal_index] = 0.001 # the initial orientation is supposed known        
        
        pose_stamped_with_covariance.pose = pose_with_covariance
        return pose_stamped_with_covariance

    def compute_wp_orientation(current_wp:list, next_wp:list) -> float:
        """Compute the orientation to give to the current waypoint with respect to the next waypoint
        
        Parameters
        ----------
            current_wp: list
                Current waypoint
            next_wp: list
                Next waypoint
        Returns
        -------
            orientation: float
        """          
        """# compute the difference vector between the current and next wp
        diff_vector = np.subtract(np.array(next_wp), np.array(current_wp))
        # compute the orientation between the current wp and the next one
        return np.arctan2(diff_vector[1], diff_vector[0])"""

        def convert_from_cartesian_coordinate_to_pixel(cartesian_coordinates:list, map: OccupancyGrid):
            """Convert Cartesian point (x,y) in pixel point (u,v)
            Parameters
            ----------
                cartesian_coordinate: list
                    Cartesian coordinate
            Returns
            -------
                coordinates_world_px: np.array
                    Coordinate of point in pixel (row,col)
            """ 
            origin_pose = map.info.origin
            A_world_map = np.identity(3)
            A_world_map[0][2] = - origin_pose.position.x
            A_world_map[1][2] = - origin_pose.position.y
            # convert cartesian coordinate with respect to world frame (cell (0,0))
            cartesian_coordinates_map = np.array([[cartesian_coordinates[0], cartesian_coordinates[1], 1]]).transpose()
            cartesian_coordinates_world = np.matmul(A_world_map, cartesian_coordinates_map)
            coordinates_world_px = np.array([int(cartesian_coordinates_world[0][0]/map.info.resolution), 
                                             int(cartesian_coordinates_world[1][0]/map.info.resolution)])
            return coordinates_world_px

        def check_obstacles(current_wp : list, next_wp : list, axis = 'y'):
            # read the map
            map = rospy.wait_for_message("/map", OccupancyGrid)
            # compute the wp pixel
            current_px = convert_from_cartesian_coordinate_to_pixel(current_wp, map)
            next_px  = convert_from_cartesian_coordinate_to_pixel(next_wp, map)
            
            map_data = np.reshape(np.array(map.data), (map.info.height, map.info.width))
            rospy.logdebug(f"Map shape: {np.shape(map_data)}")
            if axis == 'y':
                rospy.loginfo(f"Aligned pixels {current_px}, {next_px}")
                rospy.loginfo(f"{current_px[1]} - {next_px[1]}")
                map_slice = None
                if current_px[1] > next_px[1]:
                    map_slice = map_data[int(next_px[1]):int(current_px[1])+1, int(current_px[0])]
                elif current_px[1] < next_px[1]:
                    map_slice = map_data[int(current_px[1]):int(next_px[1])+1, int(current_px[0])]
                for px in map_slice:
                    if px > 60:
                        return True
                return False            
            elif axis == 'xy':
                # check for obstacle in submatrix with coordinates current_px and next_px
                map_slice = None
                rospy.loginfo(f"Map slice over {current_px} - {next_px}")
                if current_px[0] > next_px[0]:
                    col_slice = map_data[:, int(next_px[0]):int(current_px[0])+1]
                else:
                    col_slice = map_data[:, int(current_px[0]):int(next_px[0])+1]
                
                if current_px[1] > next_px[1]:
                    map_slice = col_slice[int(next_px[1]):int(current_px[1])+1, :]
                else:
                    map_slice = col_slice[int(current_px[1]):int(next_px[1])+1, :]
                
                row = np.shape(map_slice)[0]
                col = np.shape(map_slice)[1]
                
                n = min (row, col)

                if current_px[0] < next_px[0] and current_px[1]<next_px[1]:
                    #top-left
                    # check submatrix diagonal 
                    for i in range(n):
                        if map_slice[i][i] > 60:
                            rospy.loginfo("\n######### Ostacolo nella diagonale")
                            return True 

                    # check the remaining path
                    if row > col:
                        for i in range (row-col):
                            if map_slice[i + col -1][col-1] > 60:
                                rospy.loginfo("\n######### Ostacolo 1")
                                return True 
                    else:
                        for i in range (col-row):
                            if map_slice[row-1][row+i-1] > 60:
                                rospy.loginfo("\n######### Ostacolo 2")
                                return True 


                elif current_px[0] > next_px[0] and current_px[1]<next_px[1]:
                    #top-right
                    # check submatrix diagonal 
                    
                    if row > col:
                        i = 0
                        for j in range(col-1, -1, -1):
                            if map_slice[i][j] > 60:
                                rospy.loginfo("\n######### Ostacolo 2")
                                return True 
                            i = i + 1 
                        for i in range (row-col, row):
                            if map_slice[i][0] > 60:
                                rospy.loginfo("\n######### Ostacolo 2")
                                return True 
                    else:
                        i = 0
                        for j in range(col-1, n-1, -1):
                            if map_slice[i][j] > 60:
                                rospy.loginfo("\n######### Ostacolo 2")
                                return True 
                            i = i + 1 
                        for i in range (col-row, 0, -1):
                            if map_slice[row-1][i-1] > 60:
                                rospy.loginfo("\n######### Ostacolo 2")
                                return True 


                elif current_px[0] > next_px[0] and current_px[1]>next_px[1]:
                    #bottom-right
                    for i in range(n):
                         if map_slice[row-1-i][col-1-i] > 60:
                                rospy.loginfo("\n######### Ostacolo 2")
                                return True 
                    
                        
                    if row > col:
                        for i in range(row-col, -1, -1):
                             if map_slice[row-col-i][0] > 60:
                                rospy.loginfo("\n######### Ostacolo 2")
                                return True 
                    else:
                        for i in range (col-row-1, -1, -1):
                         if map_slice[0][i]> 60:
                                rospy.loginfo("\n######### Ostacolo 2")
                                return True 
                        

            
                elif current_px[0] < next_px[0] and current_px[1]>next_px[1]:
                    #bottom-left
                    for i in range (n):
                        if map_slice[row-i-1][i] > 60:
                            rospy.loginfo("\n######### Ostacolo 2")
                            return True 
                    if row > col:
                        for i in range (row-col-1, -1, -1):
                            if map_slice[i][col-1] > 60:
                                rospy.loginfo("\n######### Ostacolo 2")
                                return True 
                    else:
                        for i in range (col-row-1, col, 1) > 60:
                            if map_slice[0][i]:
                                rospy.loginfo("\n######### Ostacolo 2")
                                return True 
                        

                                    
                
                rospy.loginfo("\n######### Nessun Ostacolo")
                

                # for i in range(np.shape(map_slice)[0]):
                #     for j in range(np.shape(map_slice)[1]):
                #         if map_slice[i][j] > 60:
                #             return True
                # return False

        # get the x value
        x_current = current_wp[0]
        x_next = next_wp[0]
        y_current = current_wp[1]
        y_next = next_wp[1]
        
        orientation = None
        # check alignment along x-axis, in order to avoid alignment when there is an obstacle
        rospy.loginfo(f"Current wp {current_wp} - Next wp {next_wp}")
        if (x_next <= x_current + 0.15) and (x_next >= x_current - 0.15):
            # the waypoints are aligned along the x-direction
            # check for the presence of an obstacle 
            rospy.loginfo(f"Current wp {current_wp} - Next wp {next_wp}")
            if check_obstacles(current_wp=current_wp, next_wp=next_wp, axis='y'):
                # the waypoints are separed by an obstacle
                rospy.loginfo("Obstacle detected")
                orientation = math.pi
            else:
                # the waypoints are connected
                rospy.loginfo("Obstacle not detected")
                if y_next > y_current:
                    orientation = math.pi/2
                elif y_next < y_current:
                    orientation = -math.pi/2
        elif (x_next > x_current) or (x_next < x_current):
            if check_obstacles(current_wp=current_wp, next_wp=next_wp, axis='xy'):          
                if (x_next > x_current):
                    # the next wp is above the current wp
                    # set the orientation to zero
                    orientation = 0.0
                elif (x_next < x_current):
                    # the next wp is above the current wp
                    # set the orientation to zero
                    orientation = math.pi
            else:
                # the points are connected
                # compute the difference vector between the current and next wp
                diff_vector = np.subtract(np.array(next_wp), np.array(current_wp))
                orientation = np.arctan2(diff_vector[1], diff_vector[0])

        return orientation

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
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        #---- define marker pose ----#
        # position 
        marker.pose.position.x = waypoints[i][0]
        marker.pose.position.y = waypoints[i][1]
        marker.pose.position.z = 0.5
        # pose
        orientation = tf.transformations.quaternion_about_axis(waypoints[i][2], (0,0,1))
        marker.pose.orientation.x = orientation[0]
        marker.pose.orientation.y = orientation[1]
        marker.pose.orientation.z = orientation[2]
        marker.pose.orientation.w = orientation[3]

        marker.color.r, marker.color.g, marker.color.b = (0, 1, 0)
        marker.color.a = 1
        marker.scale.x, marker.scale.y, marker.scale.z = (0.5, 0.1, 0.1)
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
        measure_cartesian.append(convert_polar_to_cartesian(ray=ray, angle=((angle*2*math.pi)/(360))))

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

def get_laser_scan(laser_scan_topic: str):
    """Get the measure from laser scan
    
    Parameters
    ----------
        laser_scan_topic: str
            Laser scan topic name
    Returns
    -------
        ranges: np.array
            Array of measured ranges
    """
    laser_scan_msg = rospy.wait_for_message(laser_scan_topic, LaserScan)

    ranges = np.array(laser_scan_msg.ranges)
    intensities = np.array(laser_scan_msg.intensities)

    for indx, _ in enumerate(ranges):
        if ranges[indx] == 0.0 and intensities[indx] == 0.0:
            ranges[indx] = math.inf
        elif ranges[indx] == 0.0 and intensities[indx] != 0.0:     
            ranges[indx] = np.NaN
    return ranges

def get_measure_from_scan():
    tf_listener = TransformListener()
    tf_listener.waitForTransform("/odom", "/imu_link", rospy.Time(), rospy.Duration(20.0))
    initial_measure_polar = get_laser_scan("/scan")

    # convert polar to cartesian
    measures_cartesian = convert_laser_measure_polar_to_cartesian(initial_measure_polar.ranges)

    measures_cartesian_base_footprint = covert_laser_scan_to_frame(tf_listener=tf_listener, measure_base_scan=measures_cartesian, frame="base_footprint")
    rospy.logdebug("\nCurrent measure  \nfront: {} \nleft: {} \nbehind: {} \nright: {}".format(measures_cartesian_base_footprint[0],
                                                                                            measures_cartesian_base_footprint[90],
                                                                                            measures_cartesian_base_footprint[180],
                                                                                            measures_cartesian_base_footprint[270]))
    
                                           
    return measures_cartesian_base_footprint 
