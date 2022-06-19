from typing import List, Tuple
import rospy, rospkg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point
from sensor_msgs.msg import JointState
from laser_line_extraction.msg import LineSegment, LineSegmentList 
from visualization_msgs.msg import Marker, MarkerArray
import tf
import rosbag
from std_msgs.msg import Float64
import message_filters

import numpy as np
import copy
import math
import os, sys
from datetime import datetime
import rosbag

# set the mean and std. var for guassian noise on linear motion model
MEAN = 0.0 # m
STD_DEV = 0.2 # m 
CLIP_ON_VARIATION_MOTION_MODEL = 0.4 # correspond to the 95-th percentile
K_l = 0.001
K_r = 0.001
validation_gate = 3.219 # corresponding to the 90-th percentile of a chi^2 distribution with 2 d.o.f
# Robot parameters
WHEEL_DISTANCE = 0.287 # m
WHEEL_RADIUS = 0.033 # m
LIDAR_MAX_RANGE = 3.5 # m 

# get the path to the current package
rospack = rospkg.RosPack()
pkg_path = rospack.get_path('exam')
map_lines_file_path= os.path.join(pkg_path, "config/map_lines.npy")
compute_loss = False
callback_cnt = 0

# get the path to the current package
rospack = rospkg.RosPack()
pkg_path = rospack.get_path('exam')
log_folder = os.path.join(pkg_path, "bag_error_folders")
now = datetime.now()
dt_string = now.strftime("%d%m%Y_%H_%M")
bag_file_dir = os.path.join(log_folder, dt_string)
if not os.path.exists(bag_file_dir) and compute_loss: 
    os.makedirs(bag_file_dir)

class KalmanFilter:
    
    def __init__(self, initial_pose:Pose, map_lines_file_path = None):        
        self.predicted_state = Odometry()
        self._initialize_state(self.predicted_state, initial_pose=initial_pose)
        self.last_time = rospy.Time.now()

        # initialize the updated state to the current initial state
        self.updated_state = copy.deepcopy(self.predicted_state)        

        # initialize the joint distance attraversed by the left and right wheel
        self.previous_right_wheel, self.previous_left_wheel = self._read_wheel_joints()
        self.current_right_wheel = self.previous_right_wheel 
        self.current_left_wheel = self.previous_left_wheel

        # load map lines
        self.map_lines = np.load(map_lines_file_path)

        self.validation_gate = validation_gate 

        self.predicted_line_markers_pub = rospy.Publisher("predicted_line_markers", MarkerArray, queue_size=100)

    def _initialize_state(self, state: Odometry, initial_pose: Pose):
        """
        Parameters
        ----------
        Returns
        -------
        """
        # initialize odometry state
        state.header.frame_id = 'odom'
        state.header.stamp = rospy.Time.now()
        state.child_frame_id = 'base_footprint'
        #---- POSE ----#
        state.pose.pose.position.x = initial_pose.position.x
        state.pose.pose.position.y = initial_pose.position.y
        state.pose.pose.position.z = initial_pose.position.z
        state.pose.pose.orientation.x = initial_pose.orientation.x
        state.pose.pose.orientation.y = initial_pose.orientation.y
        state.pose.pose.orientation.z = initial_pose.orientation.z
        state.pose.pose.orientation.w = initial_pose.orientation.w
        state.pose.covariance = list(np.zeros(36, np.float64))
        for i in range(6):
            diagonal_index = (i*6)+i
            if i < 3:
                if i == 2:
                    state.pose.covariance[diagonal_index] = 0.0 # since the robot is planar and the z term is not estimated
                else:
                    state.pose.covariance[diagonal_index] = 10**-5 # the initial pose is supposed known 
            else:
                if i == 3 or i == 4:
                    state.pose.covariance[diagonal_index] = 0.0 # since the robot is planar and the roll and pitch are not estimated
                else:
                    state.pose.covariance[diagonal_index] = 0.001 # the initial orientation is supposed known
        #---- TWIST ----#
        state.twist.twist.linear.x = 0.0
        state.twist.twist.linear.y = 0.0
        state.twist.twist.linear.z = 0.0
        state.twist.twist.angular.x = 0.0
        state.twist.twist.angular.y = 0.0
        state.twist.twist.angular.z = 0.0
        state.twist.covariance = list(np.zeros(36, np.float64))

    def _read_wheel_joints(self):
        """Read the /joint_states topic
        Parameters
        ----------
            
        Returns
        -------
            right_wheel: float
            left_wheel: float
        """
        joint_state_msg = rospy.wait_for_message("/joint_states", JointState)
        return joint_state_msg.position[0], joint_state_msg.position[1] 

    def _get_predicted_state_covariance(self):
        # get the covariance of estimated pose
        slice_row_indices = np.array([0, 1, 5])
        slice_colums_indices = np.array([0, 1, 5])
        return np.reshape(self.predicted_state.pose.covariance, [6,6])[slice_row_indices,:][:, slice_colums_indices]

    def _get_position_wrt_world_frame(self):
        # compute the robot pose with respect to world
        # The predicted robot is w.r.t. map 
        # World frame: origin bottom-left corner of the image
        # y: upwards
        # x: rightwards
        x_hat_map = self.predicted_state.pose.pose.position.x
        y_hat_map = self.predicted_state.pose.pose.position.y
        # map and world are aligned
        A_world_map = np.identity(3)
        A_world_map[0][2] = 32.514755
        A_world_map[1][2] = 21.044427 
        #print(np.matmul(A_world_map, np.array([[x_hat_map], [y_hat_map], [1]])))
        return np.matmul(A_world_map, np.array([[x_hat_map], [y_hat_map], [1]])) 

    def _get_yaw_angle_predicted_state(self):
        # Robot orientation and position estimated through odometry
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion=[self.predicted_state.pose.pose.orientation.x,
                                                                          self.predicted_state.pose.pose.orientation.y,
                                                                          self.predicted_state.pose.pose.orientation.z,
                                                                          self.predicted_state.pose.pose.orientation.w])
        return yaw

    def _publish_lines(self, lines, reference_frame):
        # create Marker Array Publisher
        marker_array = MarkerArray()
        marker_array.markers = []

        for line in lines:
            rospy.loginfo(f"Line {line}")
            # marker header 
            marker = Marker()
            marker.header.frame_id = reference_frame

            # marker field
            marker.type = Marker.LINE_LIST
            marker.action = Marker.ADD
            # pose
            marker.pose.orientation.w = 1

            marker.color.r, marker.color.g, marker.color.b = (0, 255, 0)
            marker.color.a = 0.5
            marker.scale.x, marker.scale.y, marker.scale.z = (0.06, 0.06, 0.06)
            
            # compute line points
            a = math.cos(line[1])
            b = math.sin(line[1])
            x0 = a * line[0]
            y0 = b * line[0]
            pt1 = (int(x0 + 100*(-b)), int(y0 + 100*(a)))
            pt2 = (int(x0 - 100*(-b)), int(y0 - 100*(a)))
            marker.points.append(Point(pt1[0], pt1[1], 0.02))
            marker.points.append(Point(pt2[0], pt2[1], 0.02))
            
            marker_array.markers.append(marker)

        # set markers id
        id = 0
        for m in marker_array.markers:
            m.id = id
            id += 1
        
        self.predicted_line_markers_pub.publish(marker_array)
        

    def _measurement_prediction(self)->Tuple[List, List]:
        """Compute the expected measures
        Parameters
        ----------
            
        Returns
        -------
        """
        # for j-th , compute the expected line measurement
        # z_hat_{j} = h_{j}(x_hat_t, m_{k})
        z_hat = []
        # for each predicted measure compute the jacobian
        predicted_measures_jacobians = []

        # get the orientation of the robot
        # world frame and map frame are aligned
        # the yaw angle in map frame is the same in world frame
        theta_hat = self._get_yaw_angle_predicted_state()
        # get the position wrt world frame
        p_world = self._get_position_wrt_world_frame()
        rospy.logdebug(f"Position wrt world frame {p_world}")
        
        for indx, line in enumerate(self.map_lines):
            # rho_hat = rho - (x_hat*cos(alpha) + y_hat*sin(alpha))
            rho_hat = line[0] - (p_world[0][0]*math.cos(line[1]) + p_world[1][0]*math.sin(line[1]))
            # alpha_hat = alpha - theta_hat 
            alpha_hat = line[1] - theta_hat
            z_hat.append([rho_hat, alpha_hat])
            predicted_measures_jacobians.append([[0, 0, -1], [-math.cos(line[1]), -math.sin(line[1]), 0]])
            rospy.logdebug(f"Line\n{line}")
            rospy.logdebug(f"Expected line\n{[rho_hat, alpha_hat]}")

        self._publish_lines(self.map_lines[1:5], "world")
        rospy.logdebug("#####----####")
        rospy.logdebug(f"Map Lines\n{self.map_lines}")
        rospy.logdebug(f"Number of expected lines\n{len(z_hat)}")
        rospy.logdebug(f"Expected Lines\n{z_hat}\n")
        
        return z_hat, predicted_measures_jacobians
    
    def _matching(self, z_hat: List, predicted_measures_jacobian_list: List, line_segments: List) -> Tuple[np.array, np.array, np.array]:
        """Compute the matching between predicted measure and current measure
        Parameters
        ----------
            z_hat: List
                List of predicted measure
            predicted_measures_jacobian_list: List
                List of predicted measures jacobian    
            line_segments: List
                List of measured segmentation
        Returns

        -------
        """ 
        
        # get the covariance of estimated pose
        predicted_state_covariance = self._get_predicted_state_covariance()
        # Tensors containing the complete matrices
        # Complete innovation array
        complete_innovation = None
        # measured line covariance block diagonal
        measured_line_covariance_block_diagonal = None
        # complete predicted line jacobians
        complete_predicted_line_jacobians = None

        match_cnt = 0
        for indx, line_segment_msg in enumerate(line_segments):
            # get the measured line
            # [angle
            #  radius]
            measured_line = np.array([[line_segment_msg.angle], [line_segment_msg.radius]])
            measured_line_covariance = np.reshape(line_segment_msg.covariance, [2,2])
            for indx_expected, predicted_measure in enumerate(z_hat):
                # [angle
                #  radius]
                predicted_line = np.array([[predicted_measure[1]], [predicted_measure[0]]])
                predicted_line_jacobian = np.array(predicted_measures_jacobian_list[indx_expected])
                # 1. Compute the innovation measure with relative covariance
                # compute the innovation: 2x1
                innovation = np.subtract(measured_line, predicted_line)
                # compute the innovation covariance
                # H_{j} * P_hat_t * H_{j}^T + R_{i}: 2x2
                innovation_covariance = np.add(np.matmul(np.matmul(predicted_line_jacobian, predicted_state_covariance), 
                                                                   predicted_line_jacobian.transpose()), 
                                               measured_line_covariance)
                
                # 2. Compute the mahalanobis distance
                # compute the mahalanobis distance
                # innovation^T * innovation_covariance^-1 * innovation
                mahalanobis_distance = np.matmul(np.matmul(innovation.transpose(), np.linalg.inv(innovation_covariance)), 
                                                          innovation)
                


                # check if the sample belongs to the validation gate
                if mahalanobis_distance < self.validation_gate:
                    # get the position wrt world frame
                    p_world = self._get_position_wrt_world_frame()
                    rospy.loginfo(f"Position wrt world frame {p_world}")
                    rospy.loginfo(f"####\nMeasured Line {measured_line}\n")
                    rospy.loginfo(f"\nPredicted Line {predicted_line}\n")
                    rospy.loginfo(f"\nInnovation {innovation}\n")
                    rospy.loginfo(f"\nMahalanobis distance {mahalanobis_distance}\n####")
                    
                    match_cnt += 1
                    if complete_innovation is None:
                        complete_innovation = innovation
                    else:
                        complete_innovation = np.vstack((complete_innovation, innovation))
                    
                    if measured_line_covariance_block_diagonal is None:
                        measured_line_covariance_block_diagonal = measured_line_covariance
                    else:
                        measured_line_covariance_block_diagonal = np.block([[measured_line_covariance_block_diagonal, np.zeros((2*(match_cnt-1),2))],
                                                                            [np.zeros((2,2*(match_cnt-1))), measured_line_covariance]])

                    if complete_predicted_line_jacobians is None:
                        complete_predicted_line_jacobians = predicted_line_jacobian
                    else:
                        complete_predicted_line_jacobians = np.vstack((complete_predicted_line_jacobians, predicted_line_jacobian))

        rospy.loginfo(f"Number of matching {match_cnt}")
        return complete_innovation, measured_line_covariance_block_diagonal, complete_predicted_line_jacobians

    def prediction(self, joint_state:JointState):
        """Perform the preditction step of the Kalman filter
        Parameters
        ----------
            cmd_vel: list()
                The command that has been sent to the robot. Linear and angular velocity
        Returns
        -------
        """

        # compute delta_s
        self.current_right_wheel, self.current_left_wheel = joint_state.position[0], joint_state.position[1] 
        # circumference * number of rotations = traveled distance 
        delta_s_right =  (self.current_right_wheel*WHEEL_RADIUS) - (self.previous_right_wheel*WHEEL_RADIUS)
        delta_s_left = (self.current_left_wheel*WHEEL_RADIUS) - (self.previous_left_wheel*WHEEL_RADIUS)
        rospy.logdebug(f"\nDelta s right {delta_s_right} - Delta s left {delta_s_left}")
        delta_s = (delta_s_right+delta_s_left)/2
        self.previous_right_wheel = self.current_right_wheel
        self.previous_left_wheel = self.current_left_wheel 
        
        # get yaw angle from previous state
        orientation = [self.updated_state.pose.pose.orientation.x, self.updated_state.pose.pose.orientation.y, 
                       self.updated_state.pose.pose.orientation.z, self.updated_state.pose.pose.orientation.w]
        _, _, theta = tf.transformations.euler_from_quaternion(orientation)

        # compute new position
        delta_theta = (delta_s_right - delta_s_left)/WHEEL_DISTANCE
        delta_x = delta_s * math.cos(theta + delta_theta/2)
        delta_y = delta_s * math.sin(theta + delta_theta/2) 
        
        # update predicted state
        #---- POSE ----#
        self.predicted_state.pose.pose.position.x = self.updated_state.pose.pose.position.x + delta_x
        self.predicted_state.pose.pose.position.y = self.updated_state.pose.pose.position.y + delta_y
        self.predicted_state.pose.pose.position.z = self.updated_state.pose.pose.position.z + 0.0
        # orientation
        # compute the new theta
        new_theta = theta + delta_theta
        new_quaternion = tf.transformations.quaternion_about_axis(new_theta, (0, 0, 1))
        self.predicted_state.pose.pose.orientation.x = new_quaternion[0] 
        self.predicted_state.pose.pose.orientation.y = new_quaternion[1]
        self.predicted_state.pose.pose.orientation.z = new_quaternion[2]
        self.predicted_state.pose.pose.orientation.w = new_quaternion[3]
        #---- POSE COVARIANCE ----#
        # compute jacobian of the motion model with respect to x,y,z,r,p,y
        motion_model_jacobian_motion = np.identity(3)
        motion_model_jacobian_motion[0][2] = -delta_s*math.sin(theta + delta_theta/2)
        motion_model_jacobian_motion[1][2] = delta_s*math.cos(theta + delta_theta/2)
        # compute the jacobian of the motion model with respect to the command
        motion_model_jacobian_command = np.zeros((3,2))
        motion_model_jacobian_command[0][0] = 1/2*math.cos(theta + delta_theta/2) - (delta_s/(2*WHEEL_DISTANCE))*math.sin(theta + delta_theta/2)
        motion_model_jacobian_command[0][1] = 1/2*math.cos(theta + delta_theta/2) + (delta_s/(2*WHEEL_DISTANCE))*math.sin(theta + delta_theta/2)
        motion_model_jacobian_command[1][0] = 1/2*math.sin(theta + delta_theta/2) + (delta_s/(2*WHEEL_DISTANCE))*math.cos(theta + delta_theta/2)
        motion_model_jacobian_command[1][1] = 1/2*math.sin(theta + delta_theta/2) - (delta_s/(2*WHEEL_DISTANCE))*math.cos(theta + delta_theta/2)
        motion_model_jacobian_command[2][0] = 1/WHEEL_DISTANCE
        motion_model_jacobian_command[2][1] = -1/WHEEL_DISTANCE

        # updata covariance matrix
        slice_row_indices = np.array([0, 1, 5])
        slice_colums_indices = np.array([0, 1, 5])
        previous_state_covariance = np.reshape(self.updated_state.pose.covariance, [6,6])[slice_row_indices,:][:, slice_colums_indices]
        # P_hat = F_x * P_t-1 * F_x_{T} + F_u * Q_t * F_u_{T}
        covariance_noise = np.zeros((2,2))
        covariance_noise[0][0] = K_r * abs(delta_s_right)
        covariance_noise[1][1] = K_l* abs(delta_s_left)
        covariance_motion = np.matmul(np.matmul(motion_model_jacobian_motion, previous_state_covariance), np.transpose(motion_model_jacobian_motion))
        rospy.logdebug(f"Covariance model\n{covariance_motion}")
        covariance_command = np.matmul(np.matmul(motion_model_jacobian_command, covariance_noise), np.transpose(motion_model_jacobian_command))
        current_covariance = np.add(covariance_motion, covariance_command)
        rospy.logdebug(f"Current covariance\n{current_covariance}")
        # covariance
        predicted_covariance = np.reshape(self.predicted_state.pose.covariance, [6,6])
        # xx - xy - xyaw
        predicted_covariance[0][0] = current_covariance[0][0]
        predicted_covariance[0][1] = current_covariance[0][1]
        predicted_covariance[0][5] = current_covariance[0][2]
        # yx - yy - yyaw
        predicted_covariance[1][0] = current_covariance[1][0]
        predicted_covariance[1][1] = current_covariance[1][1]
        predicted_covariance[1][5] = current_covariance[1][2]
        # yawx - yawy - yawyaw
        predicted_covariance[5][0] = current_covariance[2][0]
        predicted_covariance[5][1] = current_covariance[2][1]
        predicted_covariance[5][5] = current_covariance[2][2]
        self.predicted_state.pose.covariance = list(predicted_covariance.flat)
        rospy.logdebug(f"#####\nFinal predicted covariance:\n{self.predicted_state.pose.covariance}\n#####")

    def measure(self, line_segments_list: LineSegmentList) -> Tuple[np.array, np.array, np.array]:
        """Perform the measurement step of the Kalman filter
        Parameters
        ----------

        Returns
        -------
        """
        # 1. Get the measurement prediction
        z_hat_list, predicted_measures_jacobian_list = self._measurement_prediction()
        # 2. Get the lines interpolated through the Lidar
        #print(line_segments_msg)
        line_segmnets = line_segments_list.line_segments
        # 3. Compute the matching
        complete_innovation, measured_line_covariance_block_diagonal, complete_predicted_line_jacobians = self._matching(z_hat_list, predicted_measures_jacobian_list, line_segmnets)        
        rospy.logdebug(f"\nComplete innovation size:{np.shape(complete_innovation)}\n-\nMeasured line covariance block diagonal Size: {np.shape(measured_line_covariance_block_diagonal)}\n-\nComplete Predicted Line Jacobians: {np.shape(complete_predicted_line_jacobians)}")
        return complete_innovation, measured_line_covariance_block_diagonal, complete_predicted_line_jacobians
    
    def update(self, complete_innovation=np.array, measured_line_covariance_block_diagonal=np.array, complete_predicted_line_jacobians=np.array):
        """Perform the update step of the Kalman filter
        Parameters
        ----------

        Returns
        -------
        """
        if complete_innovation is not None:
            # 1. Compute Kalman Gain
            # P_hat * Complete_jacobian^T * Complete_Innovation_Covariance^-1
            # get the predicted state covariance
            predicted_state_covariance = self._get_predicted_state_covariance()
            complete_innovation_covariance = np.add(np.matmul(np.matmul(complete_predicted_line_jacobians, predicted_state_covariance), 
                                                                    complete_predicted_line_jacobians.transpose()), 
                                                    measured_line_covariance_block_diagonal)
            kalman_gain = np.matmul(np.matmul(predicted_state_covariance, complete_predicted_line_jacobians.transpose()),
                                    np.linalg.inv(complete_innovation_covariance))
            rospy.loginfo(f"Complete Innovation:\n{complete_innovation}")
            rospy.loginfo(f"Kalman Gain:\n{kalman_gain}")
            # 2. Compute update state
            # get the orientation of the robot
            # world frame and map frame are aligned
            # the yaw angle in map frame is the same in world frame
            theta_hat = self._get_yaw_angle_predicted_state()
            # compute updated state
            # updated_state = x_hat + K * v
            predicted_state = np.array([[self.predicted_state.pose.pose.position.x], [self.predicted_state.pose.pose.position.y], [theta_hat]])
            if np.count_nonzero(np.matmul(kalman_gain, complete_innovation) > 1):
                rospy.logdebug(f"Predicted state covariance {predicted_state_covariance}")
                rospy.logdebug(f"Inverse of innovation matrix {np.linalg.inv(complete_innovation_covariance)}")
                rospy.logdebug(f"Kalman gain: {np.matmul(kalman_gain, complete_innovation)}")
            updated_state = np.add(predicted_state, np.matmul(kalman_gain, complete_innovation))
            rospy.loginfo(f"Add element\n{np.matmul(kalman_gain, complete_innovation)}")
            rospy.loginfo(f"Updated state:\n{updated_state}")
            # updated_covariance = P_hat - K*Innovation_covariance*K^T
            updated_state_covariance = np.subtract(predicted_state_covariance, 
                                                   np.matmul(np.matmul(kalman_gain, complete_innovation_covariance), kalman_gain.transpose()))
            rospy.logdebug(f"Updated State Covariance:\n {updated_state_covariance}")
            #### Update State
            # position
            self.updated_state.pose.pose.position.x = updated_state[0][0]
            self.updated_state.pose.pose.position.y = updated_state[1][0]
            # orientation
            new_quaternion = tf.transformations.quaternion_about_axis(updated_state[2][0], (0, 0, 1))
            self.updated_state.pose.pose.orientation.x = new_quaternion[0] 
            self.updated_state.pose.pose.orientation.y = new_quaternion[1]
            self.updated_state.pose.pose.orientation.z = new_quaternion[2]
            self.updated_state.pose.pose.orientation.w = new_quaternion[3]
            # covariance
            updated_covariance = np.reshape(self.updated_state.pose.covariance, [6,6])
            # xx - xy - xyaw
            updated_covariance[0][0] = updated_state_covariance[0][0]
            updated_covariance[0][1] = updated_state_covariance[0][1]
            updated_covariance[0][5] = updated_state_covariance[0][2]
            # yx - yy - yyaw
            updated_covariance[1][0] = updated_state_covariance[1][0]
            updated_covariance[1][1] = updated_state_covariance[1][1]
            updated_covariance[1][5] = updated_state_covariance[1][2]
            # yawx - yawy - yawyaw
            updated_covariance[5][0] = updated_state_covariance[2][0]
            updated_covariance[5][1] = updated_state_covariance[2][1]
            updated_covariance[5][5] = updated_state_covariance[2][2]
            self.updated_state.pose.covariance = list(updated_covariance.flat)
        else:
            self.updated_state = copy.deepcopy(self.predicted_state)

    def get_last_state(self) -> Odometry:
        return self.updated_state

def time_sync_callback(joint_state: JointState, line_segment_list: LineSegmentList):
    
    global callback_cnt
    ekf.prediction(joint_state)
    if (callback_cnt%30 == 0):
        rospy.loginfo(f"Predicted Position\n {ekf.predicted_state.pose.pose.position}")
        rospy.loginfo(f"Predicted Orientation\n {ekf.predicted_state.pose.pose.orientation}")
    
    #if (callback_cnt%10 == 0):
    complete_innovation, measured_line_covariance_block_diagonal, complete_predicted_line_jacobians = ekf.measure(line_segment_list) 
    ekf.update(complete_innovation, measured_line_covariance_block_diagonal, complete_predicted_line_jacobians)
    if (callback_cnt%30 == 0):
        rospy.loginfo(f"Updated Position\n{ekf.updated_state.pose.pose.position}")
        rospy.loginfo(f"Updated Orientation\n{ekf.updated_state.pose.pose.orientation}")
    #else:
    #    ekf.updated_state = copy.deepcopy(ekf.predicted_state)

    callback_cnt += 1
    
    odom_ekf_pub.publish(ekf.get_last_state())

    if compute_loss:
        # get GT odometry
        odom_msg = rospy.wait_for_message("/odom", Odometry)
        new_odometry = ekf.get_last_state()
        # Get the ground truth pose
        gt_pose = odom_msg.pose
        # Get updated pose
        odom_ekf_pub.publish(ekf.get_last_state())
        updated_pose = ekf.get_last_state().pose 
        error_x = Float64(gt_pose.pose.position.x - updated_pose.pose.position.x)
        error_y = Float64(gt_pose.pose.position.y - updated_pose.pose.position.y)
        # get GT quaternion
        # Robot orientation and position estimated through odometry
        _, _, yaw_gt = tf.transformations.euler_from_quaternion(quaternion=[gt_pose.pose.orientation.x,
                                                                            gt_pose.pose.orientation.y,
                                                                            gt_pose.pose.orientation.z,
                                                                            gt_pose.pose.orientation.w])

        _, _, yaw_updated_pose = tf.transformations.euler_from_quaternion(quaternion=[updated_pose.pose.orientation.x,
                                                                                    updated_pose.pose.orientation.y,
                                                                                    updated_pose.pose.orientation.z,
                                                                                    updated_pose.pose.orientation.w]) 
        error_yaw = Float64(yaw_gt - yaw_updated_pose)                     

        bag.write('error_x', error_x)
        bag.write('error_y', error_y)
        bag.write('error_yaw', error_yaw)
        bag.write('odometry_ekf', ekf.get_last_state())
        bag.write('odometry_gt', odom_msg)
        if (callback_cnt%30 == 0):
            rospy.loginfo(f"Error x {error_x}")
            rospy.loginfo(f"Error y {error_y}")
            rospy.loginfo(f"Error yaw {error_yaw}")


if __name__ == '__main__':
    
    rospy.init_node("ekf_test")
    odom_ekf_pub = rospy.Publisher("/odom_ekf", Odometry, queue_size=1)
    joint_state_sub = message_filters.Subscriber('/joint_states', JointState)
    line_extractor_sub = message_filters.Subscriber('/line_segments', LineSegmentList)

    rate = rospy.Rate(30)
    initial_pose = Pose()
    initial_pose.position.x = -24.0
    initial_pose.position.y = -9.5
    initial_pose.position.z = .0
    initial_pose.orientation.x = .0
    initial_pose.orientation.y = .0
    initial_pose.orientation.z = 0.0
    initial_pose.orientation.w = 0.1
    ekf = KalmanFilter(initial_pose=initial_pose, map_lines_file_path=map_lines_file_path)

    bag = None
    if compute_loss:
        bag_file = os.path.join(bag_file_dir, "error.bag")
        bag = rosbag.Bag(bag_file, 'w')
    
    ts = message_filters.TimeSynchronizer([joint_state_sub, line_extractor_sub], 10)
    ts.registerCallback(time_sync_callback)
        
    rospy.spin()

    if compute_loss:
        bag.close()