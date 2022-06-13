from re import L
from typing import List, Tuple
import rospy, rospkg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import tf

from laser_line_extraction.msg import LineSegment, LineSegmentList 

import numpy as np
import copy
import math
import os, sys

# set the mean and std. var for guassian noise on linear motion model
MEAN = 0.0 # m
STD_DEV = 0.2 # m 
CLIP_ON_VARIATION_MOTION_MODEL = 0.4 # correspond to the 95-th percentile

# Robot parameters
WHEEL_DISTANCE = 0.287 # m
WHEEL_RADIUS = 0.033 # m
LIDAR_MAX_RANGE = 3.5 # m 

# get the path to the current package
rospack = rospkg.RosPack()
pkg_path = rospack.get_path('exam')
map_lines_file_path= os.path.join(pkg_path, "config/map_lines.npy")

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

        self.validation_gate = 5.99 # corresponding to the 95-th percentile of a chi^2 distribution with 2 d.o.f

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
                    state.pose.covariance[diagonal_index] = 1000000000000.0 # since the robot is planar and the z term is not estimated
                else:
                    state.pose.covariance[diagonal_index] = 10**-5 # the initial pose is supposed known 
            else:
                if i == 3 or i == 4:
                    state.pose.covariance[diagonal_index] = 1000000000000.0 # since the robot is planar and the roll and pitch are not estimated
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

    def prediction(self):
        """Perform the preditction step of the Kalman filter
        Parameters
        ----------
            cmd_vel: list()
                The command that has been sent to the robot. Linear and angular velocity
        Returns
        -------
        """

        # compute delta_s
        self.current_right_wheel, self.current_left_wheel = self._read_wheel_joints()
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
        motion_model_jacobian_motion = np.identity(6)
        motion_model_jacobian_motion[0][5] = -delta_s*math.sin(theta + delta_theta/2)
        motion_model_jacobian_motion[1][5] = delta_s*math.cos(theta + delta_theta/2)
        # compute the jacobian of the motion model with respect to the command
        motion_model_jacobian_command = np.zeros((6,2))
        motion_model_jacobian_command[0][0] = 1/2*math.cos(theta + delta_theta/2) - (delta_s/(2*WHEEL_DISTANCE))*math.sin(theta + delta_theta/2)
        motion_model_jacobian_command[0][1] = 1/2*math.cos(theta + delta_theta/2) + (delta_s/(2*WHEEL_DISTANCE))*math.sin(theta + delta_theta/2)
        motion_model_jacobian_command[1][0] = 1/2*math.sin(theta + delta_theta/2) + (delta_s/(2*WHEEL_DISTANCE))*math.cos(theta + delta_theta/2)
        motion_model_jacobian_command[1][1] = 1/2*math.sin(theta + delta_theta/2) - (delta_s/(2*WHEEL_DISTANCE))*math.cos(theta + delta_theta/2)
        motion_model_jacobian_command[5][0] = 1/WHEEL_DISTANCE
        motion_model_jacobian_command[5][1] = -1/WHEEL_DISTANCE

        # updata covariance matrix
        previous_state_covariance = np.reshape(np.array(self.updated_state.pose.covariance), (6,6))
        # P_hat = F_x * P_t-1 * F_x_{T} + F_u * Q_t * F_u_{T}
        covariance_noise = np.zeros((2,2))
        covariance_noise[0][0] = abs(delta_s_right)
        covariance_noise[1][1] = abs(delta_s_left)
        covariance_motion = np.matmul(np.matmul(motion_model_jacobian_motion, previous_state_covariance), np.transpose(motion_model_jacobian_motion))
        covariance_command = np.matmul(np.matmul(motion_model_jacobian_command, covariance_noise), np.transpose(motion_model_jacobian_command))
        current_covariance = np.add(covariance_motion, covariance_command)        
        self.predicted_state.pose.covariance = list(np.array(current_covariance).flatten())
            
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
        # Robot orientation and position estimated through odometry
        _, _, theta_hat = tf.transformations.euler_from_quaternion(quaternion=[self.predicted_state.pose.pose.orientation.x,
                                                                          self.predicted_state.pose.pose.orientation.y,
                                                                          self.predicted_state.pose.pose.orientation.z,
                                                                          self.predicted_state.pose.pose.orientation.w])
        x_hat = self.predicted_state.pose.pose.position.x
        y_hat = self.predicted_state.pose.pose.position.y

        for line in self.map_lines:
            # rho_hat = rho - (x_hat*cos(alpha) + y_hat*sinc(alpha))
            rho_hat = line[0] - (x_hat*math.cos(line[1]) + y_hat*math.sin(line[1]))
            if rho_hat > LIDAR_MAX_RANGE:
                continue
            else:
                # alpha_hat = alpha - theta_hat 
                alpha_hat = line[1] - theta_hat
                z_hat.append([rho_hat, alpha_hat])
                predicted_measures_jacobians.append([[0, 0, -1], [-math.cos(line[1]), -math.sin(line[1]), 0]])
                
        
        rospy.logdebug(f"Number of expected lines {len(z_hat)}")
        rospy.logdebug(f"Expected Lines {z_hat}")
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
        for line_segment_msg in line_segments:
            # get the measured line
            # [angle
            #  radius]
            measured_line = np.array([[line_segment_msg.angle], [line_segment_msg.radius]])
            measured_line_covariance = np.reshape(line_segment_msg.covariance, [2,2])
            rospy.logdebug(f"Measured lines {measured_line}")
            for indx, predicted_measure in enumerate(z_hat):
                # [angle
                #  radius]
                predicted_line = np.array([[predicted_measure[1]], [predicted_measure[0]]])
                predicted_line_jacobian = np.array(predicted_measures_jacobian_list[indx])
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
        
        return complete_innovation, measured_line_covariance_block_diagonal, complete_predicted_line_jacobians


    def measure(self) -> Tuple[np.array, np.array, np.array]:
        """Perform the measurement step of the Kalman filter
        Parameters
        ----------

        Returns
        -------
        """
        # 1. Get the measurement prediction
        z_hat_list, predicted_measures_jacobian_list = self._measurement_prediction()
        # 2. Get the lines interpolated through the Lidar
        line_segments_msg = rospy.wait_for_message("/line_segments", LineSegmentList)
        line_segmnets_list = line_segments_msg.line_segments
        # 3. Compute the matching
        complete_innovation, measured_line_covariance_block_diagonal, complete_predicted_line_jacobians = self._matching(z_hat_list, predicted_measures_jacobian_list, line_segmnets_list)
        rospy.logdebug(f"\nComplete innovation size: {np.shape(complete_innovation)}\n-\nMeasured line covariance block diagonal Size: {np.shape(measured_line_covariance_block_diagonal)}\n-\nComplete Predicted Line Jacobians: {np.shape(complete_predicted_line_jacobians)}")

        return complete_innovation, measured_line_covariance_block_diagonal, complete_predicted_line_jacobians
    
    def update(self, complete_innovation=np.array, measured_line_covariance_block_diagonal=np.array, complete_predicted_line_jacobians=np.array):
        """Perform the update step of the Kalman filter
        Parameters
        ----------

        Returns
        -------
        """
        # 1. Compute Kalman Gain
        # P_hat * Complete_jacobian^T * Complete_Innovation_Covariance^-1
        # get the predicted state covariance
        predicted_state_covariance = self._get_predicted_state_covariance()
        complete_innovation_covariance = np.add(np.matmul(np.matmul(complete_predicted_line_jacobians, predicted_state_covariance), 
                                                                   complete_predicted_line_jacobians.transpose()), 
                                                measured_line_covariance_block_diagonal)
        kalman_gain = np.matmul(np.matmul(predicted_state_covariance, complete_predicted_line_jacobians.transpose()),
                                np.linalg.inv(complete_innovation_covariance))
        rospy.loginfo(f"Kalman Gain shape {np.shape(kalman_gain)}")
        
        # 2. Compute update position

        # 3. Compute update covariance matrix

        self.updated_state = copy.deepcopy(self.predicted_state)
        
if __name__ == '__main__':
    rospy.init_node("ekf_test")
    rate = rospy.Rate(30)
    initial_pose = Pose()
    initial_pose.position.x = -24.0
    initial_pose.position.y = -9.5
    initial_pose.position.z = .0
    initial_pose.orientation.x = .0
    initial_pose.orientation.y = .0
    initial_pose.orientation.z = .0
    initial_pose.orientation.w = 1
    ekf = KalmanFilter(initial_pose=initial_pose, map_lines_file_path=map_lines_file_path)
    
    i = 0
    while not rospy.is_shutdown():
        ekf.prediction()
        if (i%30 == 0):
            rospy.loginfo(f"Predicted Position\n {ekf.predicted_state.pose.pose.position}")
            rospy.loginfo(f"Predicted Orientation\n {ekf.predicted_state.pose.pose.orientation}")
        complete_innovation, measured_line_covariance_block_diagonal, complete_predicted_line_jacobians = ekf.measure()
        ekf.update(complete_innovation, measured_line_covariance_block_diagonal, complete_predicted_line_jacobians)
        rate.sleep()
        i += 1    