import cmd
import math
from tkinter import LEFT
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import tf

import numpy as np
import copy

# set the mean and std. var for guassian noise on linear motion model
MEAN = 0.0 # m
STD_DEV = 0.2 # m 
CLIP_ON_VARIATION_MOTION_MODEL = 0.4 # correspond to the 95-th percentile

#
WHEEL_DISTANCE = 0.287 # m
WHEEL_RADIUS = 0.033 # m


class KalmanFilter:
    
    def __init__(self, initial_pose:Pose):
        self.predicted_state = Odometry()
        self._initialize_state(self.predicted_state, initial_pose=initial_pose)
        self.last_time = rospy.Time.now()

        # initialize the updated state to the current initial state
        self.updated_state = copy.deepcopy(self.predicted_state)        

        # initialize the joint distance attraversed by the left and right wheel
        self.previous_right_wheel, self.previous_left_wheel = self._read_wheel_joints()
        self.current_right_wheel = self.previous_right_wheel 
        self.current_left_wheel = self.previous_left_wheel

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
            
    def measure():
        pass

    def update(self):
        """Perform the update step of the Kalman filter
        Parameters
        ----------

        Returns
        -------
        """
        self.updated_state = copy.deepcopy(self.predicted_state)
        
if __name__ == '__main__':
    rospy.init_node("ekf_test")
    rate = rospy.Rate(30)
    initial_pose = Pose()
    initial_pose.position.x = -2.0
    initial_pose.position.y = -0.5
    initial_pose.position.z = .0
    initial_pose.orientation.x = .0
    initial_pose.orientation.y = .0
    initial_pose.orientation.z = .0
    initial_pose.orientation.w = 1
    ekf = KalmanFilter(initial_pose=initial_pose)
    
    i = 0
    while not rospy.is_shutdown():
        ekf.prediction()
        if (i%30 == 0):
            rospy.loginfo(f"Predicted Position\n {ekf.predicted_state.pose.pose.position}")
            rospy.loginfo(f"Predicted Orientation\n {ekf.predicted_state.pose.pose.orientation}")
        ekf.update()
        rate.sleep()
        i += 1    