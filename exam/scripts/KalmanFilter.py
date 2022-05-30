import cmd
import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import tf

import numpy as np
import copy
class KalmanFilter:
    
    def __init__(self, initial_pose:Pose):
        self.predicted_state = Odometry()
        self._initialize_state(self.predicted_state, initial_pose=initial_pose)
        self.last_time = rospy.Time.now()

        # initialize the updated state to the current initial state
        self.updated_state = copy.deepcopy(self.predicted_state)        

    def prediction(self, cmd_vel:list()):
        """Perform the preditction step of the Kalman filter
        Parameters
        ----------
            cmd_vel: list()
                The command that has been sent to the robot. Linear and angular velocity
        Returns
        -------
        """

        # get yaw angle
        orientation = [self.updated_state.pose.pose.orientation.x, self.updated_state.pose.pose.orientation.y, 
                       self.updated_state.pose.pose.orientation.z, self.updated_state.pose.pose.orientation.w]
        _, _, theta = tf.transformations.euler_from_quaternion(orientation)

        # compute delta_t
        current_time = rospy.Time.now()
        delta_t = (current_time - self.last_time).to_sec()

        # compute delta_x, delta_y, delta_theta
        delta_x = ((-cmd_vel[0]/cmd_vel[1] * math.sin(theta)) + (cmd_vel[0]/cmd_vel[1] * math.sin(theta + cmd_vel[1]*delta_t))) # -v_t/w_t*sin(theta) + v_t/w_t*sin(theta+w_t*delta_t)
        delta_y = ((cmd_vel[0]/cmd_vel[1] * math.cos(theta)) - (cmd_vel[0]/cmd_vel[1] * math.cos(theta + cmd_vel[1]*delta_t))) # +v_t/w_t*cos(theta) - v_t/w_t*cos(theta+w_t*delta_t)
        delta_theta = cmd_vel[1] * delta_t

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
        self.predicted_state.pose.pose.orientation.z = new_quaternion[1]
        self.predicted_state.pose.pose.orientation.y = new_quaternion[2]
        self.predicted_state.pose.pose.orientation.w = new_quaternion[3]
        #---- POSE COVARIANCE ----#
        

    def measure():
        pass

    def update():
        pass


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