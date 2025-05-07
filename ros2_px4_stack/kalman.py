import rclpy
from rclpy.node import Node 
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped 
import numpy as np 
import matplotlib.pyplot as plt 
from scipy.linalg import solve_discrete_are 

class KalmanFilter(Node):
    def __init__(self):
        """
        Initialize Kalman Filter parameters here such as 
        dynamics and covariance matrices. 
        """
        super().__init__("kalman_filter")

        # Define QoS Policy 
        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = DurabilityPolicy.VOLATILE
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

        # TODO: Ideally make a custom message for state that includes position and velocity without unused att/rate fields.
        # Declare publishers. 
        self.pos_est_topic = "pos_est"
        self.vel_est_topic = "vel_est" 
        self.pos_est_pub = self.create_publisher(PoseStamped, self.pos_est_topic, qos_profile)
        self.vel_est_pub = self.create_publisher(TwistStamped, self.vel_est_topic, qos_profile) 
        
        # Declare subscription 
        self.pose_topic = "mavros/local_position/pose" 
        self.pose_cb = self.create_subscription(PoseStamped, self.pose_topic, self.pose_cb, qos_profile) #TODO: QoS policy

        # Filter dynamics (double integrator):  
        self.sampling_rate = 30 # Hz, mavros/local_position/pose publishing rate 
        self.dt = 1 / self.sampling_rate 

        # TODO: Cleanly write this using np.eye(), np.hstack(), and np.vstack()
        self.A_kf = np.array([[1, 0, 0, self.dt,       0,       0], 
                              [0, 1, 0,       0, self.dt,       0],
                              [0, 0, 1,       0,       0, self.dt],
                              [0, 0, 0,       1,       0,       0],
                              [0, 0, 0,       0,       1,       0],
                              [0, 0, 0,       0,       0,       1]])

        self.G = np.array([[0.5 * self.dt ** 2,                  0,                  0], 
                           [                 0, 0.5 * self.dt ** 2,                  0],
                           [                 0,                  0, 0.5 * self.dt ** 2],
                           [           self.dt,                  0,                  0],
                           [                 0,            self.dt,                  0],
                           [                 0,                  0,            self.dt]]) 

        self.C = np.array([[1, 0, 0, 0, 0, 0],
                           [0, 1, 0, 0, 0, 0],
                           [0, 0, 1, 0, 0, 0]])

        # Size of state space
        self.n = 6
        
        # Variances and covariances of process and measurement noise
        self.w = 1e-2 # TODO: Investigate whether uncertainty along z-axis should be less.
        self.v = 1e-1
        self.W = self.w * np.eye(self.n // 2)
        self.V = self.v * np.eye(self.n // 2)
        self.W_eff = self.G @ self.W @ self.G.T

        # Prediction variables 
        self.x_hat = np.zeros((self.n, 1))
        self.y_hat = np.zeros((self.n // 2, 1))

        # Riccati variable, Q, and Kalman Gain, L 
        self.Q = solve_discrete_are(self.A_kf.T, self.C.T, self.W_eff, self.V)
        self.L = self.Q @ self.C.T @ np.linalg.inv(self.C @ self.Q @ self.C.T + self.V)
        

    def kalman_filter(self, y_mes_noised):
        """
        Vanilla Kalman Filter  

        Inputs: 
            - Noisy quad position measurement, nd.array of shape (3, 1)
        Outputs:
            - None, it updates the state estimate. 
        """

        # Compute state estimate from k - 1 measurements 
        self.x_hat = self.A_kf @ self.x_hat

        # Compute prediction of y at k given k - 1 measurements 
        self.y_hat = self.C @ self.x_hat

        # Compute correction between predicted and measured y 
        y_correction = y_mes_noised - self.y_hat

        # Compute measurement update (correct x_hat) 
        self.x_hat = self.x_hat + self.L @ y_correction  

    def simulate_lidar(self, y_mes):
        """
        Add noise to the simulation measurement in order to simulate the lidar's precision 

        Input:
            - Position measurement from the simulation, nd.array of size (3, 1).
        Output: 
            - Position measurement from simulation with noise, nd.array of size (3, 1).  
        """

        # All you need is to add noise to the measurement obtained from the simulation 
        return y_mes + np.random.normal(loc=0.0, scale=np.sqrt(self.v), size=(self.n // 2, 1))

    def from_rosmsg_to_numpy(self, pose_msg): 
        """
        Function to extract state estimates from ros messages.

        Inputs: 
            - Quad pose at current time, rclpy.PoseStamped object.
        Outputs: 
            -  The position of the quat, measurement, as nd.array of shape (3, 1).
        """

        pos = pose_msg.pose.position
        return np.array([[pos.x, pos.y, pos.z]]).T

    def from_numpy_to_rosmsg(self):
        """
        Takes the state estimate and populates a PoseStamped message and a TwistStamped message with 
        the estimate information.  
        
        Inputs:
            - None, it uses the state estimate class variable. 
        Outputs: 
            - Position estimate message, rclpy.PoseStamped type.
            - Velocity estimate message, rclpy.TwistStamped type. 
        """ 
        # Populate PoseStamped msg
        pose_msg = PoseStamped() 
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "world"
        pose_msg.pose.position.x = self.x_hat[0, 0]
        pose_msg.pose.position.y = self.x_hat[1, 0]
        pose_msg.pose.position.z = self.x_hat[2, 0]
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0

        # Populate TwistStamped msg
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "world"
        twist_msg.twist.linear.x = self.x_hat[3, 0]
        twist_msg.twist.linear.y = self.x_hat[4, 0]
        twist_msg.twist.linear.z = self.x_hat[5, 0]
        twist_msg.twist.angular.x = 0.0
        twist_msg.twist.angular.y = 0.0
        twist_msg.twist.angular.z = 0.0
        
        return pose_msg, twist_msg 

    def pose_cb(self, pose_msg):
        """
        Pose callback from the /SQ01/mavros/local_position/pose topic 
        """

        # Convert pose msgs to numpy array 
        pos = self.from_rosmsg_to_numpy(pose_msg)

        # Noise up msg 
        pos_noisy = self.simulate_lidar(pos) 

        # Run Kalman Filter 
        self.kalman_filter(pos_noisy) 

        # Populate pos and vel estimates
        pos_est, vel_est = self.from_numpy_to_rosmsg()

        # Publish state estimate 
        self.pos_est_pub.publish(pos_est)
        self.vel_est_pub.publish(vel_est) 

def main(args=None):
    rclpy.init(args=args)

    kf_node = KalmanFilter()

    rclpy.spin(kf_node)
    
    kf_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()