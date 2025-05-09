import rclpy
from rclpy.node import Node 
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped 
import numpy as np 
import matplotlib.pyplot as plt 
from scipy.linalg import solve_discrete_are 
from scipy.spatial.transform import Rotation 
import copy 


# Quadcopter class for dynamics 
class Quad():
    def __init__(self, x0, dt, m=2.5, d=0.35):
        """
        Constructor for Quadcopter

        Inputs: 
            - x0, true state initial state, (13, 1) nd.array 
            - dt, discretized time interval, 30 for gazebo sim, float. 
            - Quad mass, about 2.5 Kg for x500 model, float.
            - Quad arm length, about 0.35 m for x500 model, float.
        """
        self.dt = dt
        self.m = m 
        self.d = d
        self.g = 9.81 # m/s^2
        self.I = np.eye(3)
        self.I[0, 0] = m * (d ** 2)
        self.I[1, 1] = m * (d ** 2) 
        self.I[2, 2] = 0.5 * m * (d ** 2)
        self.L = x0.shape[0]

        self.x = copy.deepcopy(x0) 

        # state and control history (h for history not hat)
        self.xh = [copy.deepcopy(x0)]
        self.uh = [] 

        # Measurement noise for position measurement (simulate lidar) 
        self.R = np.diag([1e-1, 1e-1, 1e-1]) #TODO Try experimenting with this value. simpler sim worked with 1e-2

    def W(self, u, alphas=[0.1, 0.01, 0.01, 0.1, 0.1, 0.01, 0.01, 0.1]):
        """
        Process noise. 
        You could implement a model that adds more noise the faster 
        the vehicle is thrusting and the higher the moments, but for now
        let's just make it a fixed variance for each.

        Here thrust and moments have the same effect on each other's uncertainties
        (that's what the noise characteristics, alpha, are). But you could modify this 
        to something that makes more sense. 
        Perhaps thrust should be noisier if yaw is high.  
        """

        # This adds noise of scale 10^2 which is way off
        # var1 = alphas[0] * u[0] ** 2 + alphas[1] * u[1] ** 2 + alphas[2] * u[2] ** 2 + alphas[3] * u[3] ** 2
        # var2 = alphas[4] * u[0] ** 2 + alphas[5] * u[1] ** 2 + alphas[6] * u[2] ** 2 + alphas[7] * u[3] ** 2

        # If hover thrust is about 2.5 * 9.81 ~ 25 N, then var1 ~ 0.1
        # If moments are about m * d ** 2 * alpha ~ 2.5 * 0.111 * 0.75 ~ 0.2, then var2 ~ 0.01
        var1 = 0.1
        var2 = 0.01
        cov = np.eye(4)
        cov[0, 0] = var1
        cov[1, 1] = var2 
        cov[2, 2] = var2
        cov[3, 3] = var2

        return cov

    def f(self, xk, uk, dt):
        """
        Dynamics model for the drone. 
        Propagates state from k to k + 1 

        Inputs: 
            - State, x, at k, nd.array(13, 1) 
            - Control law, u, at k, total thrust and moment along each euler angle, nd.array (4, 1)
            - dt, discrete timestep 
        Output: 
            - xkp1, propagated state
        """

        # Extract state
        pk = xk[:3, :]
        vk = xk[3:6, :]
        qk = xk[6:10, :]
        omk = xk[10:13, :]

        # Check for zero norm quaternions 
        if np.linalg.norm(qk) < 1e-6:
            print("WARNING: qk has near-zero norm!", qk.T)

        # Extract inputs
        Tk = uk[0, :].reshape(1, 1)
        tauk = uk[1:, :]

        # Compute drone's acceleration at k 
        e3 = np.array([[0, 0, -1]]).T
        zb = np.array([[0, 0, 1]]).T
        
        q_roll = np.roll(qk.flatten(), -1)
        q_roll /= np.linalg.norm(q_roll) # Normalize and roll quaternion (avoid zero norm, and change to scalar last format)
        Rk = Rotation.from_quat(q_roll).as_matrix()
        ak = self.g * e3 + Rk @ zb * Tk / self.m 

        # Propagate translational dynamics 
        pkp1 = pk + vk * dt 
        vkp1 = vk + ak * dt 

        # Compute skew-symmetric omega matrix 
        wx, wy, wz = omk[0, 0], omk[1, 0], omk[2, 0]
        Omk = np.array([[ 0, -wx, -wy, -wz], # This choice of skey symmetric matrix requires scalar first quaternion convention 
                        [wx,   0,  wz, -wy],
                        [wy, -wz,   0,  wx],
                        [wz,  wy,  -wx,  0]])

        # Propagate rotational dynamics 
        # qkp1 = qk + 0.5 * Omk @ qk * dt # NOTE: scipy method is more stable than this
        rot = Rotation.from_quat(np.roll(qk.flatten(), -1)) 
        delta_rot = Rotation.from_rotvec(omk.flatten() * dt)        
        new_rot = delta_rot * rot
        qkp1 = np.roll(new_rot.as_quat(), 1).reshape(4, 1)   
        qkp1 = qkp1 / np.linalg.norm(qkp1) 
        omega = omk.reshape(3,) # Required for cross product
        omkp1 = omk + np.linalg.inv(self.I) @ (tauk + np.cross(omega, self.I @ omega).reshape(3, 1)) * dt

        # Construct propagated state vector 
        xkp1 = np.vstack((pkp1, vkp1, qkp1, omkp1))

        return xkp1 

    def h(self, xk):
        """
        Measurement model for lidar sensor 

        Inputs: 
            - xk, current state
        Outputs: 
            - Current position, nd.array (3, 2L + 1)
        """
        return xk[:3, :]

    def move(self, uk):
        """
        Simple physics simulation of the quadcopter 

        Inputs: 
            - uk, the control law at timestep k, nd.array (4, 1) 
        """

        # Add process noise to the desired commands 
        uk_noisy = np.random.multivariate_normal(uk[:, 0], self.W(uk)).reshape(4, 1)

        # Propagate state forward
        self.x = self.f(self.x.reshape(self.L, 1), uk_noisy, self.dt)

        # Keep history 
        self.uh.append(copy.deepcopy(uk_noisy))
        self.xh.append(copy.deepcopy(self.x))

    def sense(self):
        """
        Simulate lidar sensor reading by adding noise to measurement

        Inputs: 
            - None, uses class variable self.x

        Outputs: 
            - noised up measurement
        """

        # Get measurement from measurement model 
        pk_mes = self.h(self.x)

        # Add noise to the reading  
        return np.random.multivariate_normal(pk_mes[:, 0], self.R).reshape(3, 1)


# UKF Class 
class UKF(Node):
    def __init__(self, quad, x0, Q0, alpha=1e-3, beta=2, kappa=0):
        """
            Unscented Kalman Filter 
            
            Inputs:
                - quad model 
                - x0, initial state
                - Q0, initial covariance 
                - alpha, determines spread of sigma points distribution around mean state, usually 1e-3.
                - beta, incorporates prior knowledge to distribution, optimally 2 for Gaussians. 
                - Kappa, secondary scaling parameter, usually zero. 
        """
        ### ROS Stuff ########################
        # Inherit Node class
        super().__init__("ukf")

        # Define QoS Policy 
        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = DurabilityPolicy.VOLATILE
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT


        # Publishers and subscriptions 
        self.pos_est_topic = "pos_est"
        self.vel_est_topic = "vel_est" 
        self.pos_est_pub = self.create_publisher(PoseStamped, self.pos_est_topic, qos_profile)
        self.vel_est_pub = self.create_publisher(TwistStamped, self.vel_est_topic, qos_profile) 
        
        # Declare subscription 
        self.pose_topic = "mavros/local_position/pose" 
        self.pose_cb = self.create_subscription(PoseStamped, self.pose_topic, self.pose_cb, qos_profile) 


        ######################################

        # UT Params 
        self.kappa = kappa
        self.L = x0.shape[0]
        self.alpha = alpha 
        self.beta = beta 
        self.lamb = self.alpha ** 2 * (self.L + self.kappa) - self.L 

        # Initialize filter state and covariance 
        self.xhat = copy.deepcopy(x0)
        self.Q = copy.deepcopy(Q0)

        # Initialize filter state and covariance history 
        self.xhath = [copy.deepcopy(x0)]
        self.Qh = [copy.deepcopy(Q0)]

        # Save quad model instance as a class variable for dynamics 
        self.quad = quad

        # Dynamics and measurement models (assume we have perfect model knowledge) 
        self.f = quad.f 
        self.h = quad.h

        # Model noise as either a function of control or just even noise 
        even_noise = True 
        if even_noise: 
            self.W = np.diag([1e-6, 1e-6, 1e-6, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-9, 1e-4, 1e-4, 1e-9]) # vel 1.0 Best correct orientation
            # self.W = np.diag([1e-4, 1e-4, 1e-3, 1e-1, 1e-1, 1e-1, 1e-3, 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2])  # vel 3.0
            # self.W = np.diag([1e-6, 1e-6, 1e-6, 1e-3, 1e-3, 1e-3, 1e-7, 1e-7, 1e-7, 1e-9, 1e-7, 1e-7, 1e-9]) # vel 1.0 Perfect inverse orientation 
            self.R = quad.R
        else:
            self.W = quad.W 
            self.R = quad.R 

    def filter_UT(self, y_mes, ubar):
        """ 
            Apply Unscented Kalman Filter 

            Inputs: 
                - y_mes, measurement from quad simulation, nd.array(3, 1)
                - ubar, mean input control law since we don't have the actual control inputs, nd.array(4, 1)
        """

        # Compute weights 
        Wm = np.zeros((2 * self.L + 1, 1))
        Wm[0, :] = self.lamb / (self.L + self.lamb)
        Wm[1:, :] = 1 / (2 * (self.L + self.lamb)) 

        Wc = np.zeros((2 * self.L + 1, 1))
        Wc[0, :] = self.lamb / (self.L + self.lamb) + (1 - self.alpha ** 2 + self.beta) 
        Wc[1:, :] = Wm[1:, :] 
        
        # Compute sigma points 
        self.Q = self.project_to_psd(self.Q) # stabilize
        sqrtQ = np.linalg.cholesky((self.L + self.lamb) * self.Q + 1e-6 * np.eye(self.L))
        Xk = np.zeros((2 * self.L + 1, self.L))
        Xk[0, :] = self.xhat[:, 0]
        Xk[1:self.L+1, :] = self.xhat[:, 0] + sqrtQ.T
        Xk[self.L+1:, :] = self.xhat[:, 0] - sqrtQ.T

        # Push sigma points through dynamics and recover state at k+1
        Xkp1 = np.array([self.f(Xk[i, :].reshape(self.L, 1), ubar, self.quad.dt).flatten() for i in range(2 * self.L + 1)]).reshape((2 * self.L + 1, self.L))
        Quat = Xkp1[:, 6:10]                 
        norms = np.linalg.norm(Quat, axis=1, keepdims=True) # Normalize quaternion sigma points along the manifold to prevent spikes  
        # norms[norms < 1e-6] = 1.0
        Xkp1[:, 6:10] = Quat / norms          
        self.xhat = np.sum(Wm * Xkp1, axis=0).reshape(-1, 1)

        # Recover state covariance at k_1 
        self.Q = (Wc * (Xkp1 - self.xhat.T)). T @ (Xkp1 - self.xhat.T) + self.W 

        # Recompute sigma points for measurement
        self.Q = self.project_to_psd(self.Q) # stabilize Q 
        sqrtQ = np.linalg.cholesky((self.L + self.lamb) * self.Q + 1e-6 * np.eye(self.L))
        Xk = np.zeros((2 * self.L + 1, self.L))
        Xk[0, :] = self.xhat[:, 0]
        Xk[1:self.L+1, :] = self.xhat[:, 0] + sqrtQ.T
        Xk[self.L+1:, :] = self.xhat[:, 0] - sqrtQ.T

        # Push sigma points through measurement model and recover mean at k+1
        Ykp1 = self.h(Xk.T).T 
        ykp1 = np.sum(Wm * Ykp1, axis=0).reshape(-1, 1)

        # Recover y covariance at k+1
        Qy_kp1 = (Wc * (Ykp1 - ykp1.T)).T @ (Ykp1 - ykp1.T) + self.R

        # Compute xy cross covariance at k+1 
        Qxy_kp1 =  (Wc * (Xkp1 - self.xhat.T)).T @ (Ykp1 - ykp1.T) 

        # Compute Kalman gain and perform measurement update 
        Kappa = Qxy_kp1 @ np.linalg.inv(Qy_kp1) 
        self.xhat += Kappa @ (y_mes - ykp1)
        self.xhat[6:10, :] = self.xhat[6:10, :] / np.linalg.norm(self.xhat[6:10, :]) # Normalize predicted quaternion to prevent spikes
        self.Q -= Kappa @ Qy_kp1 @ Kappa.T  

    def filter(self, y_mes):
        """
        Wrapper function to keep track of estimate and covariance history
        """
        # Create noise input control law about hover condition 
        ubar = np.zeros((4, 1))
        ubar[0, 0] = self.quad.m * self.quad.g

        # Run filter 
        self.filter_UT(y_mes, ubar)

        # Keep history (Won't need this as will obtain from rosbag)
        # self.xhath.append(copy.deepcopy(self.xhat))
        # self.Qh.append(copy.deepcopy(self.Q))

    def project_to_psd(self, Q, eps=1e-6):
        Q_sym = 0.5 * (Q + Q.T)
        eigvals, eigvecs = np.linalg.eigh(Q_sym)
        eigvals_clipped = np.clip(eigvals, eps, None)
        return eigvecs @ np.diag(eigvals_clipped) @ eigvecs.T

    def simulate_lidar(self, y_mes):
        """
        Add noise to the simulation measurement in order to simulate the lidar's precision 

        Input:
            - Position measurement from the simulation, nd.array of size (3, 1).
        Output: 
            - Position measurement from simulation with noise, nd.array of size (3, 1).  
        """

        # All you need is to add noise to the measurement obtained from the simulation
        return np.random.multivariate_normal(y_mes[:, 0], self.R).reshape(3, 1) 

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
        pose_msg.pose.position.x = self.xhat[0, 0]
        pose_msg.pose.position.y = self.xhat[1, 0]
        pose_msg.pose.position.z = self.xhat[2, 0]
        pose_msg.pose.orientation.w = self.xhat[6, 0] # Scalar first quaternion 
        pose_msg.pose.orientation.x = self.xhat[7, 0]
        pose_msg.pose.orientation.y = self.xhat[8, 0]
        pose_msg.pose.orientation.z = self.xhat[9, 0]

        # Populate TwistStamped msg
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "world"
        twist_msg.twist.linear.x = self.xhat[3, 0]
        twist_msg.twist.linear.y = self.xhat[4, 0]
        twist_msg.twist.linear.z = self.xhat[5, 0]
        twist_msg.twist.angular.x = self.xhat[10, 0]
        twist_msg.twist.angular.y = self.xhat[11, 0]
        twist_msg.twist.angular.z = self.xhat[12, 0]
        
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
        self.filter(pos_noisy) 

        # Populate pos and vel estimates
        pos_est, vel_est = self.from_numpy_to_rosmsg()

        # Publish state estimate 
        self.pos_est_pub.publish(pos_est)
        self.vel_est_pub.publish(vel_est) 






###########################################################################

def main(args=None):
    rclpy.init(args=args)

    # Define initial conditions 
    Q0 = np.diag([1e-1, 1e-1, 1e-1, 1e-1, 1e-1, 1e-1, 1e-3, 1e-3, 1e-3, 1e-3, 1e-2, 1e-2, 1e-2])
    # Q0 = np.diag([1.] * 13) * 0.1
    x0 = np.array([[0., 0., 1.5, 0., 0., 0., 1., 0., 0., 0., 0., 0., 0.,]]).T # p: 0, 0, 1.5; v: 0, 0, 0; q: 1, 0, 0, 0; omega: 0, 0, 0

    # Initialize quad then ufk
    sampling_rate = 30 # Hz, same as flight controller publishing freq.
    dt = 1 / sampling_rate
    quad = Quad(x0, dt)
    kf_node = UKF(quad, x0, Q0, alpha=1e-1, beta=2, kappa=-10)

    rclpy.spin(kf_node)
    
    kf_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()