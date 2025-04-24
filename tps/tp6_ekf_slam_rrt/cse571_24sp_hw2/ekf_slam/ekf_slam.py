""" 
Written by Yi Li for CSE571: Probabilistic Robotics (Spring 2024)
"""

import numpy as np

from utils import minimized_angle


class ExtendedKalmanFilterSLAM:
    def __init__(self, pose, pose_cov, control_noise_param, measure_cov):
        self.control_noise_param = control_noise_param
        self.measure_cov = measure_cov

        self.N = 0
        self._init_pose = pose 
        self._init_pose_cov = pose_cov
        self.observed_landmarks = []
        self.reset()

    def reset(self):
        self.N = 0 # number of landmarks observed
        self.mu = np.ones((3, 1)) # [xs, ys, theta] -> [xs, ys, theta, xm1, ym1, xm2, ym2, ...]
        self.sigma = np.eye(3) # covariance matrix, 3 x 3 -> 2N+3 x 2N+3
        self.mu[:3] = self._init_pose
        self.sigma[:3, :3] = self._init_pose_cov

    def init_landmark_cov(self, z, mu_pred, sigma_pred, idx):
        self.observed_landmarks.append(idx)
        self.N += 1
        mu_pred = np.concatenate((mu_pred, np.zeros((2, 1))), axis=0)
        sigma_pred = np.concatenate([
            sigma_pred, np.zeros((2, sigma_pred.shape[1]))
        ], axis=0)
        sigma_pred = np.concatenate([
            sigma_pred, np.zeros((sigma_pred.shape[0], 2))
        ], axis=1)
        
        xs, ys, theta = mu_pred[:3].ravel()
        phi, r = z.ravel()

        # init landmark mean
        landmark_mu = np.zeros((2, 1))
        landmark_mu[0,0] = xs+r*np.cos(theta+phi)
        landmark_mu[1,0] = ys+r*np.sin(theta+phi)

        # init landmark covariance
        # landmark_cov = np.eye(2)
        Hms = np.array([[1, 0, -r * np.sin(theta + phi)],
						[0, 1, r * np.cos(theta + phi)]])
        Hmz = np.array([[-r*np.sin(theta+phi),np.cos(theta+phi)],
						[r*np.cos(theta+phi),np.sin(theta+phi)]])
        landmark_cov = Hms @ self.sigma[:3,:3] @ Hms.T + Hmz @ self.measure_cov @ Hmz.T
        
        mu_pred[-2:] = landmark_mu
        sigma_pred[-2:, -2:] = landmark_cov
        return mu_pred, sigma_pred
  
    def update(self, u, obs, marker_ids):
        """Update the state estimate after taking an action and receiving a landmark
        observation.
        Check Table 10.1 in the book Sec 10.2 for reference
        Implement the jacobian based on the one you inferred in the assignment document.
        
        u: action
        obs: landmark observations
        marker_ids: landmark IDs
        
        mu: the state of the robot and the landmarks
        sigma: the covariance matrix of the state and the landmarks
        s: the state of the robot
        m: the state of the landmarks
        j: the index of the landmark currently being observed
        z: the observation of the landmark j
        """
        
        # PREDICTION #
        Fx = np.zeros((3, 3 + 2 * self.N)) # 3 for robot, 2 for each landmark
        Fx[:3, :3] = np.eye(3)
        theta = self.mu[2][0]
        rot1, trans, rot2 = u.ravel()
        # Update mu_pred, sigma_pred, Gt in the prediction step
        mu_pred = self.mu       # [3 + 2 * self.N, 1]
        sigma_pred = self.sigma # [3 + 2 * self.N, 3 + 2 * self.N]
        Gt = None
        # YOUR IMPLEMENTATION STARTS HERE
        
        # YOUR IMPLEMENTATION ENDS HERE
        
        # CORRECTION
        for z, marker_id in zip(obs, marker_ids):
            # add new landmark if not observed before
            if marker_id not in self.observed_landmarks:
                mu_pred, sigma_pred = self.init_landmark_cov(z, mu_pred, sigma_pred, marker_id)
                
            # update X_pred with observation on landmark
            j = self.observed_landmarks.index(marker_id)
            H = self.H(mu_pred, z, j)
            z_pred = self.observe(mu_pred, j)
            # YOUR IMPLEMENTATION STARTS HERE
            # correct mu_pred and sigma_pred with observation on landmark
            
            # YOUR IMPLEMENTATION ENDS HERE

        self.mu = mu_pred
        self.sigma = sigma_pred
        return self.mu, self.sigma
    
    @staticmethod
    def G(mu, u, Fx):
        """Compute the Jacobian of the dynamics with respect to the state."""
        xt, yt, theta = mu[:3].ravel()
        rot1, trans, rot2 = u.ravel()
        jacobian = np.zeros((3,3))
        Gt = np.eye(Fx.shape[1])
        # YOUR IMPLEMENTATION STARTS HERE
        
        # YOUR IMPLEMENTATION ENDS HERE
        return Gt

    @staticmethod
    def V(mu, u):
        """Compute the Jacobian of the dynamics with respect to the control."""
        prev_x, prev_y, prev_theta = mu[:3].ravel()
        rot1, trans, rot2 = u.ravel()
        Vt = np.zeros((3, 3))
        # YOUR IMPLEMENTATION STARTS HERE
        
        # YOUR IMPLEMENTATION ENDS HERE
        return Vt

    @staticmethod
    def noise_from_motion(u, alphas):
        """Compute covariance matrix for noisy action.
        Sec. 5.4.1 Table 5.5
        u: [rot1, trans, rot2]
        alphas: noise parameters for odometry motion model
        """
        variances = np.zeros(3)
        variances[0] = alphas[0] * u[0]**2 + alphas[1] * u[1]**2
        variances[1] = alphas[2] * u[1]**2 + alphas[3] * (u[0]**2 + u[2]**2)
        variances[2] = alphas[0] * u[2]**2 + alphas[1] * u[1]**2
        return np.diag(variances)
    
    def R(self, mu, u):
        Vt = self.V(mu, u)
        control_noise_cov = self.noise_from_motion(u, self.control_noise_param)
        Rt = None
        # YOUR IMPLEMENTATION HERE
        
        # YOUR IMPLEMENTATION HERE
        return Rt
    
    def H(self, mu, z, j):
        """Compute the Jacobian of the observation of landmark j with respect to the state.
        mu: tracked state of robot and all observed landmarks [3 + num_landmarks * 2, 1]
        z: landmark observation [2, 1]
        j: landmark index
        """
        # x = x.ravel()
        xt, yt, theta = mu[:3].ravel()
        xm, ym = mu[3+2*j:3+2*j+2].ravel()
        phi, r = z.ravel()
        Hs = None # Jacobian of the observation model with respect to the robot state
        Hm = None # Jacobian of the observation model with respect to the landmark
        H = None # the H_t^i matrix to return
        # YOUR IMPLEMENTATION STARTS HERE
        
        # YOUR IMPLEMENTATION ENDS HERE
        return H
    
    @staticmethod
    def observe(mu, j):
        """Compute observation, given current state and landmark ID.
        x: [x, y, theta]
        marker_id: int
        return: [phi, r]
        """
        dx = mu[2*j+3] - mu[0]
        dy = mu[2*j+3+1] - mu[1]
        return np.array(
            [minimized_angle(np.arctan2(dy, dx) - mu[2]),
             np.sqrt(dx**2 + dy**2),]
        ).reshape((-1, 1))
    
