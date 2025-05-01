import pybullet as p
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt

from utils import minimized_angle

try:
    import torch
    from observation_model import ObservationModel
    torch_available = True
except ImportError:
    torch_available = False

class Field:
    # NUM_MARKERS = 6
    NUM_MARKERS = 8

    MAP_FACTOR = 1
    INNER_OFFSET_X = 32 * MAP_FACTOR
    INNER_OFFSET_Y = 13 * MAP_FACTOR

    INNER_SIZE_X = 420 * MAP_FACTOR
    INNER_SIZE_Y = 270 * MAP_FACTOR

    COMPLETE_SIZE_X = INNER_SIZE_X + 2 * INNER_OFFSET_X
    COMPLETE_SIZE_Y = INNER_SIZE_Y + 2 * INNER_OFFSET_Y

    MARKER_OFFSET_X = 21 * MAP_FACTOR
    MARKER_OFFSET_Y = 0 * MAP_FACTOR

    # MARKER_DIST_X = 442 * MAP_FACTOR
    # MARKER_DIST_Y = 292 * MAP_FACTOR

    MARKER_DIST_X = 350 * MAP_FACTOR
    MARKER_DIST_Y = 300 * MAP_FACTOR

    # MARKERS = (1, 2, 3, 4, 5, 6)
    MARKERS = (1, 2, 3, 4, 5, 6, 7, 8)

    MARKER_X_POS = {
        1: MARKER_OFFSET_X,
        2: MARKER_OFFSET_X + 0.5 * MARKER_DIST_X,
        3: MARKER_OFFSET_X + MARKER_DIST_X,
        4: MARKER_OFFSET_X + MARKER_DIST_X,
        5: MARKER_OFFSET_X + 0.5 * MARKER_DIST_X,
        6: MARKER_OFFSET_X,
        7: MARKER_OFFSET_X,
        8: MARKER_OFFSET_X + MARKER_DIST_X,
    }

    MARKER_Y_POS = {
        1: MARKER_OFFSET_Y,
        2: MARKER_OFFSET_Y,
        3: MARKER_OFFSET_Y,
        4: MARKER_OFFSET_Y + MARKER_DIST_Y,
        5: MARKER_OFFSET_Y + MARKER_DIST_Y,
        6: MARKER_OFFSET_Y + MARKER_DIST_Y,
        7: MARKER_OFFSET_Y + 0.5*MARKER_DIST_Y,
        8: MARKER_OFFSET_Y + 0.5*MARKER_DIST_Y,
    }

    def __init__(
        self,
        alphas,
        beta,
        max_range,
        gui=True,
        # min_range=50,
    ):
        self.alphas = alphas
        self.beta = beta
        self.max_range = max_range
        self.min_range = 50 #min_range
        # initialize pybullet environment
        if gui:
            physicsClient = p.connect(p.GUI)
            cameraDistance = 4.0*self.MAP_FACTOR
            cameraYaw = 120
            cameraPitch = -60
            cameraTargetPosition = [4*self.MAP_FACTOR, 2*self.MAP_FACTOR, 0.5]

            p.resetDebugVisualizerCamera(cameraDistance, cameraYaw, cameraPitch, cameraTargetPosition)
        else:
            physicsClient = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,-10)
        self.p = p
        
        # add the robot and landmarks to the pybullet scene
        self.create_scene()
        self.add_robot()
        

    def forward(self, x, u):
        """Compute next state, given current state and action.

        Implements the odometry motion model.

        x: [x, y, theta]
        u: [rot1, trans, rot2]
        """
        prev_x, prev_y, prev_theta = x
        rot1, trans, rot2 = u

        x_next = np.zeros(x.size)
        theta = prev_theta + rot1
        x_next[0] = prev_x + trans * np.cos(theta)
        x_next[1] = prev_y + trans * np.sin(theta)
        x_next[2] = minimized_angle(theta + rot2)

        return x_next.reshape((-1, 1))

    def get_marker_id(self, step):
        """Compute the landmark ID at a given timestep."""
        return ((step // 2) % self.NUM_MARKERS) + 1

    def observe(self, x, marker_id):
        """Compute observation, given current state and landmark ID.

        x: [x, y, theta]
        marker_id: int
        """
        dx = self.MARKER_X_POS[marker_id] - x[0]
        dy = self.MARKER_Y_POS[marker_id] - x[1]
        return np.array(
            [minimized_angle(np.arctan2(dy, dx) - x[2])]
        ).reshape((-1, 1))
        
    def observe_lidar(self, x, marker_id):
        """Compute observation, given current state and landmark ID.

        x: [x, y, theta]
        marker_id: int
        return: [bearing, range]
        """
        dx = self.MARKER_X_POS[marker_id] - x[0]
        dy = self.MARKER_Y_POS[marker_id] - x[1]
        return np.array([
            minimized_angle(np.arctan2(dy, dx) - x[2]),
            np.sqrt(dx**2 + dy**2)
        ]).reshape((-1, 1))
            
    def observe_multiple(self, x):
        """Compute observation for all landmarks within range"""
        observations, marker_ids = [], []
        for m in self.MARKERS:
            dx = self.MARKER_X_POS[m] - x[0]
            dy = self.MARKER_Y_POS[m] - x[1]
            dist = np.sqrt(dx**2 + dy**2)
            if dist <= self.max_range and dist >= self.min_range:
                observations.append(np.stack([
                    minimized_angle(np.arctan2(dy, dx) - x[2]), dist
                ]))
                marker_ids.append(m)
        return observations, marker_ids

    def noise_from_motion(self, u, alphas):
        """Compute covariance matrix for noisy action.

        u: [rot1, trans, rot2]
        alphas: noise parameters for odometry motion model
        """
        variances = np.zeros(3)
        variances[0] = alphas[0] * u[0]**2 + alphas[1] * u[1]**2
        variances[1] = alphas[2] * u[1]**2 + alphas[3] * (u[0]**2 + u[2]**2)
        variances[2] = alphas[0] * u[2]**2 + alphas[1] * u[1]**2
        return np.diag(variances)

    def likelihood(self, innovation, beta):
        """Compute the likelihood of innovation, given covariance matrix beta.

        innovation: difference between expected and observed bearing angle
        beta: noise parameters for landmark observation model
        """
        norm = np.sqrt(np.linalg.det(2 * np.pi * beta))
        inv_beta = np.linalg.inv(beta)

        return np.exp(-0.5 * innovation.T.dot(inv_beta).dot(innovation)) / norm

    def sample_noisy_action(self, u, alphas=None):
        """Sample a noisy action, given a desired action and noise parameters.

        u: desired action
        alphas: noise parameters for odometry motion model (default: data alphas)
        """
        if alphas is None:
            alphas = self.alphas

        cov = self.noise_from_motion(u, alphas)
        return np.random.multivariate_normal(u.ravel(), cov)

    def sample_noisy_observation(self, obs, beta=None):
        """Sample a noisy observation given a current state, landmark ID, and noise
        parameters.

        x: current state
        marker_id: int
        beta: noise parameters for landmark observation model (default: data beta)
        """
        if beta is None:
            beta = self.beta
        noisy_observation = []
        for z in obs:
            noisy_observation.append(np.random.multivariate_normal(z.ravel(), beta).reshape(-1, 1))
            # if noisy_observation[-1][1, 0] < self.min_range:
            #     noisy_observation[-1][1, 0] = self.min_range
        return noisy_observation

    def get_figure(self):
        return plt.figure(1)

    def rollout(self, x0, policy, num_steps, dt=0.1):
        """Collect data from an entire rollout."""
        states_noisefree = np.zeros((num_steps, 3))
        states_real = np.zeros((num_steps, 3))
        action_noisefree = np.zeros((num_steps, 3))
        obs_noisefree = []
        obs_real = []
        marker_ids = []

        x_noisefree = x_real = x0
        for i in range(num_steps):
            t = i * dt

            u_noisefree = policy(x_real, t)
            x_noisefree = self.forward(x_noisefree, u_noisefree)

            u_real = self.sample_noisy_action(u_noisefree)
            x_real = self.forward(x_real, u_real)
            states_noisefree[i, :] = x_noisefree.ravel()
            states_real[i, :] = x_real.ravel()
            action_noisefree[i, :] = u_noisefree.ravel()

        for i in range(num_steps):
            x_real = states_real[i, :].reshape((-1, 1))
            z_noisefree, ids = self.observe_multiple(x_real)
            z_real = self.sample_noisy_observation(z_noisefree)

            obs_noisefree.append(z_noisefree)
            obs_real.append(z_real)
            marker_ids.append(ids)

        states_noisefree = np.concatenate([x0.T, states_noisefree], axis=0)
        states_real = np.concatenate([x0.T, states_real], axis=0)

        return (
            states_noisefree, states_real,
            action_noisefree,
            obs_noisefree, obs_real, marker_ids
        )
    
    # pybullet
    def create_scene(self):
        self.plane_id = self.p.loadURDF("plane.urdf")
        h = 0.5
        r = 0.05

        pillar_shape = self.p.createCollisionShape(
            self.p.GEOM_CYLINDER, radius=r, height=h)
        colors = [
            [0.9, 0.0, 0.0, 1.0],
            [0.0, 0.9, 0.0, 1.0],
            [0.0, 0.0, 0.9, 1.0],
            [0.5, 0.5, 0.0 ,1.0],
            [0.0, 0.5, 0.5, 1.0],
            [0.5, 0.0, 0.5, 1.0],
            [0.25, 0.25, 0.25, 1.0],
            [0.75, 0.75, 0.75, 1.0],
        ]
        self.pillar_ids = []
        self.text_ids = []
        for m in self.MARKERS:
            x, y = self.MARKER_X_POS[m]/100, self.MARKER_Y_POS[m]/100
            pillar_id = self.p.createMultiBody(
                baseCollisionShapeIndex=pillar_shape, basePosition=[x, y, h/2])
            self.pillar_ids.append(pillar_id)
            self.p.setCollisionFilterGroupMask(pillar_id, -1, 0, 0)
            self.p.changeVisualShape(pillar_id, -1, rgbaColor=colors[m-1])

            text_id = self.p.addUserDebugText(
                str(m),
                textPosition=[0,0,h/2+0.1],
                textColorRGB=[0, 0, 0],
                textSize=2,
                parentObjectUniqueId=pillar_id,
            )
            self.text_ids.append(text_id)
    
    def add_robot(self):
        self.racer_car_id = self.p.loadURDF('racecar.urdf', [0,0,0], [0,0,0,1])
    
    def move_robot(self, x):
        p = [x[0]/100., x[1]/100., 0]
        q = self.p.getQuaternionFromEuler([0,0,x[2]+np.pi])
        self.p.resetBasePositionAndOrientation(self.racer_car_id, p, q)
    
    def get_robot_x(self):
        p, q = self.p.getBasePositionAndOrientation(
            self.racer_car_id)
        theta = self.p.getEulerFromQuaternion(q)[2] + np.pi
        return [p[0]*100, p[1]*100, theta]

    def plot_observation(self, x, obs, marker_ids):
        if hasattr(self, 'obs_ids'):
            for idx in self.obs_ids:
                self.p.removeUserDebugItem(idx)
        self.obs_ids = []
        xyz0 = np.array([x[0, 0] / 100., x[1, 0] / 100., 0.05])
        for z, idx in zip(obs, marker_ids):
            phi = z[0]
            if len(z) > 1:
                dist = z[1] / 100.
            else:
                marker_x = np.array([
                    self.MARKER_X_POS[idx] / 100.,
                    self.MARKER_Y_POS[idx] / 100.,
                    0.05
                ])
                dist = np.linalg.norm(xyz0 - marker_x)
            xyz1 = [
                x[0] / 100. + np.cos(phi + x[2]) * dist,
                x[1] / 100. + np.sin(phi + x[2]) * dist,
                0.05
            ]
            self.obs_ids.append(
                self.p.addUserDebugLine(xyz0, xyz1, [0, 0, 0], 2)
            )
    
    def plot_path_step(self, x_previous, x_current, color):
        xyz_previous = [x_previous[0]/100., x_previous[1]/100., 0.05]
        xyz_current = [x_current[0]/100., x_current[1]/100., 0.05]
        self.p.addUserDebugLine(xyz_previous, xyz_current, color, 2)
    
    def plot_particles(self, particles, weights):
        xyz = np.concatenate(
            (particles[:,:2]/100., np.full((len(particles),1), 0.2)), axis=1)
        color = np.zeros((len(particles),3))
        color[:,0] = 1
        color = color * weights.reshape(-1,1) * 50
        color = np.clip(color, 0, 1)
        kwargs = {}
        if hasattr(self, 'particle_id'):
            kwargs['replaceItemUniqueId'] = self.particle_id
        self.particle_id = self.p.addUserDebugPoints(
            xyz, color, pointSize=2, **kwargs)
    
    def draw_cov_ellipse(self, mu, cov, color=[0, 1, 0], old_line_ids={}):
        """
        Draws an ellipse using plotDebugLines in PyBullet.

        :param mu: Mean of a Gaussian
        :param cov: Covariance of a Gaussian
        :param color: Color in PyBullet format, e.g., [1, 0, 0] for red
        """
        n_poly = 8
        U, s, Vh = np.linalg.svd(cov)
        a, b = s[0], s[1]
        vx, vy = U[0, 0], U[0, 1]
        theta = np.arctan2(vy, vx)
        R = np.array([[np.cos(theta), -np.sin(theta)],
                    [np.sin(theta), np.cos(theta)]])
        phi = np.arange(0, 2 * np.pi, 2 * np.pi / n_poly)
        rot = []
        for i in range(n_poly):
            rect = np.array([10 * np.sqrt(a) * np.cos(phi[i]),
                            10 * np.sqrt(b) * np.sin(phi[i])])
            rotated_rect = np.dot(R, rect).reshape(2,) + mu.reshape(2,)
            rot.append(rotated_rect)

        rot = np.asarray(rot)/100
        rot = np.concatenate((rot, np.ones((rot.shape[0], 1)) * 0.05), axis=1)
        new_line_ids = {}
        for i in range(n_poly-1):
            kwargs = {}
            if i in old_line_ids:
                kwargs['replaceItemUniqueId'] = old_line_ids[i]
            new_line_ids[i] = self.p.addUserDebugLine(rot[i], rot[i+1], color, 2, **kwargs)
        
        kwargs = {}
        if n_poly-1 in old_line_ids:
            kwargs['replaceItemUniqueId'] = old_line_ids[n_poly-1]
        new_line_ids[n_poly-1] = self.p.addUserDebugLine(rot[-1], rot[0], color, 2, **kwargs)
        return new_line_ids
    
    def plot_covariance(self, mu, cov):
        num_eclipse = (mu.shape[0]-1)//2
        if not hasattr(self, 'existing_lines'):
            self.existing_lines = {}
        existing_lines = self.existing_lines
        for i in range(num_eclipse):
            if i==0:
                mu_i = mu[:2]
                cov_i = cov[:2,:2]
            else:
                mu_i = mu[2*i+1: 2*i+3]
                cov_i = cov[2*i+1: 2*i+3, 2*i+1: 2*i+3]
            
            if i in existing_lines:
                line_ids = existing_lines[i]
            else:
                line_ids = {}
            self.existing_lines[i] = self.draw_cov_ellipse(mu_i, cov_i, old_line_ids=line_ids)

    def render_panorama(self, resolution=32):
        car_pos, car_orient = self.p.getBasePositionAndOrientation(
            self.racer_car_id)
        steering = self.p.getEulerFromQuaternion(car_orient)[2] + np.pi

        camera_height = 0.2

        # left camera
        left_cam = np.array(car_pos) + [0,0,camera_height]
        left_cam_to = np.array([
            car_pos[0] + np.cos(steering + 1 * np.pi / 2) * 10,
            car_pos[1] + np.sin(steering + 1 * np.pi / 2) * 10,
            car_pos[2] + camera_height,
        ])

        # front camera
        front_cam = np.array(car_pos) + [0,0,camera_height]
        front_cam_to = np.array([
            car_pos[0] + np.cos(steering + 0 * np.pi / 2) * 10,
            car_pos[1] + np.sin(steering + 0 * np.pi / 2) * 10,
            car_pos[2] + camera_height,
        ])

        # right camera
        right_cam = np.array(car_pos) + [0,0,camera_height]
        right_cam_to = np.array([
            car_pos[0] + np.cos(steering + 3 * np.pi / 2) * 10,
            car_pos[1] + np.sin(steering + 3 * np.pi / 2) * 10,
            car_pos[2] + camera_height,
        ])

        # back camera
        back_cam = np.array(car_pos) + [0,0,camera_height]
        back_cam_to = np.array([
            car_pos[0] + np.cos(steering + 2 * np.pi / 2) * 10,
            car_pos[1] + np.sin(steering + 2 * np.pi / 2) * 10,
            car_pos[2] + camera_height,
        ])

        cam_eyes = [left_cam, front_cam, right_cam, back_cam]
        cam_targets = [left_cam_to, front_cam_to, right_cam_to, back_cam_to]
        
        images = []
        #depths = []
        #masks = []
        for i in range(4):
            # Define the camera view matrix
            view_matrix = self.p.computeViewMatrix(
                cameraEyePosition=cam_eyes[i],
                cameraTargetPosition=cam_targets[i],
                cameraUpVector = [0,0,1]
            )
            # Define the camera projection matrix
            projection_matrix = self.p.computeProjectionMatrixFOV(
                fov=90,
                aspect=1.0,
                nearVal=0.1,
                farVal=100.0
            )
            # Add the camera to the scene
            _, _, rgb, depth, segm = self.p.getCameraImage(
                width=resolution,
                height=resolution,
                viewMatrix=view_matrix,
                projectionMatrix=projection_matrix,
                renderer=self.p.ER_BULLET_HARDWARE_OPENGL
            )
            rgb = np.array(rgb).astype('uint8').reshape((resolution, resolution, 4))

            images.append(rgb[:,:,:3])
            #depths.append(depth)
            #masks.append(segm)

        l,f,r,b = images
        rgb_strip = np.concatenate([l,f,r,b], axis=1)
        rgb_strip = np.concatenate(
            [rgb_strip[:,-resolution//2:], rgb_strip[:,:-resolution//2]],
            axis=1,
        )

        return rgb_strip
