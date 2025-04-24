import matplotlib.pyplot as plt
import numpy as np
import pybullet as p
import pybullet_data
import time


class ArmEnvironment:
    def __init__(self, urdf_file, start_pos, goal_pos, obstacles):
        self.start_pos = start_pos
        self.goal_pos = goal_pos
        self.obstacles = obstacles
        self.setup_pybullet_environment(urdf_file)
        # self.setup_camera()
        self.add_goal(goal_pos)
        self.add_obstacles(obstacles)
        self.set_ee_link()
        self.joint_info = self.get_revolute_joint_info()
        self.joint_indices = [i[0] for i in self.joint_info]
        self.goal_tolerance = 0.05
        self.num_discretize = 100
        self.calculate_c_space()
        self.map = self.c_space
        self.c_space_dim = len(self.c_space.shape)
        self.c_space_limit = [self.c_space.shape[i] for i in range(self.c_space_dim)]
        
        # Set the start and goal joint angles
        self.start_joint_pose = [self.theta_discretized[i][self.start_pos[i]] for i in range(len(self.joint_info))]
        self.goal_joint_pose = self.collision_free_ik(self.goal_pos, self.joint_indices)
        self.set_robot_joint(self.joint_indices, self.start_joint_pose)
        
        self.start_joint_state = np.array([np.argmin(np.abs(self.theta_discretized[i] - self.start_joint_pose[i])) for i in range(len(self.start_joint_pose))]).reshape((self.c_space_dim, 1))
        self.goal_joint_state = np.array([np.argmin(np.abs(self.theta_discretized[i] - self.goal_joint_pose[i])) for i in range(len(self.goal_joint_pose))]).reshape((self.c_space_dim, 1))
        self.start = self.start_joint_state
        self.goal = self.goal_joint_state

        self.change_env = False
        self.change_env_step_threshold = 50
        self.change_env_std_dev = 0.1

        print("start_joint_angles", self.start_joint_pose)
        print('start_joint_state', self.start_joint_state.flatten())
        print("goal_joint_angles", self.goal_joint_pose)
        print('goal_joint_state', self.goal_joint_state.flatten())

    def __del__(self):
        # Disconnect from PyBullet
        p.disconnect()
    
    def sample(self):
        # Sample random clear point from map
        clear = np.argwhere(self.c_space == 0)
        idx = np.random.choice(len(clear))
        return clear[idx, :].reshape((self.c_space_dim, 1))
    
    def goal_criterion(self, config):
        # Check if the robot has reached the goal
        return self.dist_to_goal(config) < self.goal_tolerance
    
    def state_validity_checker(self, config):
        """ Return True if all states are valid

            @param config: a [2 x n] numpy array of states
        """
        config = config.reshape((self.c_space_dim, -1))
        # print(config, self.c_space_limit)
        for x in config.T:
            for i, xi in enumerate(x):
                if xi<0 or xi>=self.c_space_limit[i]:
                    return False
            
            if self.map[tuple([int(j) for j in x])]==1:
                return False
        return True
    
    def edge_validity_checker(self, config1, config2):
        """ Return True if edge is validl

            @param config1: a [C x 1] numpy array of state
            @param config2: a [C x 1] numpy array of state
        """
        config1 = config1.flatten()
        config2 = config2.flatten()
        assert(config1.shape == (self.c_space_dim, ))
        assert(config2.shape == (self.c_space_dim, ))
        n = max([i for i in self.c_space_limit])
        
        configs = np.vstack([np.linspace(config1[i], config2[i], n).reshape(1, n) for i in range(self.c_space_dim)])
        return self.state_validity_checker(configs)
        
    def setup_pybullet_environment(self, urdf_file):
        # Connect to PyBullet and load the URDF file
        self.PCID = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, 0)
        self.ROBOT_ID = p.loadURDF(urdf_file, basePosition=[0, 0, 0])

    def setup_camera(self):
        # Set up the camera in the environment
        camera_distance = 3.5
        camera_yaw = 0
        camera_pitch = -15
        camera_target_position = (0, 0, 0.5)

        p.resetDebugVisualizerCamera(
            cameraDistance=camera_distance,
            cameraYaw=camera_yaw,
            cameraPitch=camera_pitch,
            cameraTargetPosition=camera_target_position,
        )
        
    def init_visualizer(self):
        """ Initialize visualizer
        """

        self.fig = plt.figure()
        self.ax1 = self.fig.add_subplot(1, 1, 1)

        # Plot img
        visit_map = 1 - np.copy(self.map) # black is obstacle, white is free space
        self.ax1_img = self.ax1.imshow(visit_map, interpolation="nearest", cmap="gray")
        

    def visualize_plan(self, plan=None, tree=None, visited=None, pause=0.5):
        """
            Visualize the final path
            @param plan: a final [2 x n] numpy array of states
        """
        fig = plt.figure()

        visit_map = 1 - np.copy(self.map) # black is obstacle, white is free space
        if visited is not None:
            visit_map[visited == 1] = 0.5
        plt.imshow(visit_map, interpolation="nearest", cmap="gray", vmin=0, vmax=1)

        if tree is not None:
            for idx in range(len(tree.vertices)):
                if idx == tree.GetRootID():
                    continue
                econfig = tree.vertices[idx]
                sconfig = tree.vertices[tree.edges[idx]]
                x = [sconfig[0], econfig[0]]
                y = [sconfig[1], econfig[1]]
                plt.plot(y, x, 'r')

        if plan is not None:
            for i in range(np.shape(plan)[1] - 1):
                x = [plan[0,i], plan[0,i+1]]
                y = [plan[1,i], plan[1,i+1]]
                plt.plot(y, x, 'b', linewidth=3)
                fig.canvas.draw()
                plt.pause(pause) 

        fig.canvas.draw()
        
    def add_obstacles(self, obstacles):
        # Add obstacles to the environment
        self.obstacle_ids = []
        for obstacle_pos, obstacle_size in obstacles:
            self.obstacle_ids.append(self.add_obstacle(obstacle_pos, obstacle_size))

    def randomize_obstables(self):
        # Add obstacles to the environment
        valid = False

        while not valid:
            for obstacle_id in self.obstacle_ids:
                obPos, obOrn = p.getBasePositionAndOrientation(obstacle_id)
                newPos = np.random.normal(obPos, self.change_env_std_dev)
                p.resetBasePositionAndOrientation(obstacle_id, newPos, obOrn)
            
            goal_state_valid = not self.is_in_collision(self.goal_joint_state)
            start_state_valid = not self.is_in_collision(self.start_joint_state)
            valid = goal_state_valid and start_state_valid

        # self.start_joint_state
        # self.goal_joint_state

    def add_obstacle(self, cube_position, cube_size):
        # Add a single obstacle to the environment
        cube_orientation = p.getQuaternionFromEuler([0, 0, 0])
        cube_color = [0.2, 0.2, 0.2, 1]  # RGBA color
        cube_position[2] += cube_size[2] / 2  # from bottom-center representation to mass-center representation
        collision_shape_id = p.createCollisionShape(
            p.GEOM_BOX, halfExtents=[size / 2 for size in cube_size]
        )
        visual_shape_id = p.createVisualShape(
            p.GEOM_BOX, halfExtents=[size / 2 for size in cube_size], rgbaColor=cube_color
        )

        obstacle_id = p.createMultiBody(
            baseMass=0,
            baseInertialFramePosition=[0, 0, 0],
            baseCollisionShapeIndex=collision_shape_id,
            baseVisualShapeIndex=visual_shape_id,
            basePosition=cube_position,
            baseOrientation=cube_orientation,
        )

        return obstacle_id

    def add_goal(self, goal_position):
        # Add the goal to the environment
        sphere_visual = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.1, rgbaColor=[1, 0.84, 0, 1.0])
        sphere_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=-1, baseVisualShapeIndex=sphere_visual, basePosition=goal_position)

    def set_ee_link(self):
        # Set the end-effector link index for the robot
        for link_index in range(p.getNumJoints(self.ROBOT_ID)):
            link_info = p.getJointInfo(self.ROBOT_ID, link_index)
            link_name = link_info[12].decode('utf-8')
            if link_name == 'ee':
                self.EE_IDX = link_index
                break

    def get_joint_angles(self, joint_indices):
        # Get the joint angles for the specified joint indices
        joint_angles = []
        for joint_index in joint_indices:
            joint_info = p.getJointState(self.ROBOT_ID, joint_index)
            joint_angle = joint_info[0]
            joint_angles.append(joint_angle)
        return joint_angles


    def is_in_collision(self, joint_state):
        # Check if the robot is in collision with the obstacles
        # Set the robot's joint angles
        joint_angles = [self.theta_discretized[i][joint_state[i,0]] for i in range(joint_state.shape[0])]
        self.set_robot_joint(self.joint_indices, joint_angles)

        # Step the simulation for one step to update the robot's pose
        p.stepSimulation()

        # Check for collisions between the robot and each obstacle
        for obstacle_id in self.obstacle_ids:
            contact_points = p.getContactPoints(bodyA=self.ROBOT_ID, bodyB=obstacle_id)
            if contact_points:
                return True  # Collision detected

        return False  # No collision detected

    def dist_to_goal(self, endpoint_position):
        # Calculate the distance between the endpoint and the goal
        distance = self.compute_distance(endpoint_position, self.goal_joint_state)
        # distance = np.sqrt((goal_position[0] - endpoint_position[0]) ** 2 + (goal_position[2] - endpoint_position[2]) ** 2)
        return distance
    
    def compute_distance(self, config1, config2):
        # Compute the distance between two configurations
        return np.linalg.norm(np.array(config1) - np.array(config2))
    
    def start_sim(self):
        # Run the simulation for a specified number of steps
        for i in range (10000):
            p.stepSimulation()
            time.sleep(1./240.)

    def print_robot_joint_info(self, robotId):
        # Print the joint information for the robot
        numJoints = p.getNumJoints(robotId)

        # Iterate over all the joints in the robot and print their information
        for i in range(numJoints):
            jointInfo = p.getJointInfo(robotId, i)
            print("Joint index: ", jointInfo[0])
            print("Joint name: ", jointInfo[1])
            print("Joint type: ", jointInfo[2])
            print("Joint lower limit: ", jointInfo[8])
            print("Joint upper limit: ", jointInfo[9])

    def calculate_c_space(self):
        # Get the configuration space for the robot
        # Define the range of joint angles for each joint
        num_joints = len(self.joint_info)
        theta_discretized = []
        for i in range(num_joints):
            theta_discretized.append(np.linspace(self.joint_info[i][1], self.joint_info[i][2], self.num_discretize))
        
        c_space_shape = [len(theta_discretized[i]) for i in range(num_joints)]
        c_space = np.zeros(c_space_shape)

        for indices in np.ndindex(tuple(c_space_shape)):
            joint_angles = [theta_discretized[i][indices[i]] for i in range(num_joints)]
            for k in range(num_joints):
                p.resetJointState(self.ROBOT_ID, self.joint_info[k][0], joint_angles[k])
            p.stepSimulation()
            contact_points = p.getContactPoints(physicsClientId=self.PCID)
            if len(contact_points) > 0:
                c_space[indices] = 1

        self.c_space = c_space
        self.theta_discretized = theta_discretized

    def visualize_c_space(self):
        # Visualize the configuration space
        # Plot the configuration space
        fig = plt.figure()
        ax = fig.add_subplot(111)

        ax.imshow(self.c_space)
        ax.set_xlabel('Joint angle 2')
        ax.set_ylabel('Joint angle 1')
        plt.show()
        fig.savefig("c_space.png")

    def follow_path(self, path):
        time_step = 0.0001
        max_steps = 10000
        tolerance = 1e-3
        count = 0
        # Make the robot follow the given path
        joint_indices = [i[0] for i in self.joint_info]

        # Set the robot state to start state
        start_joint_angles = [self.theta_discretized[i][self.start_joint_state[i]] for i in range(len(self.joint_info))]
        self.set_robot_joint(joint_indices, start_joint_angles)
        print(f"Following path:\n{path}")
        for indices in path:
            target_joint_angles = [self.theta_discretized[i][indices[i]] for i in range(len(self.joint_info))]

            # The plan changes for Dynamic path planning
            if self.change_env:
                if count > self.change_env_step_threshold:
                    self.start_joint_state = np.array(indices).reshape(-1,1)
                    return False

            for i in range(max_steps):
                # Get the current joint angles
                current_joint_angles = [p.getJointState(self.ROBOT_ID, j)[0] for j in joint_indices]
                current_joint_angles = np.array(self.bound_joint_angles(current_joint_angles))

                # Compute the joint angle errors
                joint_angle_errors = np.array(target_joint_angles) - np.array(current_joint_angles)
                joint_angle_errors = np.array(self.bound_joint_angles(joint_angle_errors))

                # Check if the robot has reached the target angles within a certain tolerance
                if np.all(np.abs(joint_angle_errors) < tolerance):
                    break
                for k in range(len(target_joint_angles)):
                    p.setJointMotorControl2(bodyUniqueId=self.ROBOT_ID, 
                                            jointIndex=joint_indices[k],
                                            controlMode=p.POSITION_CONTROL, 
                                            targetPosition=target_joint_angles[k],
                                            positionGain=0.1,
                                            velocityGain=0.05,
                                            maxVelocity=3,
                                            force=100,
                                            targetVelocity=0,
                                            )

                # Step the simulation
                p.stepSimulation()
                time.sleep(time_step)
            count += 1
        return True

    def set_robot_joint(self, joint_indices, joint_angles):
        # Set the robot's joint angles
        for k in range(len(joint_angles)):
            p.resetJointState(self.ROBOT_ID, joint_indices[k], joint_angles[k])
        p.stepSimulation()

    def get_revolute_joint_info(self):
        # Get the joint information for all revolute joints in the robot
        numJoints = p.getNumJoints(self.ROBOT_ID)
        joint_info = []

        # Iterate over all the joints in the robot and print their information
        for i in range(numJoints):
            jointInfo = p.getJointInfo(self.ROBOT_ID, i)
            if jointInfo[2] == 0:
                joint_info.append((jointInfo[0], jointInfo[8], jointInfo[9]))

        return joint_info #(joint_index, joint lower limit, joint upper limit)

    def collision_free_ik(self, target_pos, joint_indices, max_iterations=500, step_size=0.01):
        # Find collision-free inverse kinematics solution
        for _ in range(max_iterations):
            joint_angles = p.calculateInverseKinematics(self.ROBOT_ID, self.EE_IDX, target_pos)

            self.set_robot_joint(joint_indices, joint_angles)

            p.stepSimulation()
            time.sleep(0.01)

            contact_points = p.getContactPoints(physicsClientId=self.PCID)

            if len(contact_points) == 0:
                joint_angles = self.bound_joint_angles(joint_angles)
                return joint_angles

            # Move target position slightly and try again
            target_pos += np.random.uniform(-step_size, step_size, 3)

        return None  # Return None if a collision-free solution cannot be found

    def bound_joint_angles(self, joint_angles):
        joint_angles = [ja + 2*np.pi if ja < -np.pi else ja for ja in joint_angles]
        joint_angles = [ja - 2*np.pi if ja >  np.pi else ja for ja in joint_angles]
        return joint_angles
