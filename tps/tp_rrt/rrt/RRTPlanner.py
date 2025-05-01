from RRTTree import RRTTree
import numpy as np
import sys
import time


class RRTPlanner(object):
    def __init__(self, planning_env, bias = 0.05, eta = 1.0, max_iter = 10000):
        self.env = planning_env         # Map Environment
        self.tree = RRTTree(self.env)
        self.bias = bias                # Goal Bias
        self.max_iter = max_iter        # Max Iterations
        self.eta = eta                  # Distance to extend

    def Plan(self, start_config, goal_config):
        """ Some utility functions you can use:
            From RRTTree:
                new_vertex_id = self.tree.AddVertex(node, cost))
                self.tree.AddEdge(start_id, end_id)
                vertex_id, vertex = self.tree.GetNearestVertex(state)
            From RRTPlanner:
                new_state = self.sample(target_state)
                new_state = self.extend(state1, state2)
            From ArmEnvironment:
                cost = self.env.compute_distance(state1, state2
                reached_goal = self.env.goal_criterion(state)
        """
        # Initialize an empty plan.
        goal_id = -1
        plan = []
        plan_time = time.time()

        self.tree.AddVertex(start_config, cost=0)
        # TODO: YOUR IMPLEMENTATION STARTS HERE
        
        # YOUR IMPLEMENTATION ENDS HERE

        if goal_id < 0:
            print("WARNING: RRT Failed!")
            sys.exit(-1)

        # Construct plan
        plan.insert(0, goal_config)
        cid = goal_id
        root_id = self.tree.GetRootID()
        while cid != root_id:
            cid = self.tree.edges[cid]
            plan.insert(0, self.tree.vertices[cid])
        plan = np.concatenate(plan, axis=1)

        plan_time = time.time() - plan_time
        print("Planning complete!")
        print(f"Planning Time: {plan_time:.3f}s")

        return plan.T

    def extend(self, x_near, x_rand):
        """
        may be useful:
            self.eta: ratio to extend from x_near to x_rand
            bool_result = self.env.edge_validity_checker(x_near, x_new)
        return None if edge not valid
        """
        # TODO: YOUR IMPLEMENTATION STARTS HERE
        
        # YOUR IMPLEMENTATION ENDS HERE

    def sample(self, goal):
        # Sample random point from map
        if np.random.uniform() < self.bias:
            return goal
        return self.env.sample()
