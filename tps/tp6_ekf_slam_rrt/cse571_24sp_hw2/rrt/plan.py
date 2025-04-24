import argparse
import matplotlib.pyplot as plt
import numpy as np
import os
import random
import time
from ArmEnvironment import ArmEnvironment
from RRTPlanner import RRTPlanner


def get_args():
    parser = argparse.ArgumentParser(description='script for testing planners')
    parser.add_argument('--seed', type=int, default=0,
                        help='Random seed for sampling')
    parser.add_argument('-o', '--num_obstacles', type=int, default=2,
                        help='Number of obstacles to add to the environment')
    parser.add_argument('-eta', '--eta', type=float, default=1.0,
                        help='eta for RRT')
    parser.add_argument('-b', '--bias', type=float, default=0.05,
                        help='Goal bias for RRT')
    parser.add_argument('-v', '--visualize', action='store_true',
                        help='Visualize the configuration space')
    parser.add_argument('-f', '--follow_path', action='store_true',
                        help='Visualize arm\'s path')
    return parser.parse_args()


if __name__ == "__main__":
    args = get_args()
        
    if args.num_obstacles == 0:
        obstacles = []
    elif args.num_obstacles == 1:
        obstacles = [([-0.3, 0, 1.3], [0.25, 0.25, 0.25])]
    elif args.num_obstacles == 2:
        obstacles = [([0.3, 0, 0.6], [0.25, 0.25, 0.25]),
                    ([-0.3, 0, 1.3], [0.25, 0.25, 0.25]),]
    
    random.seed(args.seed)
    np.random.seed(args.seed)

    urdf_file = f"{os.path.dirname(__file__)}/2dof_planar_robot.urdf"
    start_pos = (0, 0)
    goal_pos = [0, 0, 2.0]
    env = ArmEnvironment(urdf_file, start_pos, goal_pos, obstacles)

    planner = RRTPlanner(env, bias=args.bias, eta=args.eta)

    # Get the path from the planner
    path = planner.Plan(env.start, env.goal)
    cost = [
        env.compute_distance(path[i], path[i+1])
        for i in range(len(path) - 1)
    ]
    print(f"Cost: {sum(cost)}")
    if args.visualize:
        env.visualize_plan(path.T, planner.tree)
        print('Plotting configuration space. Close the window to continue.')
        plt.show()
    if args.follow_path:
        if path is not None:
            env.follow_path(path)
            time.sleep(5)
        else:
            print("No plan returned.")
