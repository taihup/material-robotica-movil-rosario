""" Written by Brian Hou for CSE571: Probabilistic Robotics (Winter 2019)
    Modified by Wentao Yuan for CSE571: Probabilistic Robotics (Spring 2022)
    Modified by Aaron Walsman and Zoey Chen for CSEP590A: Robotics (Spring 2023)
    Modified by Yi Li for CSE571: Probabilistic Robotics (Spring 2024)
"""

import time
import argparse
import numpy as np
import matplotlib.pyplot as plt

from utils import minimized_angle
from soccer_field import Field
import policies
from ekf_slam import ExtendedKalmanFilterSLAM


def localize(
    env: Field,
    policy,
    filt,
    x0,
    num_steps,
    plot=False,
    step_pause=0.,
    step_breakpoint=False,
):
    
    # Collect data from an entire rollout
    (states_noisefree,
     states_real,
     action_noisefree,
     obs_noisefree,
     obs_real,
     marker_ids) = env.rollout(x0, policy, num_steps)
    states_filter = np.zeros(states_real.shape)
    states_filter[0, :] = x0.ravel()

    errors = np.zeros((num_steps, 3))
    position_errors = np.zeros(num_steps)
    mahalanobis_errors = np.zeros(num_steps)

    for i in range(num_steps):
        x_real = states_real[i+1, :].reshape((-1, 1))
        u_noisefree = action_noisefree[i, :].reshape((-1, 1))
        z_real = obs_real[i]
        ids = marker_ids[i]

        if filt is None:
            mean, cov = x_real, np.eye(3)
        elif type(filt) == ExtendedKalmanFilterSLAM:
            # filters only know the action and observation
            mean, cov = filt.update(u_noisefree, z_real, ids)
        states_filter[i+1, :] = mean[:3].ravel()

        # np.set_printoptions(suppress=True)
        # print(mean[3:, 0])
        # markers = np.concatenate([
        #     np.array([env.MARKER_X_POS[j], env.MARKER_Y_POS[j]])
        #     for j in filt.observed_landmarks
        # ])
        # print(markers)
        if plot:
            # move the robot
            env.move_robot(x_real)
            # plot observations
            env.plot_observation(x_real, z_real, ids)
                        
            # plot actual trajectory
            x_real_previous = states_real[i, :].reshape((-1, 1))
            env.plot_path_step(x_real_previous, x_real, [0,0,1])
            
            # # plot noisefree trajectory
            # noisefree_previous = states_noisefree[i]
            # noisefree_current = states_noisefree[i+1]
            # env.plot_path_step(noisefree_previous, noisefree_current, [0,1,0])
            
            # plot estimated trajectory
            if filt is not None:
                filter_previous = states_filter[i]
                filter_current = states_filter[i+1]
                env.plot_path_step(filter_previous, filter_current, [1,0,0])
                env.plot_covariance(mean, cov)
                    
        # pause/breakpoint
        if step_pause:
            time.sleep(step_pause)
        if step_breakpoint:
            breakpoint()
        
        errors[i, :] = (mean[:3] - x_real).ravel()
        errors[i, 2] = minimized_angle(errors[i, 2])
        position_errors[i] = np.linalg.norm(errors[i, :2])

    mean_position_error = position_errors.mean()

    if filt is not None:
        print('-' * 80)
        print('Mean position error:', mean_position_error)
        
    marker_gt = []
    for key in env.MARKER_X_POS.keys():
        marker_gt.append([env.MARKER_X_POS[key], env.MARKER_Y_POS[key]])
    marker_gt = np.array(marker_gt)
    marker_error = np.zeros(env.NUM_MARKERS)
    for j in range(env.NUM_MARKERS):
        marker_pred = filt.mu[3+2*j:3+2*j+2].reshape(1,2)
        closest_dist = np.min(np.linalg.norm(marker_gt - marker_pred, axis=1))
        marker_error[j] = closest_dist
    print('Mean marker error:', np.mean(marker_error))
    if plot:
        while True:
            env.p.stepSimulation()
    
    return mean_position_error, np.mean(marker_error)


def setup_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'filter_type', choices=('none', 'pf', 'ekf', 'ekf_slam'),
        help='filter to use for localization')
    parser.add_argument(
        '--plot', action='store_true',
        help='turn on plotting')
    parser.add_argument(
        '--seed', type=int, default=0,
        help='random seed')
    parser.add_argument(
        '--num-steps', type=int, default=200,
        help='timesteps to simulate')

    # Noise scaling factors
    parser.add_argument(
        '--data-factor', type=float, default=1,
        help='scaling factor for motion and observation noise (data)')
    parser.add_argument(
        '--filter-factor', type=float, default=1,
        help='scaling factor for motion and observation noise (filter)')
    parser.add_argument(
        '--motion-factor', type=float, default=1,
        help='scaling factor for motion noise (data and filter)')
    parser.add_argument(
        '--observation-factor', type=float, default=1,
        help='scaling factor for observation noise (data and filter)')
    parser.add_argument(
        '--lidar-range', type=float, default=250,
        help='maximum range of the lidar sensor')
    
    # Debugging arguments
    parser.add_argument(
        '--step-pause', type=float, default=0.,
        help='slows down the rollout to make it easier to visualize')
    parser.add_argument(
        '--step-breakpoint', action='store_true',
        help='adds a breakpoint to each step for debugging purposes')
    parser.add_argument(
        '--multi_run', type=int, default=1,
    )
    
    return parser


if __name__ == '__main__':
    args = setup_parser().parse_args()
    print('Data factor:', args.data_factor)
    print('Filter factor:', args.filter_factor)
    print('Seed:', args.seed)
    
    control_cov = (np.array([0.05, 0.005, 0.1, 0.01])*args.motion_factor)**2 # motion noise, check book Sec. 5.4.1 Table 5.6
    measure_cov = (np.diag([np.deg2rad(5), 5])*args.observation_factor)**2 # observation noise, 5 degree 1 cm
    # measure_cov = (np.diag([0.01, 5])*args.observation_factor)**2 # observation noise, 5 degree 1 cm
    pe_list = []
    anees_list = []
    
    for i in range(args.multi_run):
        if args.seed is not None:
            np.random.seed(args.seed)
            
        env = Field(
            args.data_factor * control_cov,
            args.data_factor * measure_cov,
            args.lidar_range,
            gui=args.plot
        )
        policy = policies.OpenLoopRectanglePolicy()

        pose = np.array([180, 50, 0]).reshape((-1, 1))
        # pose = np.array([240, 50, 0]).reshape((-1, 1))
        # pose_cov = np.diag([10, 10, 1])
        pose_cov = np.diag([1, 1, 0.1])

        if args.filter_type == 'none':
            filt = None
        elif args.filter_type == 'ekf_slam':
            filt = ExtendedKalmanFilterSLAM(
                pose,
                pose_cov,
                args.filter_factor * control_cov,
                args.filter_factor * measure_cov,
            )

        # You may want to edit this line to run multiple localization experiments.
        pe, anees = localize(env, policy, filt, pose, args.num_steps, args.plot, args.step_pause, args.step_breakpoint)
        pe_list.append(pe)
        anees_list.append(anees)
        args.seed += 1
        
    if args.multi_run > 1:
        print('-' * 80)
        print('Motion factor:', args.motion_factor)
        print('Observation factor:', args.observation_factor)
        print('Data factor:', args.data_factor)
        print('Filter factor:', args.filter_factor)
        print('Seed:', args.seed)
        print('Mean Trajectory Error:', np.mean(pe_list))
        print('Standard deviation of Trajectory:', np.std(pe_list))
        print('Mean Map Error:', np.mean(anees_list))
        print('Standard deviation of Map:', np.std(anees_list))
        
