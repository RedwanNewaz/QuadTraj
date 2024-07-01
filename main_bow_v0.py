import matplotlib.pyplot as plt
import numpy as np
from utility import discretizeActionSpace, get_action_value
from Quadrotor import Quadrotor
from model import QuadrotorKinematics
from CollisionChecker import getObstacleChecker, minCollisionDistance, goalDistance
from bayes_opt import BayesianOptimization
from scipy.optimize import NonlinearConstraint
from functools import partial
from argparse import ArgumentParser
from configparser import ConfigParser

def compute_short_traj(quad, action):
    # Simulation parameters
    dt = 0.1  # Time step in seconds
    pred_time = 2 * dt # Total simulation time in seconds
    sim_time = 0
    vx, vy, vz, wz = action
    while sim_time < pred_time:
        quad.predict_motion(vx, vy, vz, wz, dt)
        x, y, z = quad.get_position()
        psi = quad.get_orientation()
        sim_time += dt
        yield x, y, z, psi



def target_function(robot, ActionSpace, index):
    index = int(index)
    indices_4d, value = get_action_value(index, ActionSpace)
    quad_t = QuadrotorKinematics(initial_position=robot.get_position(), initial_orientation=robot.get_orientation())
    traj = [[x, y, z, psi] for x, y, z, psi in compute_short_traj(quad_t, value)]
    goal_dist = min(goalDistance(pos[:3], robot.goal_pos) for pos in traj)
    return np.exp(-goal_dist)

def constraint_function(robot, ActionSpace, manager, index):
    index = int(index)
    indices_4d, value = get_action_value(index, ActionSpace)
    quad_t = QuadrotorKinematics(initial_position=robot.get_position(), initial_orientation=robot.get_orientation())
    traj = [[x, y, z, psi] for x, y, z, psi in compute_short_traj(quad_t, value)]
    obs_dist = min(minCollisionDistance(pos[:3], manager) for pos in traj)
    return  np.exp(-obs_dist)

def init_robot(args):
    config = ConfigParser()
    config.read(args.config)
    # robot parameter
    x = config.getfloat('ROBOT', 'x')
    y = config.getfloat('ROBOT', 'y')
    z = config.getfloat('ROBOT', 'z')
    robot = Quadrotor(x=x, y=y, z=z)
    # Define the obstacle positions and size with cubes
    cube_positions = [
        [1, 1, 1], [3, 1, 1], [1, 3, 1], [3, 3, 1], [2, 2, 2],
        [1, 1, 3], [3, 1, 3], [1, 3, 3], [3, 3, 3]
    ]

    robot.set_obstacles(cube_positions, args.cube_size)

    robot.goal_pos = [config.getfloat('GOAL', key) for key in ['x', 'y', 'z']]

    return robot
def main(args):
    num_subdivision = args.num_sub_div
    dimesion = args.num_dim
    ActionSpace = discretizeActionSpace(dimesion, num_subdivision)
    sample_size = np.prod([num_subdivision] * dimesion)

    num_trajectories = num_subdivision * 3
    safety_dist = 0.345 * 2


    # config BOW
    robot = init_robot(args)
    manager = getObstacleChecker()

    pbounds = {'index': (0, sample_size)}
    fun_target_function = partial(target_function, robot, ActionSpace)
    fun_constraint_function = partial(constraint_function, robot, ActionSpace, manager)
    constraint = NonlinearConstraint(fun_constraint_function, -np.inf, safety_dist)

    num_collide = 0
    while True:

        optimizer = BayesianOptimization(
            f=fun_target_function,
            constraint=constraint,
            pbounds=pbounds,
            verbose=0,  # verbose = 1 prints only when a maximum is observed, verbose = 0 is silent
            random_state=num_trajectories,
        )

        optimizer.maximize(
            init_points=num_trajectories,
            n_iter=1,
        )
        print(optimizer.max)
        index = int(optimizer.max['params']['index'])
        indices_4d, value = get_action_value(index, ActionSpace)
        quad_t = QuadrotorKinematics(initial_position=robot.get_position(), initial_orientation=robot.get_orientation())
        best_traj = [[x, y, z, psi] for x, y, z, psi in compute_short_traj(quad_t, value)]

        # show the best trajectory and execute the control
        x, y, z, psi = best_traj[0]
        if minCollisionDistance([x, y, z], manager) < 0.345:
            num_collide += 1
            print('collide', num_collide)
            # continue
        robot.update_pose(x, y, z, 0, 0, psi)

        current = np.array([x, y, z])
        target = np.array(robot.goal_pos)
        if np.linalg.norm(current - target) < args.cube_size:
            print('goal found ! collide = ', num_collide)
            break


if __name__ == '__main__':

    parser = ArgumentParser()
    parser.add_argument("--config", type=str, default="test/test1.txt")
    parser.add_argument("--num_sub_div", type=int, default=10)
    parser.add_argument("--num_dim", type=int, default=4)
    parser.add_argument("--cube_size", type=float, default=0.5)

    args = parser.parse_args()
    main(args)








