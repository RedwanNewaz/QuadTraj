import matplotlib.pyplot as plt
import numpy as np
from utility import discretizeActionSpace, get_action_value
from Quadrotor import Quadrotor
from model import QuadrotorKinematics
from CollisionChecker import getObstacleChecker, minCollisionDistance, goalDistance

def compute_short_traj(quad, action):
    # Simulation parameters
    dt = 0.1  # Time step in seconds
    pred_time = 2.0 # Total simulation time in seconds
    sim_time = 0
    vx, vy, vz, wz = action
    while sim_time < pred_time:
        quad.predict_motion(vx, vy, vz, wz, dt)
        x, y, z = quad.get_position()
        psi = quad.get_orientation()
        sim_time += dt
        yield x, y, z, psi

def sample_trajectories(robot, num_trajectories):
    indexes = np.random.uniform(0, sample_size, num_trajectories).astype(int)
    for index in indexes:
        indices_4d, value = get_action_value(index, ActionSpace)
        quad_t = QuadrotorKinematics(initial_position=robot.get_position(), initial_orientation=robot.get_orientation())
        traj = [[x, y, z, psi] for x, y, z, psi in compute_short_traj(quad_t, value)]
        yield np.array(traj)


if __name__ == '__main__':
    num_subdivision = 25
    dimesion = 4
    ActionSpace = discretizeActionSpace(dimesion, num_subdivision)
    sample_size = np.prod([num_subdivision] * dimesion)



    # control parameter
    robot = Quadrotor(show_animation=True)

    # Define the obstacle positions and size with cubes
    cube_positions = [
        [1, 1, 1], [3, 1, 1], [1, 3, 1], [3, 3, 1], [2, 2, 2],
        [1, 1, 3], [3, 1, 3], [1, 3, 3], [3, 3, 3]
    ]
    cube_size = 0.5
    robot.set_obstacles(cube_positions, cube_size)



    # Simulation loop
    goal_pos = robot.goal_pos = [2.3, 3, 3.5]
    num_trajectories = 15 * 4
    weight = 1.15
    safety_dist = 1.0
    max_steps = 100
    manager = getObstacleChecker()

    for _ in range(max_steps):
        best_traj = None
        best_traj_cost = float('inf')

        for traj in sample_trajectories(robot, num_trajectories):
            obs_dist = min(minCollisionDistance(pos[:3], manager) for pos in traj)
            goal_dist = min(goalDistance(pos[:3], goal_pos) for pos in traj)
            cost = goal_dist - min(obs_dist, safety_dist) * weight
            if cost < best_traj_cost:
                best_traj = traj
                best_traj_cost = cost
        # show the best trajectory and execute the control
        x, y, z, psi = best_traj[0]
        robot.update_pose(x, y, z, 0, 0, psi)

        current = np.array([x,  y, z])
        target = np.array(goal_pos)
        if np.linalg.norm(current - target) < cube_size:
            print('goal found !')
            break



