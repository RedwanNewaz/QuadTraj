import numpy as np
from utility import discretizeActionSpace, get_action_value
from Quadrotor import Quadrotor
from model import QuadrotorKinematics

if __name__ == '__main__':
    num_subdivision = 25
    dimesion = 4
    ActionSpace = discretizeActionSpace(dimesion, num_subdivision)
    sample_size = np.prod([num_subdivision] * dimesion)
    index = np.random.uniform(0, sample_size, 1).astype(int)
    indices_4d, value = get_action_value(index, ActionSpace)


    # control parameter
    quad = QuadrotorKinematics()
    robot = Quadrotor()
    print(f'sampled action {value}')
    vx, vy, vz, wz = value


    # Simulation parameters
    dt = 0.1  # Time step in seconds
    pred_time = 2.0 # Total simulation time in seconds
    sim_time = 0

    # Simulation loop
    while sim_time < pred_time:
        quad.predict_motion(vx, vy, vz, wz, dt)
        x, y, z = quad.get_position()
        psi = quad.get_orientation()
        robot.update_pose(x, y, z, 0, 0, psi)

        sim_time += dt

        # Print position and orientation every step
        print(f"Time: {sim_time:.3f} s")
        print(f"Position: {quad.get_position()}")
        print(f"Orientation: {psi: .3f}")
