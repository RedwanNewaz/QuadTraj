import numpy as np
from VoxelPlanner import discretize_cube_into_voxels


if __name__ == '__main__':
    cube_size = 3.0
    num_divisions = 5
    ActionSpace = discretize_cube_into_voxels(cube_size, num_divisions)
    ActionSpace = np.array(ActionSpace)
    print(ActionSpace.shape)

    Vx = Vy = Vz = np.linspace(-1.0, 1.0, 125)
    W = np.linspace(-np.pi, np.pi, 125)

