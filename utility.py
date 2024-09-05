import numpy as np
from configparser import ConfigParser
def index_1d_to_4d(index, shape):
    # shape is a tuple representing the dimensions of the 4D matrix (d1, d2, d3, d4)
    d1, d2, d3, d4 = shape

    # Compute the 4D indices
    i = index // (d2 * d3 * d4)
    index = index % (d2 * d3 * d4)

    j = index // (d3 * d4)
    index = index % (d3 * d4)

    k = index // d4
    l = index % d4

    return i, j, k, l

def discretizeActionSpace(dimesion, num_subdivision):

    #linear velocities lie between [-1, 1]
    Vx = Vy = Vz = np.linspace(-1.0, 1.0, num_subdivision)

    #angular velocity lies between [-pi, pi]
    Wz = np.linspace(-np.pi / 16.0, np.pi / 16.0, num_subdivision)

    # take a cross product
    X, Y, Z, W = np.meshgrid(Vx, Vy, Vz, Wz)

    # resultant action space is [dim x num_subdivision x num_subdivision x num_subdivision x num_subdivision]
    # example dim = 4, num_subdivision = 25 -> [4 x 25 x 25 x 25 x 25]
    return np.vstack((X, Y, Z, W)).reshape((dimesion, num_subdivision, num_subdivision, num_subdivision, num_subdivision))

def get_action_value(index, ActionSpace):
    dim = ActionSpace.shape[0]
    shape = ActionSpace.shape[1:]
    indices_4d = index_1d_to_4d(index, shape)
    value = np.array([ActionSpace[i][indices_4d] for i in range(dim)])
    return indices_4d, np.squeeze(value)

if __name__ == '__main__':
    # Example usage
    num_subdivision = 25
    dimesion = 4
    ActionSpace = discretizeActionSpace(dimesion, num_subdivision)
    sample_size = np.prod([num_subdivision] * dimesion)
    index = np.random.uniform(0, sample_size, 1).astype(int)
    indices_4d, value = get_action_value(index, ActionSpace)

    print(ActionSpace.shape)
    print(sample_size)
