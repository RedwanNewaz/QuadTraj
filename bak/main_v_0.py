import matplotlib.pyplot as plt
import numpy as np
from VoxelPlanner import discretize_cube_into_voxels, plot_voxel_grid_with_scalar_colors
from CollisionChecker import getObstacleChecker, minCollisionDistance, goalDistance


if __name__ == '__main__':
    cube_size = 3.0
    num_divisions = 5
    ActionSpace = discretize_cube_into_voxels(cube_size, num_divisions)
    ActionSpace = np.array(ActionSpace)

    currentPos = np.array((1, 1, 0))
    goal_pos = [2.3, 3, 3.5]
    weight = 2.0
    manager = getObstacleChecker()
    cost_values = []
    for u in ActionSpace:
        cand = currentPos + u
        obstacle_distance = minCollisionDistance(cand.tolist(), manager)
        goal_distance = goalDistance(cand.tolist(), goal_pos)
        cost = goal_distance - weight * obstacle_distance
        # cost = np.exp(goal_distance) - np.exp(-obstacle_distance/2.0)
        cost_values.append(cost)

    cost_values = np.array(cost_values)
    cost_values = np.reshape(cost_values, (num_divisions, num_divisions, num_divisions))
    print(cost_values.shape)
    plot_voxel_grid_with_scalar_colors(cost_values, cube_size, num_divisions)
    plt.show()

