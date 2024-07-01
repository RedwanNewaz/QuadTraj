import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D

def discretize_cube_into_voxels(cube_size, num_divisions):
    voxel_size = cube_size / num_divisions
    voxels = []
    for x in range(num_divisions):
        for y in range(num_divisions):
            for z in range(num_divisions):
                voxel_position = np.array([
                    x * voxel_size,
                    y * voxel_size,
                    z * voxel_size
                ])
                voxels.append(voxel_position)

    return voxels

def generate_scalar_values(cube_size, num_divisions):
    # Generate some example scalar values (e.g., random values)
    scalar_values = np.random.rand(num_divisions, num_divisions, num_divisions)
    return scalar_values

def plot_voxel_grid_with_scalar_colors(scalar_values, cube_size, num_divisions):
    # Generate voxel positions
    # voxel_positions = discretize_cube_into_voxels(cube_size, num_divisions)

    # Generate scalar values
    # scalar_values = generate_scalar_values(cube_size, num_divisions)

    # Normalize scalar values to [0, 1] for colormap mapping
    norm = plt.Normalize(vmin=np.min(scalar_values), vmax=np.max(scalar_values))

    # Choose a colormap
    cmap = cm.viridis

    # Map scalar values to colors using the colormap
    colors = cmap(norm(scalar_values))

    # Apply 50% transparency to the colors
    colors[..., 3] = 0.5

    # Plotting
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot voxels using ax.voxels with scalar-based colors and 50% transparency
    p = ax.voxels(np.ones((num_divisions, num_divisions, num_divisions)), facecolors=colors, edgecolors='k')

    # Set labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Set limits
    ax.set_xlim([0, num_divisions])
    ax.set_ylim([0, num_divisions])
    ax.set_zlim([0, num_divisions])

    m = cm.ScalarMappable(cmap=cmap, norm=norm)
    m.set_array([])


    fig.colorbar(m, ax=ax)

    plt.title('Voxel Grid Visualization with Scalar Colors and 50% Transparency')
    plt.show()

if __name__ == '__main__':
    # Example parameters
    cube_size = 10.0
    num_divisions = 5

    # plot_voxel_grid_with_scalar_colors(cube_size, num_divisions)
    scalar_values = generate_scalar_values(cube_size, num_divisions)
    print(scalar_values.shape)