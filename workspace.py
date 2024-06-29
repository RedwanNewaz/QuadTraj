import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def draw_cube(ax, position, size):
    # Create a list of vertices for the cube
    r = [-size / 2, size / 2]
    vertices = np.array([[x, y, z] for x in r for y in r for z in r])
    vertices += np.array(position)

    # Create a list of sides' polygons
    sides = [
        [vertices[0], vertices[1], vertices[3], vertices[2]],
        [vertices[4], vertices[5], vertices[7], vertices[6]],
        [vertices[0], vertices[1], vertices[5], vertices[4]],
        [vertices[2], vertices[3], vertices[7], vertices[6]],
        [vertices[0], vertices[2], vertices[6], vertices[4]],
        [vertices[1], vertices[3], vertices[7], vertices[5]]
    ]

    # Create a 3D polygon collection for the sides
    ax.add_collection3d(Poly3DCollection(sides, facecolors='gray', linewidths=1, edgecolors='r', alpha=0.5))

if __name__ == '__main__':

    # Initialize the 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Define the cube positions and size
    cube_positions = [
        [1, 1, 1], [3, 1, 1], [1, 3, 1], [3, 3, 1], [2, 2, 2],
        [1, 1, 3], [3, 1, 3], [1, 3, 3], [3, 3, 3]
    ]
    cube_size = 0.5

    # Draw the cubes
    for pos in cube_positions:
        draw_cube(ax, pos, cube_size)

    # Set the labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Workspace Visualization')
    ax.set_xlim3d(0, 5)
    ax.set_ylim3d(0, 5)
    ax.set_zlim3d(0, 5)

    # Set the aspect ratio
    ax.set_box_aspect([1, 1, 1])

    # Show the plot
    plt.show()
