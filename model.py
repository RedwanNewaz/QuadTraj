import numpy as np

class QuadrotorKinematics:
    def __init__(self, initial_position=[0.0, 0.0, 0.0], initial_orientation=0):
        self.position = np.array(initial_position)
        self.rotation_matrix = self.yaw_to_rotation_matrix(initial_orientation)

    @staticmethod
    def yaw_to_rotation_matrix(yaw):
        """
        Convert yaw angle to a 3x3 rotation matrix.
        Parameters:
        yaw (float): The yaw angle in radians.
        Returns:
        np.ndarray: A 3x3 rotation matrix.
        """

        cos_theta = np.cos(yaw)
        sin_theta = np.sin(yaw)
        rotation_matrix = np.array([
            [cos_theta, -sin_theta, 0],
            [sin_theta, cos_theta, 0],
            [0, 0, 1]
        ])
        return rotation_matrix

    def predict_motion(self, vx, vy, vz, wz, dt):

        # Update orientation (rotation matrix) based on angular velocity around z-axis
        omega_z_dt = wz * dt

        rotation_increment = self.yaw_to_rotation_matrix(omega_z_dt)
        self.rotation_matrix = np.dot(self.rotation_matrix, rotation_increment)

        # Update position based on linear velocities
        self.position = self.rotation_matrix @ self.position
        delta_p = np.array([vx * dt, vy * dt, vz * dt])
        self.position += delta_p

    def get_position(self):
        return self.position

    @staticmethod
    def rotation_matrix_to_euler_angles(R):
        """
        Converts a rotation matrix to Euler angles (ZYX convention).

        Parameters:
        R : ndarray
            3x3 rotation matrix

        Returns:
        tuple
            Euler angles (phi, theta, psi) in radians
        """
        assert R.shape == (3, 3), "R must be a 3x3 matrix"

        # Check for gimbal lock
        if R[2, 0] != 1 and R[2, 0] != -1:
            theta = -np.arcsin(R[2, 0])
            cos_theta = np.cos(theta)
            psi = np.arctan2(R[2, 1] / cos_theta, R[2, 2] / cos_theta)
            phi = np.arctan2(R[1, 0] / cos_theta, R[0, 0] / cos_theta)
        else:
            phi = 0
            if R[2, 0] == -1:
                theta = np.pi / 2
                psi = np.arctan2(R[0, 1], R[0, 2])
            else:
                theta = -np.pi / 2
                psi = np.arctan2(-R[0, 1], -R[0, 2])

        return phi, theta, psi

    def get_orientation(self):
        phi, theta, psi = self.rotation_matrix_to_euler_angles(self.rotation_matrix)
        return phi



# Example usage:
if __name__ == "__main__":
    from Quadrotor import Quadrotor
    quad = QuadrotorKinematics()
    robot = Quadrotor()

    # Initial velocities (m/s)
    vx = 1.0
    vy = 0.0
    vz = 0.0

    # Angular velocity around z-axis (rad/s)
    wz = 0.1

    # Simulation parameters
    dt = 0.1  # Time step in seconds
    total_time = 15.0  # Total simulation time in seconds
    num_steps = int(total_time / dt)

    # Simulation loop
    for i in range(num_steps):
        quad.predict_motion(vx, vy, vz, wz, dt)
        x, y, z = quad.get_position()
        psi = quad.get_orientation()
        robot.update_pose(x, y, z, 0, 0, psi)

        # Print position and orientation every step
        print(f"Time: {i * dt:.1f} s")
        print(f"Position: {quad.get_position()}")
        print(f"Orientation (Rotation Matrix):\n{quad.get_orientation()}\n")
