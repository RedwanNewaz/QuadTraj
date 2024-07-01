//
// Created by redwan on 7/1/24.
//

#ifndef QUAD_TRAJ_MODEL_H
#define QUAD_TRAJ_MODEL_H
#include <iostream>
#include <Eigen/Dense>
#include <cmath>

class QuadrotorKinematics {
public:
    QuadrotorKinematics(Eigen::Vector3d initial_position = Eigen::Vector3d(0.0, 0.0, 0.0), double initial_orientation = 0) {
        position = initial_position;
        rotation_matrix = yaw_to_rotation_matrix(initial_orientation);
    }

    void set_pose(Eigen::Vector3d initial_position, double initial_orientation)
    {
        position = initial_position;
        rotation_matrix = yaw_to_rotation_matrix(initial_orientation);
    }

    static Eigen::Matrix3d yaw_to_rotation_matrix(double yaw) {
        double cos_theta = std::cos(yaw);
        double sin_theta = std::sin(yaw);
        Eigen::Matrix3d rotation_matrix;
        rotation_matrix << cos_theta, -sin_theta, 0,
                sin_theta,  cos_theta, 0,
                0,          0,          1;
        return rotation_matrix;
    }

    std::vector<double> get_pose() const
    {
        double yaw = get_orientation();
        return std::vector<double>{position(0), position(1), position(2), yaw};
    }

    void predict_motion(double vx, double vy, double vz, double wz, double dt) {
        double omega_z_dt = wz * dt;
        Eigen::Matrix3d rotation_increment = yaw_to_rotation_matrix(omega_z_dt);
        rotation_matrix = rotation_matrix * rotation_increment;

        Eigen::Vector3d delta_p(vx * dt, vy * dt, vz * dt);
        position += rotation_matrix * delta_p;
    }

    Eigen::Vector3d get_position() const {
        return position;
    }

    static std::tuple<double, double, double> rotation_matrix_to_euler_angles(const Eigen::Matrix3d& R) {
        double phi, theta, psi;

        if (R(2, 0) != 1 && R(2, 0) != -1) {
            theta = -std::asin(R(2, 0));
            double cos_theta = std::cos(theta);
            psi = std::atan2(R(2, 1) / cos_theta, R(2, 2) / cos_theta);
            phi = std::atan2(R(1, 0) / cos_theta, R(0, 0) / cos_theta);
        } else {
            phi = 0;
            if (R(2, 0) == -1) {
                theta = M_PI / 2;
                psi = std::atan2(R(0, 1), R(0, 2));
            } else {
                theta = -M_PI / 2;
                psi = std::atan2(-R(0, 1), -R(0, 2));
            }
        }

        return std::make_tuple(phi, theta, psi);
    }

    double get_orientation() const {
        double phi, theta, psi;
        std::tie(phi, theta, psi) = rotation_matrix_to_euler_angles(rotation_matrix);
        return phi;
    }

private:
    Eigen::Vector3d position;
    Eigen::Matrix3d rotation_matrix;
};
#endif //QUAD_TRAJ_MODEL_H

/*
 *
int main() {
    QuadrotorKinematics quadrotor;
    quadrotor.predict_motion(1.0, 0.0, 0.0, 0.1, 0.1);
    Eigen::Vector3d position = quadrotor.get_position();
    std::cout << "Position: " << position.transpose() << std::endl;
    double orientation = quadrotor.get_orientation();
    std::cout << "Orientation (phi): " << orientation << std::endl;
    return 0;
}
 */