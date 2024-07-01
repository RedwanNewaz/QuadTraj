//
// Created by redwan on 7/1/24.
//

#ifndef QUAD_TRAJ_BOW_PLANNER_H
#define QUAD_TRAJ_BOW_PLANNER_H

#include "bo_param.h"
#include "collision_checker.h"
#include "model.h"


class bow_planner{
public:

    BO_PARAM(size_t, dim_in, 4);
    BO_PARAM(size_t, dim_out, 1);
    BO_PARAM(size_t, nb_constraints, 1);

    bow_planner(const std::vector<double>& robot_pose):robot_pose(robot_pose)
    {
//        std::vector<float> robot_pose = {1, 1, 0, 0};

        std::vector<std::vector<float>> cube_positions = {
                {1, 1, 1}, {3, 1, 1}, {1, 3, 1}, {3, 3, 1}, {2, 2, 2},
                {1, 1, 3}, {3, 1, 3}, {1, 3, 3}, {3, 3, 3}
        };
        float cube_size = 0.5f;
        checker.setupObstacleChecker(cube_positions, cube_size);
    }

    double get_obs_distance(const std::vector<double>& robot_pose) const
    {
        std::vector<float> robot_pos = {float(robot_pose[0]), float(robot_pose[1]), float(robot_pose[2])};
        float obs_distance = checker.minCollisionDistance(robot_pos);
        return (double) obs_distance;
    }

    double get_goal_dist(const std::vector<double>& robot_pose) const
    {
        std::vector<float> goal_pos = {2.3f, 3.0f, 3.5f};
        std::vector<float> robot_pos = {float(robot_pose[0]), float(robot_pose[1]), float(robot_pose[2])};
        float goal_distance = checker.goalDistance(robot_pos, goal_pos);
        return (double) goal_distance;
    }

    Eigen::VectorXd calc_control_input() const {

        using namespace limbo;
        using Stop_t = boost::fusion::vector<stop::MaxIterations<Params>>;
        using Stat_t = boost::fusion::vector<stat::Samples<Params>,
                stat::BestObservations<Params>,
                stat::AggregatedObservations<Params>>;
        using Mean_t = mean::Constant<Params>;
        using Kernel_t = kernel::Exp<Params>;
        using GP_t = model::GP<Params, Kernel_t, Mean_t>;
        using Constrained_GP_t = model::GP<Params, Kernel_t, Mean_t>;

        using Acqui_t = experimental::acqui::ECI<Params, GP_t, Constrained_GP_t>;
        using Init_t = init::RandomSampling<Params>;

        tools::par::init();
        experimental::bayes_opt::CBOptimizer<Params,
                modelfun<GP_t>,
                acquifun<Acqui_t>,
                statsfun<Stat_t>,
                initfun<Init_t>,
                stopcrit<Stop_t>,
                experimental::constraint_modelfun<Constrained_GP_t>>
                opt;

        double dist = -std::numeric_limits<double>::max();
        do {
            opt.optimize(*this);
            auto uu = opt.best_sample();
            Eigen::Vector3d position;
            position << robot_pose[0], robot_pose[1], robot_pose[2];
            double orientation = robot_pose[3];
            QuadrotorKinematics quadrotor(position, orientation);
            quadrotor.predict_motion(uu(0), uu(1), uu(2), uu(3), 0.1);
            dist = get_obs_distance(quadrotor.get_pose());
        }while (dist <= 0.345);


        return opt.best_sample();

    }

    Eigen::VectorXd operator()(const Eigen::VectorXd& u) const
    {
        Eigen::VectorXd res(2);
        // we _maximize in [0:1]
        Eigen::VectorXd uu = scaledU(u);

        Eigen::Vector3d position;
        position << robot_pose[0], robot_pose[1], robot_pose[2];
        double orientation = robot_pose[3];


        double goal_distance(1000.0), obs_distance(1000.0);
        QuadrotorKinematics quadrotor(position, orientation);

        double time = 0.0;
        const double dt = 0.1;
        double pred_time = 4.0 * dt;
        while (time < pred_time) {
            quadrotor.predict_motion(uu(0), uu(1), uu(2), uu(3), dt);
            auto pose = quadrotor.get_pose();

            goal_distance = std::min(get_goal_dist(pose), goal_distance);
//            goal_distance += get_goal_dist(pose);
            obs_distance = std::min(get_obs_distance(pose), obs_distance);
            time += dt;
        }




        // testing the constraints
        // 0: infeasible 1: feasible


        res(0) = res(1) = 0.0;
        res(0) = exp(-goal_distance);
        res(1) = std::min(exp(obs_distance), 1.0);
//        res(0) = exp(-goal_distance);

//        if(obs_distance > 0.345)
//        {
//            res(0) = exp(-goal_distance);
//            res(1) = 1.0;
//        }

        return res;
    }

private:
    QuadrotorCollisionChecker checker;

    std::vector<double> robot_pose;

    Eigen::VectorXd scaledU(const Eigen::VectorXd& u)const
    {
        Eigen::VectorXd uu(4);
        double min_speed = -1.0;
        double max_speed = 1.0;
        double min_yaw = -M_PI / 16.0;
        double max_yaw = M_PI / 16.0;

        // map linear velocities
        for (int i = 0; i < 3; ++i) {
            uu(i) = min_speed + (max_speed - min_speed) * u(i); // minmax scaler
        }
        // map angular velocity
        uu(3) = min_yaw + (max_yaw - min_yaw) * u(3);

        return uu;
    }

};
#endif //QUAD_TRAJ_BOW_PLANNER_H
