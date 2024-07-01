#include <iostream>
#include "src/bow_planner.h"
#define DEBUG(x) std::cout << x << std::endl

int main() {
    auto quadrotor = std::make_shared<QuadrotorKinematics>();

    double dist = std::numeric_limits<double>::max();
    do{
        bow_planner planner(quadrotor->get_pose());
        auto uu = planner.calc_control_input();
        quadrotor->predict_motion(uu(0), uu(1), uu(2), uu(3), 0.1);
        dist = planner.get_goal_dist(quadrotor->get_pose());
        auto safe_dist = planner.get_obs_distance(quadrotor->get_pose());
        if(safe_dist <= 0.345)
        {
            DEBUG("collide");
        }
        DEBUG(dist);
    } while (dist > 0.5);


    return 0;
}
