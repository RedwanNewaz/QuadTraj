//
// Created by redwan on 7/1/24.
//

#ifndef QUAD_TRAJ_COLLISION_CHECKER_H
#define QUAD_TRAJ_COLLISION_CHECKER_H

#include <fcl/fcl.h>
#include <iostream>
#include <vector>
#include <memory>

using namespace fcl;

class QuadrotorCollisionChecker {
public:
    QuadrotorCollisionChecker(float robot_size=0.345f)
    :robot_size(robot_size) { }

    void setupObstacleChecker(std::vector<std::vector<float>> cube_positions, float cube_size) {

        std::shared_ptr<Boxf> box(new Boxf(cube_size, cube_size, cube_size));
        std::vector<CollisionObjectf*> objs1;

        for (const auto& pos : cube_positions) {
            Transform3f tf(Translation3f(pos[0], pos[1], pos[2]));
            objs1.push_back(new CollisionObjectf(box, tf));
        }

        manager = std::make_shared<DynamicAABBTreeCollisionManagerf>();
        manager->registerObjects(objs1);
        manager->setup();
    }

    float minCollisionDistance(const std::vector<float>& robot_pos) const{
        std::shared_ptr<Boxf> g1(new Boxf(robot_size, robot_size, robot_size));
        Transform3f t1(Translation3f(robot_pos[0], robot_pos[1], robot_pos[2]));
        CollisionObjectf o1(g1, t1);

        fcl::DefaultDistanceData<float> distanceData;

        manager->distance(&o1, &distanceData, DefaultDistanceFunction<float>);

        return distanceData.result.min_distance;
    }

    float goalDistance(const std::vector<float>& robot_pos, const std::vector<float>& goal_pos) const{
        float robot_size = 0.345f;

        std::shared_ptr<Boxf> g1(new Boxf(robot_size, robot_size, robot_size));
        Transform3f t1(Translation3f(robot_pos[0], robot_pos[1], robot_pos[2]));
        CollisionObjectf o1(g1, t1);

        std::shared_ptr<Boxf> g2(new Boxf(robot_size, robot_size, robot_size));
        Transform3f t2(Translation3f(goal_pos[0], goal_pos[1], goal_pos[2]));
        CollisionObjectf o2(g2, t2);

        DistanceRequestf request;
        DistanceResultf result;

        distance(&o1, &o2, request, result);

        return result.min_distance;
    }

private:
    std::shared_ptr<DynamicAABBTreeCollisionManagerf> manager;
    float robot_size;


};
#endif //QUAD_TRAJ_COLLISION_CHECKER_H
/*
 * Example code will return
 * Collision between manager and robot: 0.5775
 * Goal distance: 3.68851
int main() {
    QuadrotorCollisionChecker checker;
    std::vector<float> robot_pos = {1, 1, 0};
    std::vector<std::vector<float>> cube_positions = {
            {1, 1, 1}, {3, 1, 1}, {1, 3, 1}, {3, 3, 1}, {2, 2, 2},
            {1, 1, 3}, {3, 1, 3}, {1, 3, 3}, {3, 3, 3}
    };
    float cube_size = 0.5f;
    checker.setupObstacleChecker(cube_positions, cube_size);


    float min_distance = checker.minCollisionDistance(robot_pos);
    std::cout << "Collision between manager and robot: " << min_distance << std::endl;

    std::vector<float> goal_pos = {2.3f, 3.0, 3.5f};
    float goal_distance = checker.goalDistance(robot_pos, goal_pos);
    std::cout << "Goal distance: " << goal_distance << std::endl;

    return 0;
}

 */