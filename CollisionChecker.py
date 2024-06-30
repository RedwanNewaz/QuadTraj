import fcl
import numpy as np


def getObstacleChecker():
    # Define the cube positions and size
    cube_positions = [
        [1, 1, 1], [3, 1, 1], [1, 3, 1], [3, 3, 1], [2, 2, 2],
        [1, 1, 3], [3, 1, 3], [1, 3, 3], [3, 3, 3]
    ]
    cube_size = 0.5
    box = fcl.Box(cube_size, cube_size, cube_size)
    objs1 = [fcl.CollisionObject(box, fcl.Transform(np.array(t))) for t in cube_positions]
    manager1 = fcl.DynamicAABBTreeCollisionManager()
    manager1.registerObjects(objs1)
    return manager1

def minCollisionDistance(robot_pos, manager):
    robot_size = 0.345
    g1 = fcl.Box(robot_size,robot_size,robot_size)
    t1 = fcl.Transform(np.array(robot_pos))
    o1 = fcl.CollisionObject(g1, t1)

    req = fcl.DistanceRequest(enable_nearest_points=True, enable_signed_distance=False)
    rdata = fcl.DistanceData(request=req)

    manager.distance(o1, rdata, fcl.defaultDistanceCallback)

    return rdata.result.min_distance



def goalDistance(robot_pos, goal_pos):
    robot_size = 0.345

    g1 = fcl.Box(robot_size, robot_size, robot_size)
    t1 = fcl.Transform(np.array(robot_pos))
    o1 = fcl.CollisionObject(g1, t1)

    g2 = fcl.Box(robot_size, robot_size, robot_size)
    t2 = fcl.Transform(np.array(goal_pos))
    o2 = fcl.CollisionObject(g2, t2)

    request = fcl.DistanceRequest()
    result = fcl.DistanceResult()

    ret = fcl.distance(o1, o2, request, result)

    return result.min_distance


if __name__ == '__main__':

    manager1 = getObstacleChecker()
    t = (1, 1, 0)
    min_distance = minCollisionDistance(t, manager1)
    print('Collision between manager and robot: {:.3f}'.format(min_distance))

    goal_pos = [2.3, 3, 3.5]

    goal_distance = goalDistance(t, goal_pos)
    print('goal distance: {:.3f}'.format(goal_distance))



