import numpy as np
from config import *
from utils import *

if CONTROLLER == 'erc':
    from robot_erc import Robot
elif CONTROLLER == 'iapf':
    from robot_iapf import Robot
else:
    from robot_bc import Robot

def collision(robots):
    for i in range(NUM_ROBOT):
        position = robots[i].position
        for j in range(len(OBSTACLES)):
            obstacle = OBSTACLES[j]
            obs_point = nearest_point_to_obstacle(position[:2], obstacle)
            if np.linalg.norm(position[:2]-obs_point) < ROBOT_RADIUS:
                print("Obstacle")
                return True
            
    for i in range(NUM_ROBOT-1):
        pi = robots[i].position
        for j in range(i+1, NUM_ROBOT):
            pj = robots[j].position
            if np.linalg.norm(pi - pj) < 2*ROBOT_RADIUS:
                print("Inter-agent")
                return True
    return False
            

if __name__ == "__main__":
    # Create Robots
    robots = []
    for i in range(NUM_ROBOT):
        robot = Robot(i, INITS[i,:])
        robots.append(robot)

    iter = 0
    while iter < ITER_MAX:
        iter += 1
        if iter % 10 == 0:
            print("Iteration {}".format(iter))

        for i in range(NUM_ROBOT):
            robots[i].compute_control(robots, dt=TIMESTEP)

        # Check collision
        if collision(robots):
            # print("Collision")
            break

        # Check reach goal
        count = 0
        for i in range(NUM_ROBOT):
            if robots[i].position[0] > XGOAL:
                count += 1
        if count == NUM_ROBOT:
            break

    # Save data
    import pickle
    with open(FILE_NAME, 'wb') as file:
        data = []
        for i in range(NUM_ROBOT):
            d = dict()
            d['path'] = np.array(robots[i].path)
            data.append(d)
        pickle.dump(data, file)