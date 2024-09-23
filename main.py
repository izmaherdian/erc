import numpy as np
from config import *
from utils import *
if CONTROLLER == 'edc':
    from robot_edc import Robot
else:
    from robot_bc import Robot

def collision(robots):
    for i in range(NUM_ROBOT):
        position = robots[i].position
        for j in range(len(OBSTACLES)):
            obstacle = OBSTACLES[j]
            obs_point = nearest_point_to_obstacle(position[:2], obstacle)
            if np.linalg.norm(position[:2]-obs_point) < ROBOT_RADIUS:
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
        for i in range(NUM_ROBOT):
            robots[i].compute_control(robots, dt=TIMESTEP)

        # Check collision
        if collision(robots):
            print("Collision")
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