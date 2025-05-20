import numpy as np
from config_traj import *
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

    prev_center = None
    stable_count = 0
    STABLE_THRESHOLD = 10  # iterasi berturut-turut stabil untuk berhenti

    iter = 0
    while iter < ITER_MAX:
        iter += 1
        # if iter % 10 == 0:
        #     print("Iteration {}".format(iter))

        for i in range(NUM_ROBOT):
            robots[i].compute_control(robots, dt=TIMESTEP)
            # print("Robot {}: {}".format(i, robots[i].position))

        # Check collision
        if collision(robots):
            print("Collision")
            break

        # > XGOAL
        # Check reach goal
        # count = 0
        # for i in range(NUM_ROBOT):
        #     if robots[i].position[0] > XGOAL:
        #         count += 1
        # if count == NUM_ROBOT:
        #     break

        # == XGOAL
        # Check reach goal
        # center = np.zeros(3)
        # velocity_sum = np.zeros(3)
        # for i in range(NUM_ROBOT):
        #     center += robots[i].position
        #     velocity_sum += robots[i].velocity
        # center /= NUM_ROBOT
        # avg_velocity = np.linalg.norm(velocity_sum) / NUM_ROBOT

        # threshold = 0.1
        # velocity_threshold = 0.1
        # position_delta_threshold = 0.1
        # if abs(center[0] - XGOAL) < threshold and avg_velocity < velocity_threshold:
        #     if prev_center is not None and np.linalg.norm(center - prev_center) < position_delta_threshold:
        #         stable_count += 1
        #     else:
        #         stable_count = 0
        #     if stable_count >= STABLE_THRESHOLD:
        #         print("Formasi sudah stabil di goal. Simulasi berhenti.")
        #         break
        # else:
        #     stable_count = 0

        # prev_center = center.copy()

        # XGOAL, YGOAL
        # Check reach goal
        # center = np.zeros(3)
        # velocity_sum = np.zeros(3)
        # for i in range(NUM_ROBOT):
        #     center += robots[i].position
        #     velocity_sum += robots[i].velocity
        # center /= NUM_ROBOT
        # print("Center: ", center)
        # avg_velocity = np.linalg.norm(velocity_sum) / NUM_ROBOT

        # goal = np.array([XGOAL, YGOAL])
        # center_2d = center[:2]
        # dist_to_goal = np.linalg.norm(center_2d - goal)

        # threshold = 0.1
        # velocity_threshold = 0.1
        # position_delta_threshold = 0.1

        # if dist_to_goal < threshold and avg_velocity < velocity_threshold:
        #     if prev_center is not None and np.linalg.norm(center - prev_center) < position_delta_threshold:
        #         stable_count += 1
        #     else:
        #         stable_count = 0
        #     if stable_count >= STABLE_THRESHOLD:
        #         print("Formasi sudah stabil di goal. Simulasi berhenti.")
        #         break
        # else:
        #     stable_count = 0

        # prev_center = center.copy()

        # XGOAL, YGOAL, ZGOAL
        # Check reach goal
        center = np.zeros(3)
        velocity_sum = np.zeros(3)
        for i in range(NUM_ROBOT):
            center += robots[i].position
            velocity_sum += robots[i].velocity
        center /= NUM_ROBOT
        print("Center: ", center)
        avg_velocity = np.linalg.norm(velocity_sum) / NUM_ROBOT

        goal = np.array([XGOAL, YGOAL, ZGOAL])
        dist_to_goal = np.linalg.norm(center - goal)

        threshold = 0.1
        velocity_threshold = 0.1
        position_delta_threshold = 0.1

        if dist_to_goal < threshold and avg_velocity < velocity_threshold:
            if prev_center is not None and np.linalg.norm(center - prev_center) < position_delta_threshold:
                stable_count += 1
            else:
                stable_count = 0
            if stable_count >= STABLE_THRESHOLD:
                print("Formasi sudah stabil di goal. Simulasi berhenti.")
                break
        else:
            stable_count = 0

        prev_center = center.copy()

    # Save data
    import pickle
    with open(FILE_NAME, 'wb') as file:
        data = []
        for i in range(NUM_ROBOT):
            d = dict()
            d['path'] = np.array(robots[i].path)
            data.append(d)
        pickle.dump(data, file)