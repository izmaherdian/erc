import numpy as np
import math

from config_traj import *
from utils import *
from enum import Enum

class Mode(Enum):
    FORMATION = 0
    TAILGATING = 1

class Robot():
    def __init__(self, index, position, velocity=np.zeros(3)):
        self.index = index
        
        self.stamp = 0.0
        self.position = position
        self.velocity = velocity
        self.control = np.zeros(3)

        self.mode = Mode.FORMATION

        self.path = [np.concatenate([[self.stamp], self.position, self.velocity, self.control, [self.mode.value]])]

    def update_state(self, control, dt):
        # Limit the control signals
        control_norm = np.linalg.norm(control)
        if control_norm > UMAX:
            control = control/control_norm*UMAX

        velocity = self.velocity + control*dt
        velocity_norm = np.linalg.norm(velocity)
        if velocity_norm > VMAX:
            velocity = velocity/velocity_norm*VMAX

        position = self.position + velocity*dt

        # print("Robot {}: position = {}".format(self.index, position))

        # Update state
        self.stamp += dt
        self.position = position
        self.velocity = velocity
        self.control = control
        self.path.append(np.concatenate([[self.stamp], self.position, self.velocity, self.control, [self.mode.value]]))

    def compute_control(self, robots, dt):
        v_mig = self.behavior_migration(robots)
        v_obs = self.behavior_obstacle()
        v_col = self.behavior_collision(robots)
        v_form = self.behavior_formation(robots)
        v_rand = self.behavior_random()
        desired_velocity = v_mig + v_form + v_obs + v_col + v_rand

        desired_control = (desired_velocity - self.velocity)/dt
        self.update_state(desired_control, dt)

    # > XGOAL 
    # def behavior_migration(self):
    #     return VREF*UREF

    # == XGOAL
    # def behavior_migration(self, robots):
    #     center = np.zeros(3)
    #     for robot in robots:
    #         center += robot.position
    #     center /= NUM_ROBOT

    #     error = XGOAL - center[0]
    #     max_speed = VREF
    #     slow_down_radius = 1.0  # jarak mulai melambat

    #     if abs(error) < slow_down_radius:
    #         # proporsional kecepatan sesuai jarak error
    #         speed = max_speed * (abs(error) / slow_down_radius)
    #     else:
    #         speed = max_speed

    #     if speed < 0.05:
    #         speed = 0  # berhenti kalau sangat kecil

    #     direction = np.sign(error)
    #     velocity = np.array([speed * direction, 0, 0])

    #     return velocity

    # XGOAL, YGOAL
    def behavior_migration(self, robots):
        center = np.zeros(3)
        for robot in robots:
            center += robot.position
        center /= NUM_ROBOT

        goal = np.array([XGOAL, YGOAL])
        center_2d = center[:2]
        error_vec = goal - center_2d
        error_dist = np.linalg.norm(error_vec)

        max_speed = VREF
        slow_down_radius = 1.0  # jarak mulai melambat

        if error_dist < slow_down_radius and error_dist > 0:
            # proporsional kecepatan sesuai jarak error
            speed = max_speed * (error_dist / slow_down_radius)
        elif error_dist == 0:
            speed = 0
        else:
            speed = max_speed

        if speed < 0.05:
            speed = 0  # berhenti kalau sangat kecil

        if error_dist > 0:
            direction = error_vec / error_dist
        else:
            direction = np.array([0.0, 0.0])

        velocity_2d = speed * direction
        velocity = np.array([velocity_2d[0], velocity_2d[1], 0.0])  # tetap 0 untuk z

        return velocity
    
    def behavior_formation(self, robots):
        v_form = 0
        for i in range(NUM_ROBOT):
            v_form += (robots[i].position - self.position) - (TOPOLOGY[i,:] - TOPOLOGY[self.index,:])
        return W_form*v_form
    
    def behavior_obstacle(self):
        v_obs = 0
        for j in range(len(OBSTACLES)):
            obstacle = OBSTACLES[j]
            obs_point = nearest_point_to_obstacle(self.position[:2], obstacle)
            obs_rel = self.position - np.concatenate([obs_point,[self.position[2]]])
            obs_dis = np.linalg.norm(obs_rel)
            if obs_dis < ALERT_RADIUS:
                v_obs += 0.5*(1/obs_dis - 1/ALERT_RADIUS)/(obs_dis**2)*obs_rel/obs_dis
        return W_obs*v_obs
    
    def behavior_collision(self, robots):
        v_col = 0
        for i in range(NUM_ROBOT):
            if i == self.index:
                continue
            pos_rel = self.position - robots[i].position
            pos_dis = np.linalg.norm(pos_rel)
            if pos_dis < ALERT_RADIUS:
                v_col += 2*(1/pos_dis - 1/ALERT_RADIUS)/(pos_dis**2)*pos_rel/pos_dis
        return W_col*v_col

    def behavior_random(self):
        return W_rand*np.concatenate([np.random.rand(2),[0]])