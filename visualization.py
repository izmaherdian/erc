import numpy as np
import pickle
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from config import *

with open(FILE_NAME, 'rb') as file:
    data = pickle.load(file)

def plot_path():
    fig = plt.figure(figsize=SIZE)
    ax = fig.subplots()

    # Plot obstacles
    for i in range(len(OBSTACLES)):
        ax.add_patch(Polygon(OBSTACLES[i],color='grey'))

    # Plot start and goal
    # plt.scatter(INIT[0], INIT[1], marker='s', s= 50)
    # plt.scatter(GOAL[0], GOAL[1], marker='^', s= 50)

    for i in range(NUM_ROBOT):
        path = data[i]['path']
        plt.plot(path[:,1], path[:,2], linewidth=2)

    plt.axis('scaled')
    plt.xlim(XLIM)
    plt.ylim(YLIM)
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.tight_layout()

def plot_speed():
    plt.figure(figsize=(6,3))
    for i in range(NUM_ROBOT):
        path = data[i]['path']
        time_stamp = path[:,0]
        velocity = path[:,4:6]
        speed = np.linalg.norm(velocity, axis=1)
        plt.plot(time_stamp, speed, linewidth=2)
    
    plt.xlim(time_stamp[0], time_stamp[-1])
    plt.ylim(0,VMAX+0.1)
    plt.xlabel('time (s)')
    plt.ylabel('speed (m/s)')
    plt.tight_layout()

def plot_mode():
    plt.figure(figsize=(6,3))
    size = 10
    path = data[0]['path']
    x = np.arange(0, path.shape[0], size)
    num_state = 0
    for i in range(NUM_ROBOT):
        num_state += data[i]['path'][:,10]
    num_state = num_state[x]

    plt.bar(path[:,0][x], num_state, label="Tailgating")
    plt.bar(path[:,0][x], NUM_ROBOT-num_state, bottom=num_state, label="Formation")
    plt.xlabel("Time (s)")
    plt.ylabel("Number of Robot")
    plt.xlim([-1, path[-1,0]+1])
    plt.ylim([0, NUM_ROBOT])
    plt.tight_layout()
    plt.legend()

def plot_order():
    plt.figure(figsize=(6,3))
    path = data[0]['path']
    headings = []
    for i in range(1,len(path)):
        heading = 0
        for j in range(NUM_ROBOT):
            heading += data[j]['path'][i,4:6]/np.linalg.norm(data[j]['path'][i,4:6])
        headings.append(np.linalg.norm(heading)/NUM_ROBOT)
    plt.plot(path[1:,0], headings, 'b-')
    plt.xlabel("Time (s)")
    plt.ylabel("Order")
    plt.xlim([0, path[-1,0]])
    plt.ylim(0,1.1)
    plt.tight_layout()
    plt.grid(True)

plot_path()
plot_speed()
# plot_mode()
plot_order()
plt.show()