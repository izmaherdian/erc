import numpy as np
import pickle
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import matplotlib.cm as cmap
from config import *

with open(FILE_NAME, 'rb') as file:
    data = pickle.load(file)

def plot_path():
    fig = plt.figure(figsize=SIZE)
    ax = fig.subplots()

    # Plot obstacles
    for i in range(len(OBSTACLES)):
        ax.add_patch(Polygon(OBSTACLES[i],color='dimgrey'))

    # Plot start and goal
    # plt.scatter(INIT[0], INIT[1], marker='s', s= 50)
    # plt.scatter(GOAL[0], GOAL[1], marker='^', s= 50)

    for i in range(NUM_ROBOT):
        path = data[i]['path']
        velocity = path[:,4:6]
        speed = np.linalg.norm(velocity, axis=1)
        pp = ax.scatter(path[:,1], path[:,2], c=speed, cmap=cmap.summer, marker='.', s=20)

    fig.colorbar(pp, ax=ax, orientation="horizontal", fraction=0.1)
    plt.axis('scaled')
    plt.xlim(XLIM)
    plt.ylim(YLIM)
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.tight_layout()

plot_path()
plt.show()