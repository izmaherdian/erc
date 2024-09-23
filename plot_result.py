import numpy as np
import pickle
import matplotlib.pyplot as plt
import matplotlib.cm as cmap
from matplotlib.patches import Polygon
from config import *

def get_circle(x,y,r):
    theta = np.linspace(0, 2 * np.pi, 20)
    a = x + r * np.cos( theta )
    b = y + r * np.sin( theta )
    return a, b

shape = 1
controller = 'edc'
file_name = "results/data_{}_shape{}.txt".format(controller, shape)
with open(file_name, 'rb') as file:
    data = pickle.load(file)

def plot_path():
    fig = plt.figure(figsize=(10,3))
    ax = fig.subplots()

    # Plot obstacles
    for i in range(len(OBSTACLES)):
        ax.add_patch(Polygon(OBSTACLES[i],color='dimgrey'))

    for i in range(NUM_ROBOT):
        path = data[i]['path']
        velocity = path[:,4:6]
        speed = np.linalg.norm(velocity, axis=1)
        pp = ax.scatter(path[:,1], path[:,2], c=speed, cmap=cmap.summer, marker='.', s=20)

    fig.colorbar(pp, ax=ax, orientation="horizontal", fraction=0.1, label="Speed (m/s)")

    # Plot formation
    n_plot = 6
    length = path.shape[0]
    for i in range(n_plot+1):
        iter = int(round(length/n_plot)*i)
        if iter >= length:
            iter = length-1

        # Plot robots
        for i in range(NUM_ROBOT):
            a, b = get_circle(data[i]['path'][iter,1], data[i]['path'][iter,2], ROBOT_RADIUS)
            plt.plot(a, b, '-b')

        positions = []
        for i in range(NUM_ROBOT):
            positions.append(data[i]['path'][iter,1:4])

        # Plot current formation
        positions.append(positions[0])
        formation = np.array(positions)
        plt.plot(formation[:,0], formation[:,1], '-k')

    plt.axis('scaled')
    plt.xlim(XLIM)
    plt.ylim(YLIM)
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.tight_layout()
    plt.savefig("results/path_{}_shape{}.pdf".format(controller, shape), format="pdf", bbox_inches="tight")

def plot_mode():
    plt.figure(figsize=(6,2.5))
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
    plt.savefig("results/mode_{}_shape{}.pdf".format(controller, shape), format="pdf", bbox_inches="tight")

# plot_path()
plot_mode()
plt.show()