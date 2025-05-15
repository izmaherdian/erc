import pickle
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import numpy as np
from config import *

def get_circle(x,y,r):
    theta = np.linspace( 0 , 2 * np.pi , 150 )   
    a = x + r * np.cos( theta )
    b = y + r * np.sin( theta )
    return a, b

percent = 0.3
width = 0.02
export = False
if export:
    import cv2
    image_array = []

with open(FILE_NAME, 'rb') as file:
    data = pickle.load(file)
length = data[0]['path'].shape[0]

## Plot motion paths
plt.figure(figsize=(8,5))
ax = plt.axes()
for iter in range(0,length,2):
    ax.cla()

    # Plot obstacles
    for i in range(len(OBSTACLES)):
        ax.add_patch(Polygon(OBSTACLES[i],color='grey'))

    for i in range(NUM_ROBOT):
        path = data[i]['path']
        # Plot path
        ax.plot(path[:iter,1], path[:iter,2], label="Robot {}".format(i))

        # Plot robot
        pose = data[i]['path'][iter,:]
        a, b = get_circle(pose[1], pose[2], ROBOT_RADIUS)
        ax.plot(a, b, '-k')

        # Plot motion direction
        plt.arrow(pose[1], pose[2],
                  pose[4]*percent, pose[5]*percent,
                  width=width, color='k')

    ax.axis('scaled')
    ax.grid(True)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    plt.legend()

    # find center
    center = 0
    for i in range(NUM_ROBOT):
        center +=  data[i]['path'][iter,1:3]
    center /= NUM_ROBOT
    plt.xlim([center[0]-5, center[0]+5])
    plt.ylim([center[1]-3, center[1]+3])
    plt.tight_layout()
    
    plt.gcf().canvas.mpl_connect('key_release_event',
                                        lambda event:
                                        [exit(0) if event.key == 'escape' else None])
    plt.pause(0.001)

    if export:
        file_name = "results/data.png"
        plt.savefig(file_name)
        img = cv2.imread(file_name)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        image_array.append(img)
if export:
    import imageio
    imageio.mimsave(GIF_NAME, image_array)

plt.show()