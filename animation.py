import pickle
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import numpy as np
from config_traj import *
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def plot_prism(ax, polygon_2d, z_min=0, z_max=3, color='gray', alpha=0.3, edge_color='k'):
    xs = [p[0] for p in polygon_2d]
    ys = [p[1] for p in polygon_2d]

    verts_bottom = list(zip(xs, ys, [z_min]*len(xs)))
    verts_top = list(zip(xs, ys, [z_max]*len(xs)))

    # Plot faces
    ax.add_collection3d(Poly3DCollection([verts_bottom], facecolors=color, alpha=alpha))
    ax.add_collection3d(Poly3DCollection([verts_top], facecolors=color, alpha=alpha))

    # Plot side faces
    for i in range(len(xs)):
        j = (i + 1) % len(xs)
        side = [verts_bottom[i], verts_bottom[j], verts_top[j], verts_top[i]]
        ax.add_collection3d(Poly3DCollection([side], facecolors=color, alpha=alpha))

    # Plot edges bottom
    for i in range(len(xs)):
        j = (i + 1) % len(xs)
        x_vals = [verts_bottom[i][0], verts_bottom[j][0]]
        y_vals = [verts_bottom[i][1], verts_bottom[j][1]]
        z_vals = [verts_bottom[i][2], verts_bottom[j][2]]
        ax.plot(x_vals, y_vals, z_vals, color=edge_color)

    # Plot edges top
    for i in range(len(xs)):
        j = (i + 1) % len(xs)
        x_vals = [verts_top[i][0], verts_top[j][0]]
        y_vals = [verts_top[i][1], verts_top[j][1]]
        z_vals = [verts_top[i][2], verts_top[j][2]]
        ax.plot(x_vals, y_vals, z_vals, color=edge_color)

    # Plot vertical edges
    for i in range(len(xs)):
        x_vals = [verts_bottom[i][0], verts_top[i][0]]
        y_vals = [verts_bottom[i][1], verts_top[i][1]]
        z_vals = [verts_bottom[i][2], verts_top[i][2]]
        ax.plot(x_vals, y_vals, z_vals, color=edge_color)


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
plt.figure(figsize=(10,7))
ax = plt.axes(projection='3d')

center_trajectory = []

color_list = ['blue', 'orange', 'green', 'red', 'purple', 'cyan']

for iter in range(0,length,2):
    ax.cla()

    # Plot obstacles
    for i, obstacle in enumerate(OBSTACLES):
        z_min, z_max = OBSTACLE_HEIGHTS[i]
        plot_prism(ax, obstacle, z_min=z_min, z_max=z_max, color='gray', alpha=0.3, edge_color='k')

    for i in range(NUM_ROBOT):
        color = color_list[i % len(color_list)]
        path = data[i]['path']
        ax.plot(path[:iter,1], path[:iter,2], path[:iter,3], color=color, label=f"Robot {i}")

        pose = data[i]['path'][iter,:]
        ax.scatter(pose[1], pose[2], pose[3], s=50, c=color)

        ax.quiver(pose[1], pose[2], pose[3],
                  pose[4]*percent, pose[5]*percent, pose[6]*percent,
                  length=0.5, normalize=True, color='k')
        
    # Calculate and plot center
    center = np.zeros(3)
    for i in range(NUM_ROBOT):
        pose = data[i]['path'][iter,:]
        center += pose[1:4]
    center /= NUM_ROBOT
    center_trajectory.append(center)
    center_arr = np.array(center_trajectory)

    ax.scatter(center[0], center[1], center[2], c='r', s=50, label='Center')
    ax.plot(center_arr[:, 0], center_arr[:, 1], center_arr[:, 2], linestyle='--', color='cyan', label='Center Path')

    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    ax.grid(True)
    ax.legend()

    # Plot limit
    ax.set_xlim(center[0]-7, center[0]+7)
    ax.set_ylim(center[1]-5, center[1]+5)
    ax.set_zlim(0, 5)  # Contoh batas ketinggian

    plt.tight_layout()

    plt.gcf().canvas.mpl_connect('key_release_event',
                                lambda event: [exit(0) if event.key == 'escape' else None])
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