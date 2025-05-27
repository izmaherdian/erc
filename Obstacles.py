import numpy as np

from mpl_toolkits.mplot3d.art3d import Poly3DCollection

class Obstacles:
    def __init__(self, scheme='scheme1'):
        # Pilih skema obstacles
        if scheme == 'scheme1':
            self.obstacles_2d = [
                np.array([[1.0, -0.5], [2.0, -0.5], [2.0, -0.7], [1.0, -0.7]]),
                np.array([[3.0, -3.0], [4.0, -3.0], [4.0, -4.0], [3.0, -4.0]])
            ]
            self.obstacle_heights = [
                (0, 2),
                (0, 3)
            ]

        elif scheme == 'scheme2':
            self.obstacles_2d = [
                np.array([[0.0, 0.0], [1.5, 0.0], [1.5, 1.5], [0.0, 1.5]]),
                np.array([[2.0, 1.0], [3.0, 1.0], [3.0, 2.0], [2.0, 2.0]]),
                np.array([[1.0, 3.0], [2.0, 3.0], [1.5, 4.0]])
            ]
            self.obstacle_heights = [
                (0, 1),
                (0, 1.5),
                (0, 2)
            ]

        elif scheme == 'scheme3':
            self.obstacles_2d = [
                np.array([[ -1.0, -1.0], [0.0, -1.0], [0.0, 0.0], [-1.0, 0.0]]),
            ]
            self.obstacle_heights = [
                (0, 4)
            ]

        else:
            # Default empty obstacles
            self.obstacles_2d = []
            self.obstacle_heights = []

    def plot_prism(self, ax, polygon_2d, z_min=0, z_max=3, color='gray', alpha=0.3, edge_color='k'):
        xs = [p[0] for p in polygon_2d]
        ys = [p[1] for p in polygon_2d]

        verts_bottom = list(zip(xs, ys, [z_min]*len(xs)))
        verts_top = list(zip(xs, ys, [z_max]*len(xs)))

        ax.add_collection3d(Poly3DCollection([verts_bottom], facecolors=color, alpha=alpha))
        ax.add_collection3d(Poly3DCollection([verts_top], facecolors=color, alpha=alpha))

        for i in range(len(xs)):
            j = (i + 1) % len(xs)
            side = [verts_bottom[i], verts_bottom[j], verts_top[j], verts_top[i]]
            ax.add_collection3d(Poly3DCollection([side], facecolors=color, alpha=alpha))

        for i in range(len(xs)):
            j = (i + 1) % len(xs)
            ax.plot([verts_bottom[i][0], verts_bottom[j][0]], [verts_bottom[i][1], verts_bottom[j][1]], [verts_bottom[i][2], verts_bottom[j][2]], color=edge_color, alpha=0.5)
            ax.plot([verts_top[i][0], verts_top[j][0]], [verts_top[i][1], verts_top[j][1]], [verts_top[i][2], verts_top[j][2]], color=edge_color, alpha=0.5)
            ax.plot([verts_bottom[i][0], verts_top[i][0]], [verts_bottom[i][1], verts_top[i][1]], [verts_bottom[i][2], verts_top[i][2]], color=edge_color, alpha=0.5)

    def draw(self, ax):
        for i, obstacle in enumerate(self.obstacles_2d):
            z_min, z_max = self.obstacle_heights[i]
            self.plot_prism(ax, obstacle, z_min=z_min, z_max=z_max, color='gray', alpha=0.3, edge_color='k')
