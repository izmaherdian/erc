import numpy as np

TIMESTEP = 0.05
ROBOT_RADIUS = 0.3
ALERT_RADIUS = 3*ROBOT_RADIUS
SENSING_RADIUS = 5.0
ITER_MAX = 1000

EPSILON = 0.1
ALPHA = 8

VREF = 0.5
VMAX = 1.0
UMAX = 2.0
UREF = np.array([1,0,0])
DREF = 1.0

W_form = 1.0
W_tail = 1.0
W_obs = 8.0
W_col = 8.0
W_rand = 2e-2

XLIM = [-20., 25.]
YLIM = [  0., 6.5]
SIZE = [10, 3]

INITS = np.array([[-18.0, 3.0, 0.],
                  [-18.4, 8.0, 0.],
                  [-18.7, 6.0, 0.],
                  [-18.7, 0.0, 0.],
                  [-18.4, -2.0, 0.]])

XGOAL = -18.0
YGOAL = 3.0

NUM_ROBOT = INITS.shape[0]

# Formation description
TYPE = 2
if TYPE == 1: # Polygon
    TOPOLOGY = []
    for i in range(NUM_ROBOT):
        TOPOLOGY.append([np.cos(2*np.pi/NUM_ROBOT*i), np.sin(2*np.pi/NUM_ROBOT*i), 0.0])
    TOPOLOGY = np.array(TOPOLOGY)
elif TYPE == 2: # V shape
    TOPOLOGY = np.array([[ 1.0, 0.0, 0.0],
                         [-1.0, 1.0, 0.0],
                         [ 0.0, 0.5, 0.0],
                         [ 0.0,-0.5, 0.0],
                         [-1.0,-1.0, 0.0]])

CONTROLLER = 'iapf'  # erc, bc, iapf
# Create obstacles
OBSTACLES = [
    # Forest-like
    [[-5.0, 3.0], [-5.5, 3.5], [-4.5, 4.0], [-4.0, 3.0]],
    [[-10.0, 0.7], [-9.5, 1.5], [-8.5, 1.9], [-9.0, 0.7]],
    [[-12.0, 5.0], [-11.5, 4.5], [-10.5, 4.7], [-11.5, 5.5]],
    [[-3.0, 0.5], [-2.5, 1.0], [-1.5, 1.2], [-2.0, 0.1]],
    # [[-17.4, 2.0], [-17.4, 2.5], [-15.9, 2.5], [-15.9, 2.0]],

    # Narrow gap
    [[0.0, 0.0], [ 5.0, 3.0], [15.0, 3.0], [20.0, 0.0]],
    [[0.0, 6.5], [10.0, 4.0], [15.0, 4.0], [20.0, 6.5]]
]

FILE_NAME = "results/data_{}_shape{}.txt".format(CONTROLLER, TYPE)
GIF_NAME = "results/gif_{}_shape{}.gif".format(CONTROLLER, TYPE)