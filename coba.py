import numpy as np
import math
from enum import Enum
import pickle
# Jika menggunakan matplotlib di Animator, import di sana
# from mpl_toolkits import mplot3d
# import matplotlib.pyplot as plt
# from matplotlib.patches import Polygon
# from mpl_toolkits.mplot3d.art3d import Poly3DCollection
# Jika menggunakan cv2 di Animator, import di sana
# import cv2
# import imageio

# --- Import dari file Anda yang lain ---
from Obstacles import Obstacles # Asumsikan Obstacles.py ada di direktori yang sama atau dapat diakses

# Fungsi bantuan untuk mendapatkan data agents_config yang mirip dengan main.py
# Ini untuk mensimulasikan impor konfigurasi ini dari main.py
def get_main_agents_config():
    # Ini adalah replikasi dari agents_config dari fungsi main_multi_agent di main.py
    agents_config = [
        {
            'id': 1,
            'init_pos': [1.0, 1.0, 0.0], # Menggunakan float untuk konsistensi
            'front_color': 'orange',
            'ctrlType': "xyz_pos",
            'trajSelect': (0, 3, 0),
            # 'wayPoints': wayPoints1() # wayPoints tidak digunakan secara langsung untuk INITS di sini
        },
        {
            'id': 2,
            'init_pos': [-3.0, 3.0, 0.0],
            'front_color': 'blue',
            'ctrlType': "xyz_pos",
            'trajSelect': (0, 3, 0),
            # 'wayPoints': wayPoints2()
        },
        {
            'id': 3,
            'init_pos': [-2.0, 2.0, 0.0],
            'front_color': 'green',
            'ctrlType': "xyz_pos",
            'trajSelect': (0, 3, 0),
            # 'wayPoints': wayPoints1()
        },
        {
            'id': 4,
            'init_pos': [-2.0, -2.0, 0.0],
            'front_color': 'red',
            'ctrlType': "xyz_pos",
            'trajSelect': (0, 3, 0),
            # 'wayPoints': wayPoints2()
        },
        {
            'id': 5,
            'init_pos': [-3.0, -3.0, 0.0],
            'front_color': 'purple',
            'ctrlType': "xyz_pos",
            'trajSelect': (0, 3, 0),
            # 'wayPoints': wayPoints1()
        }
    ]
    return agents_config

# --- Dari utils.py ---
def perpendicular(x:np.array, a:np.array, b:np.array):
    d_ab = np.linalg.norm(a-b)
    d_ax = np.linalg.norm(a-x)
    d_bx = np.linalg.norm(b-x)

    if d_ab != 0:
        if np.dot(a-b,x-b)*np.dot(b-a,x-a) >= 0: # x berada di antara a dan b
            px = b[0]-a[0]; py = b[1]-a[1]; dAB = px*px + py*py
            u = ((x[0] - a[0]) * px + (x[1] - a[1]) * py) / dAB
            p = [a[0] + u * px, a[1] + u * py]
        else:
            if d_ax < d_bx:
                p = a
            else:
                p = b
    else: # jika a dan b identik
        p = a
    return p

def nearest_point_to_obstacle(pose, obstacle_geometry):
    """
    Menemukan titik terdekat pada sebuah obstacle (poligon 2D) dari sebuah pose (titik 2D).
    `obstacle_geometry` adalah list dari titik-titik poligon.
    """
    nearest_point = []
    nearest_dis= float(np.inf)
    for i in range(len(obstacle_geometry)):
        per = perpendicular(pose, np.array(obstacle_geometry[i]), np.array(obstacle_geometry[(i+1) % len(obstacle_geometry)]))
        dis_per = np.linalg.norm(pose - per)
        if dis_per < nearest_dis:
            nearest_dis = dis_per
            nearest_point = per
    return nearest_point

class Config:
    def __init__(self):
        self.TIMESTEP = 0.05
        self.ROBOT_RADIUS = 0.2
        self.ALERT_RADIUS = 3 * self.ROBOT_RADIUS
        self.SENSING_RADIUS = 3.0
        self.ITER_MAX = 2000

        self.EPSILON = 0.1
        self.ALPHA = 8

        self.VREF = 0.5
        self.VMAX = 1.0
        self.UMAX = 2.0
        self.UREF = np.array([1,0,0])
        self.DREF = 1.0

        self.W_form = 1.0
        self.W_tail = 1.0
        self.W_obs = 8.0
        self.W_col = 8.0
        self.W_rand = 2e-2

        self.XLIM = [-20., 25.]
        self.YLIM = [  -5., 8.] # Disesuaikan agar lebih cocok dengan obstacle scheme1 dan area simulasi umum
        self.SIZE = [10, 3]

        # Impor INITS dari struktur agents_config main.py
        agents_config_data = get_main_agents_config()
        self.INITS = np.array([config_item['init_pos'] for config_item in agents_config_data])

        self.NUM_ROBOT = self.INITS.shape[0] # Atur NUM_ROBOT secara dinamis

        self.XGOAL = 25.0
        self.YGOAL = 3.0
        self.ZGOAL = 1.0 # Pastikan ZGOAL konsisten atau disesuaikan dengan skenario 3D

        # Deskripsi Formasi
        self.FORMATION_TYPE = 2 # 1 untuk Polygon, 2 untuk V shape
        if self.FORMATION_TYPE == 1: # Polygon
            self.TOPOLOGY = []
            for i in range(self.NUM_ROBOT):
                self.TOPOLOGY.append([np.cos(2*np.pi/self.NUM_ROBOT*i), np.sin(2*np.pi/self.NUM_ROBOT*i), 0.0])
            self.TOPOLOGY = np.array(self.TOPOLOGY)
        elif self.FORMATION_TYPE == 2: # V shape
            if self.NUM_ROBOT == 5: # Periksa apakah V-shape saat ini cocok
                self.TOPOLOGY = np.array([[ 1.0, 0.0, 0.0],
                                         [-1.0, 1.0, 0.0],
                                         [ 0.0, 0.5, 0.0],
                                         [ 0.0,-0.5, 0.0],
                                         [-1.0,-1.0, 0.0]])
            else:
                # Fallback untuk V-shape jika NUM_ROBOT bukan 5 (misalnya, formasi garis)
                print(f"Peringatan: Topologi V-shape telah ditentukan untuk 5 robot. NUM_ROBOT adalah {self.NUM_ROBOT}. Menyesuaikan topologi.")
                self.TOPOLOGY = []
                for i in range(self.NUM_ROBOT):
                    self.TOPOLOGY.append([float(-i) * 0.5, 0.0, 0.0]) # Garis sederhana
                self.TOPOLOGY = np.array(self.TOPOLOGY)


        self.CONTROLLER = 'iapf'  # erc, bc, iapf
        
        # Impor OBSTACLE_GEOMETRIES dan OBSTACLE_HEIGHTS dari Obstacles.py
        # Anda dapat memilih skema, misalnya, 'scheme1', 'scheme2', dll.
        obstacle_provider = Obstacles(scheme='scheme1') 
        # Konversi array numpy dari Obstacles.py menjadi list of lists seperti yang diharapkan oleh logika coba.py
        self.OBSTACLE_GEOMETRIES = [obs.tolist() for obs in obstacle_provider.obstacles_2d]
        self.OBSTACLE_HEIGHTS = list(obstacle_provider.obstacle_heights)

        self.FILE_NAME = f"results/coba_data_{self.CONTROLLER}_shape{self.FORMATION_TYPE}.txt"
        self.GIF_NAME = f"results/coba_gif_{self.CONTROLLER}_shape{self.FORMATION_TYPE}.gif"

class Mode(Enum):
    FORMATION = 0
    TAILGATING = 1

class RobotIAPF:
    def __init__(self, index, position, config, velocity=np.zeros(3)):
        self.index = index
        self.config = config # Menyimpan referensi ke konfigurasi

        self.stamp = 0.0
        self.position = np.array(position, dtype=float)
        self.velocity = np.array(velocity, dtype=float)
        self.control = np.zeros(3, dtype=float)

        self.mode = Mode.FORMATION

        self.path = [np.concatenate([[self.stamp], self.position, self.velocity, self.control, [self.mode.value]])]

    def update_state(self, control, dt):
        control_norm = np.linalg.norm(control)
        if control_norm > self.config.UMAX:
            control = control/control_norm*self.config.UMAX

        velocity = self.velocity + control*dt
        velocity_norm = np.linalg.norm(velocity)
        if velocity_norm > self.config.VMAX:
            velocity = velocity/velocity_norm*self.config.VMAX

        position = self.position + velocity*dt

        self.stamp += dt
        self.position = position
        self.velocity = velocity
        self.control = control
        self.path.append(np.concatenate([[self.stamp], self.position, self.velocity, self.control, [self.mode.value]]))

    def compute_control(self, robots, environment, dt):
        v_mig = self.behavior_migration(robots)
        v_obs = self.behavior_obstacle(environment)
        v_col = self.behavior_collision(robots)
        v_form = self.behavior_formation(robots)
        v_rand = self.behavior_random()
        desired_velocity = v_mig + v_form + v_obs + v_col + v_rand

        desired_control = (desired_velocity - self.velocity)/dt
        self.update_state(desired_control, dt)

    def behavior_migration(self, robots):
        center = np.zeros(3)
        for robot in robots:
            center += robot.position
        center /= self.config.NUM_ROBOT

        goal = np.array([self.config.XGOAL, self.config.YGOAL, self.config.ZGOAL])
        error_vec = goal - center
        error_dist = np.linalg.norm(error_vec)

        max_speed = self.config.VREF
        slow_down_radius = 1.0  # jarak mulai melambat

        if error_dist < slow_down_radius and error_dist > 0:
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
            direction = np.array([0.0, 0.0, 0.0])
        velocity = speed * direction
        return velocity
    
    def behavior_formation(self, robots):
        v_form = np.zeros(3) # ; Inisialisasi sebagai array numpy
        for i in range(self.config.NUM_ROBOT):
            # Pastikan TOPOLOGY diakses dengan benar dari config
            v_form += (robots[i].position - self.position) - \
                      (self.config.TOPOLOGY[i,:] - self.config.TOPOLOGY[self.index,:])
        return self.config.W_form * v_form
    
    def behavior_obstacle(self, environment):
        v_obs = np.zeros(3) # ; Inisialisasi sebagai array numpy
        for obstacle in environment.obstacles: 
            # obstacle sekarang adalah instance dari kelas Obstacle
            obs_point = nearest_point_to_obstacle(self.position[:2], obstacle.geometry) 

            z_rel = 0
            if self.position[2] < obstacle.z_min: 
                z_rel = obstacle.z_min - self.position[2] 
            elif self.position[2] > obstacle.z_max: 
                z_rel = self.position[2] - obstacle.z_max 

            obs_rel_xy = self.position[:2] - obs_point
            horizontal_dist = np.linalg.norm(obs_rel_xy)

            if horizontal_dist == 0 and z_rel == 0:
                obs_dis = 1e-6 
            elif horizontal_dist == 0:
                obs_dis = abs(z_rel)
            else:
                obs_dis = np.sqrt(horizontal_dist**2 + z_rel**2)


            if obs_dis < self.config.ALERT_RADIUS and obs_dis > 1e-6: # Menambah batas bawah untuk obs_dis
                direction_xy = obs_rel_xy / horizontal_dist if horizontal_dist != 0 else np.zeros(2)
                magnitude = 0.5 * (1/obs_dis - 1/self.config.ALERT_RADIUS) / (obs_dis**2) 
                v_obs_component = magnitude * np.array([direction_xy[0], direction_xy[1], 0]) 

                if not np.any(np.isnan(v_obs_component)) and not np.any(np.isinf(v_obs_component)):
                    v_obs += v_obs_component
                else:
                    if horizontal_dist > 1e-6:
                         v_obs += 0.1 * np.array([direction_xy[0], direction_xy[1], 0]) / horizontal_dist 
        return self.config.W_obs * v_obs
    
    def behavior_collision(self, robots):
        v_col = np.zeros(3) # ; Inisialisasi sebagai array numpy
        for i in range(self.config.NUM_ROBOT):
            if i == self.index:
                continue
            pos_rel = self.position - robots[i].position
            pos_dis = np.linalg.norm(pos_rel)
            if pos_dis < self.config.ALERT_RADIUS and pos_dis > 1e-6: # Menambah batas bawah untuk pos_dis
                v_col_component = 2*(1/pos_dis - 1/self.config.ALERT_RADIUS)/(pos_dis**2)*pos_rel/pos_dis
                if not np.any(np.isnan(v_col_component)) and not np.any(np.isinf(v_col_component)):
                     v_col += v_col_component
        return self.config.W_col*v_col

    def behavior_random(self):
        return self.config.W_rand*np.concatenate([np.random.rand(2),[0]])


class Obstacle: # Ini adalah kelas Obstacle yang digunakan oleh Environment di coba.py
    def __init__(self, geometry, height_range):
        self.geometry = geometry # list of [x,y] points
        self.z_min = height_range[0]
        self.z_max = height_range[1]

class Environment:
    def __init__(self, config):
        self.config = config
        self.robots = []
        self.obstacles = []
        self._initialize_obstacles()

    def _initialize_obstacles(self):
        # Menggunakan OBSTACLE_GEOMETRIES dan OBSTACLE_HEIGHTS dari Config, sekarang bersumber dari Obstacles.py
        for geo, height in zip(self.config.OBSTACLE_GEOMETRIES, self.config.OBSTACLE_HEIGHTS): 
            self.obstacles.append(Obstacle(geo, height)) # Membuat instance dari kelas Obstacle lokal

    def add_robot(self, robot):
        self.robots.append(robot)

class DataLogger:
    def __init__(self, file_name):
        self.file_name = file_name

    def save_data(self, robots):
        with open(self.file_name, 'wb') as file:
            data = []
            for i in range(len(robots)):
                d = dict()
                d['path'] = np.array(robots[i].path)
                data.append(d)
            pickle.dump(data, file)
        print(f"Data disimpan ke {self.file_name}")

class Simulation:
    def __init__(self, config, environment):
        self.config = config
        self.environment = environment
        self.robots = []
        self._initialize_robots()
        self.logger = DataLogger(self.config.FILE_NAME)

    def _initialize_robots(self):
        for i in range(self.config.NUM_ROBOT): # NUM_ROBOT sekarang berdasarkan INITS yang diimpor
            if self.config.CONTROLLER == 'iapf':
                robot = RobotIAPF(i, self.config.INITS[i,:], self.config) # INITS sekarang diimpor
            # elif self.config.CONTROLLER == 'erc':
            #     robot = RobotERC(i, self.config.INITS[i,:], self.config)
            # elif self.config.CONTROLLER == 'bc':
            #     robot = RobotBC(i, self.config.INITS[i,:], self.config)
            else:
                raise ValueError(f"Tipe controller tidak diketahui: {self.config.CONTROLLER}")
            self.robots.append(robot)
            self.environment.add_robot(robot) 

    def check_collision(self):
        # Tabrakan dengan obstacles
        for i in range(self.config.NUM_ROBOT):
            robot_pos = self.robots[i].position
            for obstacle_obj in self.environment.obstacles: # Obstacles ini sekarang dari Obstacles.py via Config & Environment
                if not (robot_pos[2] + self.config.ROBOT_RADIUS < obstacle_obj.z_min or \
                        robot_pos[2] - self.config.ROBOT_RADIUS > obstacle_obj.z_max):
                    obs_point_2d = nearest_point_to_obstacle(robot_pos[:2], obstacle_obj.geometry)
                    if np.linalg.norm(robot_pos[:2]-obs_point_2d) < self.config.ROBOT_RADIUS:
                        print(f"Tabrakan: Robot {i} dengan obstacle")
                        return True
            
        # Tabrakan antar agen
        for i in range(self.config.NUM_ROBOT-1):
            pi = self.robots[i].position
            for j in range(i+1, self.config.NUM_ROBOT):
                pj = self.robots[j].position
                if np.linalg.norm(pi - pj) < 2*self.config.ROBOT_RADIUS:
                    print(f"Tabrakan: Robot {i} dengan Robot {j}")
                    return True
        return False

    def check_reach_goal(self, prev_center, stable_count):
        center = np.zeros(3)
        velocity_sum = np.zeros(3)
        for robot in self.robots:
            center += robot.position
            velocity_sum += robot.velocity
        center /= self.config.NUM_ROBOT
        avg_velocity = np.linalg.norm(velocity_sum) / self.config.NUM_ROBOT

        goal = np.array([self.config.XGOAL, self.config.YGOAL, self.config.ZGOAL])
        dist_to_goal = np.linalg.norm(center - goal)

        threshold = 0.1 
        velocity_threshold = 0.1 
        position_delta_threshold = 0.1 
        STABLE_THRESHOLD = 10 

        new_stable_count = stable_count
        goal_reached = False

        if dist_to_goal < threshold and avg_velocity < velocity_threshold:
            if prev_center is not None and np.linalg.norm(center - prev_center) < position_delta_threshold:
                new_stable_count += 1
            else:
                new_stable_count = 0
            if new_stable_count >= STABLE_THRESHOLD:
                print("Formasi sudah stabil di goal. Simulasi berhenti.")
                goal_reached = True
        else:
            new_stable_count = 0
        
        return goal_reached, center.copy(), new_stable_count


    def run(self):
        prev_center = None
        stable_count = 0
        
        iter_count = 0
        while iter_count < self.config.ITER_MAX:
            iter_count += 1
            if iter_count % 100 == 0:
                 print(f"Iterasi {iter_count}")

            for robot in self.robots:
                robot.compute_control(self.robots, self.environment, self.config.TIMESTEP)

            if self.check_collision():
                print("Tabrakan terdeteksi. Simulasi berhenti.")
                break

            goal_reached, current_center, stable_count = self.check_reach_goal(prev_center, stable_count)
            prev_center = current_center
            if goal_reached:
                break
            
        self.logger.save_data(self.robots)
        print(f"Simulasi selesai setelah {iter_count} iterasi.")


class Animator:
    def __init__(self, config):
        self.config = config
        self.data = None
        self._load_data()

    def _load_data(self):
        try:
            with open(self.config.FILE_NAME, 'rb') as file:
                self.data = pickle.load(file)
        except FileNotFoundError:
            print(f"Error: File data {self.config.FILE_NAME} tidak ditemukan. Jalankan simulasi terlebih dahulu.")
            self.data = None 
            return 
        except Exception as e:
            print(f"Error memuat data: {e}")
            self.data = None
            return


    def _plot_prism(self, ax, polygon_2d, z_min=0, z_max=3, color='gray', alpha=0.3, edge_color='k'):
        """Fungsi bantuan untuk memplot prisma 3D."""
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection # Import di dalam method

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
            ax.plot([verts_bottom[i][0], verts_bottom[j][0]],
                    [verts_bottom[i][1], verts_bottom[j][1]],
                    [verts_bottom[i][2], verts_bottom[j][2]], color=edge_color)
            ax.plot([verts_top[i][0], verts_top[j][0]],
                    [verts_top[i][1], verts_top[j][1]],
                    [verts_top[i][2], verts_top[j][2]], color=edge_color)
            ax.plot([verts_bottom[i][0], verts_top[i][0]],
                    [verts_bottom[i][1], verts_top[i][1]],
                    [verts_bottom[i][2], verts_top[i][2]], color=edge_color)


    def animate(self, export_gif=False):
        import matplotlib.pyplot as plt
        from mpl_toolkits import mplot3d # mplot3d perlu diimport untuk projection='3d'

        if self.data is None:
            print("Tidak ada data untuk dianimasikan.")
            return

        length = self.data[0]['path'].shape[0]
        fig = plt.figure(figsize=(10,7))
        ax = plt.axes(projection='3d')

        center_trajectory = []
        color_list = ['blue', 'orange', 'green', 'red', 'purple', 'cyan']
        
        image_array = []
        if export_gif:
            import cv2 

        for iter_idx in range(0, length, 2): 
            ax.cla()

            # Plot obstacles menggunakan OBSTACLE_GEOMETRIES dan OBSTACLE_HEIGHTS dari config
            # Ini sekarang bersumber dari Obstacles.py
            for i, obstacle_geo in enumerate(self.config.OBSTACLE_GEOMETRIES): 
                z_min, z_max = self.config.OBSTACLE_HEIGHTS[i] 
                self._plot_prism(ax, obstacle_geo, z_min=z_min, z_max=z_max, color='gray', alpha=0.3, edge_color='k')

            current_center_pos = np.zeros(3)
            for i in range(self.config.NUM_ROBOT): # NUM_ROBOT dari config
                color = color_list[i % len(color_list)]
                path = self.data[i]['path']
                ax.plot(path[:iter_idx+1,1], path[:iter_idx+1,2], path[:iter_idx+1,3], color=color, label=f"Robot {i}" if iter_idx==0 else "")
                
                pose = path[iter_idx,:]
                ax.scatter(pose[1], pose[2], pose[3], s=50, c=color)

                percent = 0.3
                ax.quiver(pose[1], pose[2], pose[3],
                          pose[4]*percent, pose[5]*percent, pose[6]*percent,
                          length=0.5, normalize=True, color='k')
                
                current_center_pos += pose[1:4]
            
            current_center_pos /= self.config.NUM_ROBOT
            center_trajectory.append(current_center_pos)
            center_arr = np.array(center_trajectory)

            ax.scatter(current_center_pos[0], current_center_pos[1], current_center_pos[2], c='r', s=50, label='Center' if iter_idx==0 else "")
            if len(center_arr) > 1:
                 ax.plot(center_arr[:, 0], center_arr[:, 1], center_arr[:, 2], linestyle='--', color='cyan', label='Center Path' if iter_idx==0 else "")


            ax.set_xlabel('X [m]')
            ax.set_ylabel('Y [m]')
            ax.set_zlabel('Z [m]')
            ax.grid(True)
            if iter_idx == 0 : ax.legend()

            ax.set_xlim(current_center_pos[0]-7, current_center_pos[0]+7)
            ax.set_ylim(current_center_pos[1]-5, current_center_pos[1]+5)
            ax.set_zlim(0, 5)  # Sesuaikan batas Z sesuai kebutuhan berdasarkan obstacles/skenario baru

            plt.tight_layout()
            plt.pause(0.001)

            if export_gif:
                import os
                if not os.path.exists("results"):
                    os.makedirs("results")
                temp_file_name = "results/_temp_frame.png"
                plt.savefig(temp_file_name)
                img = cv2.imread(temp_file_name)
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                image_array.append(img)
        
        if export_gif and image_array:
            import imageio
            imageio.mimsave(self.config.GIF_NAME, image_array, fps=10)
            print(f"GIF disimpan ke {self.config.GIF_NAME}")

        plt.show()


if __name__ == "__main__":
    # 1. Inisialisasi Konfigurasi
    # Config sekarang memuat INITS dari struktur main.py dan obstacles dari Obstacles.py
    config = Config()

    # 2. Inisialisasi Environment (termasuk obstacles dari config baru)
    environment = Environment(config)

    # 3. Inisialisasi dan Jalankan Simulasi
    simulation = Simulation(config, environment)
    simulation.run()

    # 4. (Opsional) Animasikan hasil
    animator = Animator(config)
    if animator.data: 
        animator.animate(export_gif=False) # Set True untuk menyimpan GIF
