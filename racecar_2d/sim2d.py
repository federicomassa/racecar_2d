import pygame
import os
import racecar_2d.json_parser as json_parser
import numpy as np
import csv
import time
from collections import deque
from scipy.spatial import Delaunay
import math
import os.path

try:
  import rospy
  import tf.transformations
except ImportError:
  ros_available = False
else:
  ros_available = True 

try:
  import racecar_msgs.msg
except ImportError:
  msgs_available = False
else:
  msgs_available = True 


class TrajectoryPoint:
    def __init__(self, x=None, y=None, v=None):
        self.x = x
        self.y = y
        self.v = v

class Line:
    def __init__(self, x1, y1, x2, y2):
        self.x1 = x1
        self.x2 = x2
        self.y1 = y1
        self.y2 = y2    

class Sensor:
    def simulate(self):
        pass

class ROSManager:
  def __init__(self, simulator):
    self.players_pub = {}
    self.simulator = simulator
    
  def add_player(self, index):
    # If player was not registered yet
    if not index in self.players_pub.keys():
      self.players_pub[index] = []
      
    topic_namespace = 'player_' + str(index) + '/'
    self.players_pub[index].append(rospy.Publisher(topic_namespace + 'state', racecar_msgs.msg.State, queue_size=10))
    
  def publish(self):
    for index in self.players_pub:
      for pub in self.players_pub[index]:
        if pub.type == 'racecar_msgs/State':
          msg = racecar_msgs.msg.State()
          (x,y,theta,u,v) = self.simulator.players[index].current_state[0:5]
          msg.header.stamp = rospy.Time.now()
          msg.x = x
          msg.y = y
          msg.theta = theta
          msg.u = u
          msg.v = v
          (s,d) = self.simulator.get_track_coordinates((x,y))
          msg.s = s
          msg.d = d
          pub.publish(msg)

class SensorLaser(Sensor):
    def __init__(self, parent_player, simulator, angles, laser_range, laser_resolution):
        """
        Parameters
        simulator: Sim2D instance
        angles: list or tuple of floats
            rays angles
        laser_range: float
            maximum laser reading
        """
    
        assert simulator != None
        assert parent_player != None
        assert isinstance(angles, tuple) or isinstance(angles, list)
        assert len(angles) != 0
        assert isinstance(angles[0], float)
        assert isinstance(laser_range, float)

        self.player = parent_player
        self.simulator = simulator
        self.angles = angles
        self.range = laser_range
        self.resolution = laser_resolution

    def simulate(self, **kwargs):
        """
        Kwargs:
        --------------
        int: index_hint
            Approximate index of current delaunay triangle. See Sim2D.is_inside_track
        int: interval
            Same as arg in Sim2D.is_inside_track

        Returns
        -----------
        list: 
            a list of sensor readings, one for each angle in angles
        """

        assert self.simulator.do_triangle_sorting

        index_hint = kwargs.get('index_hint', -1)
        interval = kwargs.get('interval', -1)

        if index_hint == None:
            index_hint = -1
        if interval == None:
            interval = -1

        readings = []

        current_point = self.player.current_state[0:2]
        inside, init_index = self.simulator.is_inside_track(current_point, index_hint, interval)
        if not inside:
            for i in self.angles:
                readings.append((i, 0.0))
            return readings

        for laser_angle in self.angles:
            angle = self.player.current_state[2] + laser_angle
            point1 = list(current_point)

            # Linear search. Robust but slower than bisect (O(n) instead of O(log(n)))
            linear_count = int(np.ceil(self.range/self.resolution))
            last_index = init_index
            for i in range(linear_count):
                point1[0] = point1[0] + np.cos(angle)*self.resolution
                point1[1] = point1[1] + np.sin(angle)*self.resolution
                is_inside, last_index = self.simulator.is_inside_track(point1, last_index, 10)
                if not is_inside:
                    break

            distance = np.sqrt(np.square(point1[0] - current_point[0]) + np.square(point1[1] - current_point[1]))
            readings.append((laser_angle, distance))

        return readings


class Player:
    def __init__(self, player_index, image_path, length, model_fcn):
        self.image = Sim2D.get_image(image_path)
        self.id = player_index
        self.length = length
        self.width = self.image.get_height()/self.image.get_width()*length
        self.model_fcn = model_fcn
        self.current_state = None
        self.ref_trajectory = None
        self.index_ref_trajectory = 0
        self.sensors = {}

    def init_state(self, state):
        assert isinstance(state, list) or isinstance(state, tuple)
        assert len(state) == 5
        self.current_state = [x for x in state]

    def update_state(self, controls, dT, *argv):
        self.current_state = self.model_fcn(self.current_state, controls, dT, *argv)

    def set_csv_trajectory(self, csv_file):
        trajectory = []

        with open(csv_file) as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                trajectory.append(TrajectoryPoint(float(row['x']), float(row['y']), float(row['v'])))

        assert len(trajectory) != 0
        
        # Not temporized
        self.ref_trajectory = trajectory
        self.index_ref_trajectory = 0

    def set_trajectory(self, trajectory):
        assert isinstance(trajectory, list) or isinstance(trajectory, tuple)
        assert len(trajectory) != 0
        assert isinstance(trajectory[0], TrajectoryPoint)

        self.ref_trajectory = trajectory
        self.index_ref_trajectory = 0

    def add_sensor(self, name, sensor):
        assert sensor != None
        self.sensors[name] = sensor

    def simulate(self, name, **kwargs):
        assert name in self.sensors, "Sensor {} was not registered for player {}. Please call add_sensor first.".format(name, self.id)
        return self.sensors[name].simulate(**kwargs)    

def unicycle_model(current_state, controls, time_step, *argv):
    if len(current_state) != 5:
        raise Exception("current_state must be of length 5: x,y,theta,u,v")
    if len(controls) != 2:
        raise Exception("controls must be of length 2: v, omega")

    new_state = current_state
    new_state[0] += current_state[3]*np.cos(current_state[2])*time_step
    new_state[1] += current_state[3]*np.sin(current_state[2])*time_step
    new_state[2] += controls[1]*time_step
    new_state[3] += controls[0]*time_step
    new_state[4] = 0.0

    return new_state

def forward_unicycle_model(current_state, controls, time_step, *argv):
    if len(current_state) != 5:
        raise Exception("current_state must be of length 5: x,y,theta,u,v")
    if len(controls) != 2:
        raise Exception("controls must be of length 2: v, omega")

    new_state = current_state
    new_state[0] += current_state[3]*np.cos(current_state[2])*time_step
    new_state[1] += current_state[3]*np.sin(current_state[2])*time_step
    new_state[2] += controls[1]*time_step
    new_state[3] += controls[0]*time_step
    new_state[4] = 0.0

    # Constrain to forward-only motion
    if new_state[3] < 0.0:
        new_state[3] = 0.0

    return new_state

class Sim2D:
    def __init__(self, render=True, sort_triangles=True, real_time=True, use_ros=False):
        self.done = False
        # If render to screen or play in background
        self.__do_render = render

        # If the simulator should sleep before iterations to maintain desired frequency or not
        if render and not real_time:
            raise Exception("Rendering only supports real time mode.")

        if use_ros and not ros_available:
          raise Exception("Cannot import ros modules! Check rospy installation")
        if use_ros and not msgs_available:
          raise Exception("Cannot import racecar_msgs modules! Have you installed them in current python environment?")

        self.real_time = real_time
        self.use_ros = use_ros
        
        if use_ros:
          self.ros_manager = ROSManager(self)
          rospy.init_node('racecar_2d', anonymous=True)

        # Graphical properties
        self.screen = None
        self.clock = None
        self.track_pix_size = None
        self.__font = None
        self.scale = None
        self.zoom_action = None
        self.focus_player = None
        self.is_manual = False
        self.origin = None
        self.manual_acc_control = None
        self.manual_brake_control = None
        self.manual_steer_control = None
        self.manual_model_fcn = None
        self.clicked_point = None
        self.pressed_point = None
        self.__queue_paths = None
        self.__queue_points = []
        self.__queue_lines = []
        self.__queue_persistent_points = []
        self.__queue_players = deque()
        self.do_triangle_sorting = sort_triangles
        self.render_one_every = 1
        self.dump_file = None

        self.current_time = 0.0
        self.track_json_path = None
       
        self.frequency = 25 # Hz

        # Camera free to move with mouse
        self.__fly_mode = False

        # Dictionary of vehicles in the form {player_index, (vehicle_img, vehicle_length)}
        self.players = []
        self.__is_updated = []

        if self.__do_render:
            self.display_on()

    def display_on(self):
        if self.screen != None:
            return
        
        self.__do_render = True
        self.real_time = True
        self.screen = pygame.display.set_mode((1200,800))
        pygame.display.set_caption('TazioSim 2D')
        self.clock = pygame.time.Clock()
        self.track_pix_size = 5
        self.background_color = (255, 255, 255)
        self.track_color = (0,0,0)
        self.font_color = (0,0,0)
        pygame.init()
        self.__font = pygame.font.Font('freesansbold.ttf', 32)
        
        self.scale = 0.05 # m/pixel
        self.zoom_action = 0.9 # Relative increase in scale on scroll
        self.is_manual = False
        self.origin = (0.0,0.0)
        self.manual_acc_control = 0.8
        self.manual_brake_control = 1.0
        self.manual_steer_control = 1.0
        self.manual_model_fcn = unicycle_model

    def display_off(self, real_time=True):
        if self.screen == None:
            return
        
        self.__do_render = False
        self.real_time = real_time
        pygame.quit()
        self.screen = None
        
        self.clock = None
        self.track_pix_size = None
        self.background_color = None
        self.track_color = None
        self.font_color = None
        self.__font = None
        
        self.scale = None
        self.zoom_action = None
        self.is_manual = False
        self.origin = None
        self.manual_acc_control = None
        self.manual_brake_control = None
        self.manual_steer_control = None
        self.manual_model_fcn = None
        self.__queue_points = []
        self.__queue_lines = []
        self.__queue_persistent_points = []

    def set_csv_trajectory(self, player_index, csv_file):
        self.players[player_index].set_csv_trajectory(csv_file)

    def set_trajectory(self, player_index, trajectory):
        self.players[player_index].set_trajectory(trajectory)

    def update_player(self, player_index, controls, *argv):
        # If manual mode ignore controls from outside
        if self.focus_player != None and self.focus_player.id == player_index and self.is_manual:
            return
        else:
            self.__update_player(player_index, controls, *argv)

    def __update_player(self, player_index, controls, *argv):
        if self.players[player_index].ref_trajectory != None:
            raise Exception("Calling update player on a trajectory following vehicle. Please call update_trajectory")

        self.__queue_players.append((player_index, controls))
        self.__is_updated[player_index] = True

    def __update_player_manual(self, player_index, controls, *argv):
        self.players[player_index].current_state = self.manual_model_fcn(self.players[player_index].current_state, controls, 1.0/self.frequency, *argv)
        self.__is_updated[player_index] = True

    def update_trajectory(self, player_index):
        if self.players[player_index] == self.focus_player and self.is_manual:
            return

        player = self.players[player_index]
        traj = player.ref_trajectory

        if traj == None:
            raise Exception("Calling update_trajectory on a controlled vehicle. Please call update_player.")

        # Define integration time step
        if 1.0/self.frequency > 0.001:
            t_step = 0.001
        else:
            t_step = 1.0/self.frequency/2.0

        xnew = []
        ynew = []
        thetanew = []
        vnew = []
        
        penultimate_iteration = False

        # Total points to predict, choose best approximation
        total_points = int(np.ceil(1.0/self.frequency/t_step))
        if np.abs(t_step*(total_points+1)-1.0/self.frequency) < np.abs(t_step*total_points-1.0/self.frequency):
            total_points += 1

        for i in range(player.index_ref_trajectory, len(traj)-1):
            if i == player.index_ref_trajectory:
                delta_s = np.sqrt(np.square(traj[i+1].y-player.current_state[1]) + np.square(traj[i+1].x-player.current_state[0]))
                a = (np.square(traj[i+1].v) - np.square(player.current_state[3]))/(2*delta_s)
                if np.abs(a) < 0.0001:
                    if traj[i].v < 0.0001:
                        raise Exception("Trajectory has too low acceleration and velocity")
                    delta_t = delta_s/traj[i].v
                else:
                    delta_t = (traj[i+1].v-player.current_state[3])/a
            else:
                delta_s = np.sqrt(np.square(traj[i+1].y-traj[i].y) + np.square(traj[i+1].x-traj[i].x))
                a = (np.square(traj[i+1].v) - np.square(traj[i].v))/(2*delta_s)
                if np.abs(a) < 0.0001:
                    if traj[i].v < 0.0001:
                        raise Exception("Trajectory has too low acceleration and velocity")
                    delta_t = delta_s/traj[i].v
                else:
                    delta_t = (traj[i+1].v-traj[i].v)/a
                

            N_steps = np.ceil(delta_t/t_step)
            if N_steps == 0:
                raise Exception('t_step too large')
            if math.isnan(N_steps):
                raise Exception('Something went wrong. N_steps is NaN')

            N_steps = int(N_steps)

            # Next one will be last iteration
            if N_steps <= total_points*2 and i == len(traj)-2:
                penultimate_iteration = True

            jmin = len(xnew)
            jmax = len(xnew) + N_steps

            requested_break = False

            for j in range(jmin,jmax):
                if i != player.index_ref_trajectory and j == jmin:
                    xnew.append(traj[i].x)
                    ynew.append(traj[i].y)
                    thetanew.append(np.arctan2(traj[i+1].y-player.current_state[1], traj[i+1].x - player.current_state[0]))
                    vnew.append(traj[i].v) 
                elif i == player.index_ref_trajectory and j == jmin:
                    xnew.append(player.current_state[0])
                    ynew.append(player.current_state[1])
                    thetanew.append(np.arctan2(traj[i+1].y-player.current_state[1], traj[i+1].x - player.current_state[0]))
                    vnew.append(player.current_state[3]) 
                else:
                    xnew.append(xnew[j-1] + vnew[j-1]*np.cos(thetanew[j-1])*t_step)
                    ynew.append(ynew[j-1] + vnew[j-1]*np.sin(thetanew[j-1])*t_step)
                    thetanew.append(thetanew[j-1])
                    vnew.append(vnew[j-1] + a*t_step)

                if len(xnew) == total_points:
                    requested_break = True
                    break

            if requested_break:
                break

        if penultimate_iteration:
            player.index_ref_trajectory = 0
        else:
            player.index_ref_trajectory = i

        player.current_state = [xnew[total_points-1], ynew[total_points-1], thetanew[total_points-1], vnew[total_points-1], 0.0]
        self.__is_updated[player_index] = True
        
    def add_player(self, image_path, vehicle_length, model_fcn, init_state):
        self.players.append(Player(len(self.players), image_path, vehicle_length, model_fcn))
        self.players[len(self.players)-1].init_state(init_state)
        self.__is_updated.append(False)
        player_index = len(self.players)-1
        
        if self.use_ros:
          self.ros_manager.add_player(player_index)

        # Return player index
        return player_index

    def __focus_player(self, player):
        try:
            self.focus_player = next(p for p in self.players if p == player)
            self.origin = (player.current_state[0], player.current_state[1])
        except:
            raise Exception('Requested camera focus on non-existent player')


    def world2pix(self, world_point):
        return (int((world_point[0] - self.origin[0])/float(self.scale) + self.screen.get_width()/2.0), int((-world_point[1] + self.origin[1])/float(self.scale) + self.screen.get_height()/2.0))

    def pix2world(self, pix_point):
        return ((pix_point[0] - self.screen.get_width()/2.0)*float(self.scale) + self.origin[0], -(pix_point[1] - self.screen.get_height()/2.0)*float(self.scale) + self.origin[1])

    def set_track(self,track_json_path, dump_file=None, render_one_every=1, take_point_every=1):
        """
        Sets the track map. The track is specified in JSON format, and it should contain an array of "Inside" points, "Outside" points and "Racing" points, which constitutes the racing line, used as a reference for the curvilinear abscissa.
        
        Args: 
        -----------
        track_json_path: string
            the path of the json file
        dump_file: string
            path of the file where to dump the sorted Delaunay triangles, to avoid doing it again every time you run the simulator
        render_one_every: int
            specifies that not every point of the track has to be rendered each frame. This is useful for large maps. NB it only affects rendering 
        take_point_every: int
            take one point every X on the racing line. NB this affects how the values of the local coordinates 
        """

        self.dump_file = dump_file
        self.track_json_path = track_json_path
        self.race_line, self.ins_line, self.out_line = json_parser.json_parser(track_json_path, take_point_every)
        self.origin = (self.race_line[0][0], self.race_line[0][1])

        if self.race_line[len(self.race_line)-1] == self.race_line[0]:
            self.race_line.pop()
        if self.ins_line[len(self.ins_line)-1] == self.ins_line[0]:
            self.ins_line.pop()
        if self.out_line[len(self.out_line)-1] == self.out_line[0]:
            self.out_line.pop()
        

        # Setup Delaunay triangulation to infer if a point is inside or outside the track boundaries
        points = [] 
        for x in self.ins_line:
            points.append([x[0], x[1]])
        for x in self.out_line:
            points.append([x[0], x[1]])

        points = np.array(points)

        t = Delaunay(points)
        self.delaunay_triangles = points[self.__clean_triangles(list(t.simplices))]
        self.__curvilinear_abscissa = self.__compute_curvilinear_abscissa(self.race_line)


        dump_exists = dump_file != None and os.path.exists(dump_file)

        if self.do_triangle_sorting:
            import pickle
            if dump_exists:
                print("Loading Delaunay triangles from file {}".format(dump_file))
                self.delaunay_triangles = pickle.load(open(dump_file, 'r'))
            else:
                print("Starting Dealunay triangles sorting, this could take a while on large maps...")

                # Compute centers of each triangle in delaunay triangles
                delaunay_centers = [((t[0][0] + t[1][0] + t[2][0])/3.0, \
                    (t[0][1] + t[1][1] + t[2][1])/3.0) for t in self.delaunay_triangles]

                # Compute curvilinear abscissa of each triangle center
                from tqdm import tqdm

                delaunay_s = []
                for i in tqdm(range(len(delaunay_centers))):
                    p = delaunay_centers[i]
                    delaunay_s.append(self.get_track_coordinates(p)[0])

                # Get sorting order
                sorting_indices = np.argsort(delaunay_s)

                # Sort delaunay triangles 
                self.delaunay_triangles = [self.delaunay_triangles[i] for i in sorting_indices]

                # Dump to file for future loading
                if dump_file != None:
                    pickle.dump(self.delaunay_triangles, open(dump_file, 'wb'))

        self.render_one_every = render_one_every

    def __compute_curvilinear_abscissa(self, ref_line):
        """
        Integrates length of a reference line
        
        Parameters
        -------------
        ref_line: list or tuple of points (list or tuple of 2D coordinates)

        Returns
        list:
            list of same length of ref_line that contains the incremental length of the curve
        """

        s = []
        s.append(0.0)
        for i in range(1, len(ref_line)):
            s.append(s[i-1] + np.sqrt(np.square(ref_line[i][0] - ref_line[i-1][0]) + np.square(ref_line[i][1] - ref_line[i-1][1])))

        return s

    def get_track_coordinates(self, point):
        """
        Transform point from global coordinates (x,y) to local (s,d) using the curvilinear abscissa computed on racing line

        Args:
        -------------
        point: 
            point in global coordinates (tuple of 2 floats)

        Returns
        ---------------
        point: 
            point in local coordinates (tuple of 2 floats)
        """
        local_point = [None, None]

        dist2 = [np.square(point[0]-self.race_line[i][0]) + np.square(point[1]-self.race_line[i][1]) for i in range(len(self.race_line))]
        min_dist_index = np.argmin(dist2)

        next_index = min_dist_index + 1
        prev_index = min_dist_index - 1
        # Out of bounds case
        if next_index == len(self.race_line):
            next_index = 0
        if prev_index == -1:
            prev_index = len(self.race_line)-1

        prev_to_point = [point[0] - self.race_line[prev_index][0], point[1] - self.race_line[prev_index][1]]
        prev_to_current = [self.race_line[min_dist_index][0] - self.race_line[prev_index][0], self.race_line[min_dist_index][1] - self.race_line[prev_index][1]]

        product = np.dot(prev_to_point, prev_to_current)/np.sqrt(np.dot(prev_to_current, prev_to_current))
        if product < 0:
            raise Exception("Product < 0 WHY?")
        if self.get_curvilinear_distance(prev_index, min_dist_index) < product:
            current_to_point = [point[0] - self.race_line[min_dist_index][0], point[1] - self.race_line[min_dist_index][1]]
            current_to_next = [self.race_line[next_index][0] - self.race_line[min_dist_index][0], self.race_line[next_index][1] - self.race_line[min_dist_index][1]]
            
            product = np.dot(current_to_point, current_to_next)/np.sqrt(np.dot(current_to_next, current_to_next))
            if self.get_curvilinear_distance(min_dist_index, next_index) < product:
                raise Exception("WTF is going on")
            if product < 0:
                raise Exception("Product < 0 WHY?")
        
            actual_prev_index = min_dist_index
            actual_next_index = next_index
        else:
            actual_prev_index = prev_index
            actual_next_index = min_dist_index

        local_point[0] = self.__curvilinear_abscissa[actual_prev_index] + product
        
        race_theta = np.arctan2(self.race_line[actual_next_index][1] - self.race_line[actual_prev_index][1], self.race_line[actual_next_index][0] - self.race_line[actual_prev_index][0])
        point_theta = np.arctan2(point[1] - self.race_line[actual_prev_index][1], point[0] - self.race_line[actual_prev_index][0])

        theta = point_theta - race_theta
        actual_prev_to_point = np.sqrt(np.square(point[0] - self.race_line[actual_prev_index][0]) + np.square(point[1] - self.race_line[actual_prev_index][1]))

        local_point[1] = actual_prev_to_point*np.sin(theta)

        return local_point
            
    def get_curvilinear_distance(self, index1, index2):
        dist = self.__curvilinear_abscissa[index2] - self.__curvilinear_abscissa[index1]
        track_length = self.get_track_length()

        if index2 == 0 and index1 == len(self.race_line)-1:
            dist = np.sqrt(np.square(self.race_line[0][0]-self.race_line[len(self.race_line)-1][0]) + np.square(self.race_line[0][1]-self.race_line[len(self.race_line)-1][1]))
        elif index1 == 0 and index2 == len(self.race_line)-1:
            dist = -np.sqrt(np.square(self.race_line[0][0]-self.race_line[len(self.race_line)-1][0]) + np.square(self.race_line[0][1]-self.race_line[len(self.race_line)-1][1]))
        elif dist > track_length/2.0:
            dist -= track_length
        elif dist < -track_length/2.0:
            dist += track_length

        return dist

    def get_forward_ds(self, s1, s2):
        """
        Get distance in curvilinear abscissa from local coordinates s1 to s2 going forward

        Parameters
        ------------
        s1: float
            curvilinear abscissa of point 1
        s2: float
            curvilinear abscissa of point 2

        Returns
        -----------
        float:
            delta s between point 1 and point 2
        """

        total_length = self.get_track_length() 
        new_s2 = float(s2)

        if s2 < s1:
            new_s2 += total_length

        return (new_s2 - s1)

    def get_track_length(self):
        """
        Get total length of track along race line

        Returns
        ---------
        float:
            length of the track along the race line
        """

        total_length = self.__curvilinear_abscissa[-1] + np.sqrt(np.square(self.race_line[-1][0] - self.race_line[0][0]) + np.square(self.race_line[-1][1] - self.race_line[0][1]))
        return total_length




    @staticmethod
    def get_sweeping(vector, index=-1, interval=-1):
        """
        Helper method to retrieve a sweep order given a vector, an index hint around which to check something, possibly an interval around which to look
        E.g. vector of len 10 --> get_sweeping(vector, 2, 4) = [2, 1, 3, 0, 4, 9, 5, 8, 6], a list of indexes that expands around the index

        Parameters
        -------------
        vector: list or tuple
            The vector of which to get the sweep
        index: int, optional
            The optional index hint around which to sweep
        interval: int, optional (if index is not provided this is not used)
            Interval around the index
        """

        if index == None:
            index = -1
        if interval == None:
            interval = -1
        
        index_provided = index >= 0
        interval_provided = interval >= 0

        sweep = []
        if not index_provided and not interval_provided:
            sweep = [i for i in range(len(vector))]
        elif index_provided:
            # How many points does the sweep contain? Interval is on the left and on the right of the index
            if interval_provided:
                npoints = interval*2 + 1
            else:
                npoints = len(vector)

            current_index = index
            current_step = 1
            current_sign = -1
            sweep.append(current_index)
            for i in range(npoints-1):
                current_index += current_step*current_sign
                current_step += 1
                current_sign = -current_sign
                
                actual_index = current_index
                if current_index < 0:
                    actual_index = len(vector) + current_index
                elif current_index >= len(vector):
                    actual_index = current_index - len(vector)

                sweep.append(actual_index)

        return sweep

    def is_inside_track(self, point, index_hint = -1, check_interval = -1):
        """
        Checks if a point lies inside the track

        Parameters
        ----------
        point : list or tuple
            Point to be checked
        index_hint : int
            optional, index of the triangle around which to check for inside/outside. This might be an hint from previous evaluations
        check_interval: int
            optional, how many triangles around index_hint to check

        Returns
        ----------
        bool: 
            True if point is inside the track
        int: 
            Index of the triangle, might be used as an hint for future evaluations. If bool is False, it is None

        """

        is_inside_track = False
        triangle_index = None
        sweep = Sim2D.get_sweeping(self.delaunay_triangles, index_hint, check_interval)
        
        for index in sweep:
            if self.is_inside_triangle(point, self.delaunay_triangles[index]):
                is_inside_track = True
                triangle_index = index
                break
        
        return is_inside_track, triangle_index

    @staticmethod
    def is_inside_triangle(point, triangle):
        """
        Checks if a point (x,y) lies inside a triangle ((x0,y0), (x1,y1), (x2,y2))

        Returns
        ---------
        bool
        """
        t1 = triangle[0]
        t2 = triangle[1]
        t3 = triangle[2]
        c1 = (t2[0]-t1[0])*(point[1]-t1[1])-(t2[1]-t1[1])*(point[0]-t1[0])
        c2 = (t3[0]-t2[0])*(point[1]-t2[1])-(t3[1]-t2[1])*(point[0]-t2[0])
        c3 = (t1[0]-t3[0])*(point[1]-t3[1])-(t1[1]-t3[1])*(point[0]-t3[0])
        if (c1<0 and c2<0 and c3<0) or (c1>0 and c2>0 and c3>0):
            return True
        else:
            return False

    def __clean_triangles(self, triangles):
        """
        Cleans Delaunay triangles assuming the points were ordered appending outside line points to inside line points
        """
        clean_triangles = []
        n_ins_points = len(self.ins_line)

        for t in triangles:
            # Remove triangles formed only of inside or outside points
            if t[0] < n_ins_points and t[1] < n_ins_points and t[2] < n_ins_points:
                continue
            if t[0] >= n_ins_points and t[1] >= n_ins_points and t[2] >= n_ins_points:
                continue
                
            clean_triangles.append([t[0], t[1], t[2]])

        return np.array(clean_triangles)
            
    def set_model(self, player_index, model_fcn):
        self.players[player_index].model_fcn = model_fcn

    def render_track(self):
        if self.track_json_path == None:
            raise Exception('Please call set_track method before render_track')


        for i in range(len(self.ins_line)):
            if i % self.render_one_every != 0: 
                continue

            if i != len(self.ins_line)-1:
                start_point_px = self.world2pix(self.ins_line[i])
                end_point_px = self.world2pix(self.ins_line[i+1])
            else:
                start_point_px = self.world2pix(self.ins_line[i])
                end_point_px = self.world2pix(self.ins_line[0])

            pygame.draw.line(self.screen, self.track_color, start_point_px, end_point_px, self.track_pix_size)

        for i in range(len(self.out_line)):
            if i % self.render_one_every != 0: 
                continue

            if i != len(self.out_line)-1:
                start_point_px = self.world2pix(self.out_line[i])
                end_point_px = self.world2pix(self.out_line[i+1])
            else:
                start_point_px = self.world2pix(self.out_line[i])
                end_point_px = self.world2pix(self.out_line[0])

            pygame.draw.line(self.screen, self.track_color, start_point_px, end_point_px, self.track_pix_size) 

    # Draw vehicle in position world_pos
    def render_vehicles(self):
        for player in self.players:
            image = pygame.transform.scale(player.image, (int(float(player.length)/self.scale), int(float(player.length)/self.scale*player.image.get_height()/player.image.get_width())))
            image = pygame.transform.rotate(image, player.current_state[2]*180.0/np.pi)

            # XY coordinates of the top left edge of the vehicle image
            pix_pos = self.world2pix(player.current_state)
            pix_pos = (pix_pos[0] - int(image.get_width()/2.0), pix_pos[1] - int(image.get_height()/2.0))
            self.screen.blit(image, pix_pos)

    def render_ui(self):
        player_text = 'Player ' + str(self.focus_player.id)
        if self.is_manual:
            player_text += ' - MANUAL'
        elif not self.is_manual and self.focus_player.ref_trajectory == None:
            player_text += ' - AUTONOMOUS'
        elif not self.is_manual and self.focus_player.ref_trajectory != None:
            player_text += ' - TRAJECTORY'
        else:
            raise Exception("Unmanaged situation. Call 911")

        state_text = "v: {:.2f} m/s".format(self.focus_player.current_state[3])

        time_text ='Time: ' + "{:4.2f}".format(self.current_time)

        player_text_surface = self.__font.render(player_text, True, self.font_color, self.background_color)
        player_text_rect = player_text_surface.get_rect()
        player_text_rect.center = (int(player_text_rect.width/2.0 + 0.05*self.screen.get_width()),int(player_text_rect.height/2.0 + 0.05*self.screen.get_height()))
        self.screen.blit(player_text_surface, player_text_rect)

        state_text_surface = self.__font.render(state_text, True, self.font_color, self.background_color)
        state_text_rect = state_text_surface.get_rect()
        state_text_rect.center = (int(state_text_rect.width/2.0 + 0.05*self.screen.get_width()),int(state_text_rect.height/2.0 + 0.05*self.screen.get_height() + player_text_rect.height))
        self.screen.blit(state_text_surface, state_text_rect)

        time_text_surface = self.__font.render(time_text, True, self.font_color, self.background_color)
        time_text_rect = time_text_surface.get_rect()
        time_text_rect.center = (int(self.screen.get_width() - time_text_rect.width/2.0 - 0.05*self.screen.get_width()),int(time_text_rect.height/2.0 + 0.05*self.screen.get_height()))
        self.screen.blit(time_text_surface, time_text_rect)

    def render_paths(self):
        for path_pix,color in self.__queue_paths:
            pygame.draw.circle(self.screen, color, path_pix[0], 2*self.track_pix_size)
            for i in range(len(path_pix)-1):
                pygame.draw.line(self.screen, color, path_pix[i], path_pix[i+1], self.track_pix_size)
                pygame.draw.circle(self.screen, color, path_pix[i+1], 2*self.track_pix_size)

    def draw_point(self, point, color=(255,0,0), size=5, persistent=False):
        if self.__do_render and persistent:
            self.__queue_persistent_points.append((point, color, size))
        elif self.__do_render and not persistent:
            self.__queue_points.append((point, color, size))

    def draw_path(self, path, color=(0,0,255)):
        assert isinstance(path, list) or isinstance(path, tuple)
        path_t = []

        if isinstance(path[0], TrajectoryPoint):
            for p in path:
                path_t.append([p.x, p.y])
        else:
            path_t = path

        path_pix = []
        for t in path_t:
            path_pix.append(self.world2pix(t))

        if self.__queue_paths == None:
            self.__queue_paths = []

        self.__queue_paths.append((path_pix, color))    

    @staticmethod
    def get_image(path):        
        canonicalized_path = path.replace('/', os.sep).replace('\\', os.sep)
        image = pygame.image.load(canonicalized_path)
        return image

    def render(self):
        self.screen.fill(self.background_color)
        self.render_track()

        if len(self.players) != 0:
            self.render_vehicles()

        if not self.__fly_mode:
            self.render_ui()

        if self.__queue_paths != None:
            self.render_paths()
            self.__queue_paths = None

        for point in self.__queue_points:
            pygame.draw.circle(self.screen, point[1], self.world2pix(point[0]), point[2])

        for point in self.__queue_persistent_points:
            pygame.draw.circle(self.screen, point[1], self.world2pix(point[0]), point[2])

        self.__queue_points = []

        for line in self.__queue_lines:
            p1 = self.world2pix([line.x1, line.y1])
            p2 = self.world2pix([line.x2, line.y2])
            pygame.draw.line(self.screen, (255,0,0), p1, p2, 5)

        self.__queue_lines = []

    def tick(self):
        if self.__do_render:
            self.display_on()

        if self.use_ros:
          self.ros_manager.publish()

        for i in range(len(self.__is_updated)):
            if not self.__is_updated:
                if self.players[i] != self.focus_player:
                    print("WARNING: Player {} was not updated. Staying still.".format(i))
                else:
                    # If focus player is not updated, assume manual mode
                    self.is_manual = True

    
        # Update players in queue
        while len(self.__queue_players) != 0:
            p = self.__queue_players.pop()
            self.players[p[0]].update_state(p[1], 1.0/self.frequency)

        if self.__do_render:
            self.__interact()

            if self.__do_render:
                self.render()
                pygame.display.flip()  
        
        self.current_time += self.__sleep()

        # Reset
        for player in self.players:
            player.__is_updated = False
        

    def __interact(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.done = True
                self.display_off()
                return
            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 4:
                    # Scroll up
                    if self.scale > 0.02:
                        self.scale *= self.zoom_action
                if event.button == 5:
                    # Scroll down
                    if self.scale < 1.0:
                        self.scale /= self.zoom_action
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_0 and len(self.players) > 0:
                    self.__request_focus(0)
                elif event.key == pygame.K_1 and len(self.players) > 1:
                    self.__request_focus(1)
                elif event.key == pygame.K_2 and len(self.players) > 2:
                    self.__request_focus(2)
                elif event.key == pygame.K_3 and len(self.players) > 3:
                    self.__request_focus(3)
                elif event.key == pygame.K_4 and len(self.players) > 4:
                    self.__request_focus(4)
                elif event.key == pygame.K_5 and len(self.players) > 5:
                    self.__request_focus(5)
                elif event.key == pygame.K_6 and len(self.players) > 6:
                    self.__request_focus(6)
                elif event.key == pygame.K_7 and len(self.players) > 7:
                    self.__request_focus(7)
                elif event.key == pygame.K_8 and len(self.players) > 8:
                    self.__request_focus(8)
                elif event.key == pygame.K_9 and len(self.players) > 9:
                    self.__request_focus(9)                    
                elif event.key == pygame.K_f:
                    self.is_manual = False
                    self.__fly_mode = True
                    self.focus_player = None
                elif event.key == pygame.K_m and self.focus_player != None:
                    self.is_manual = not self.is_manual
            if event.type == pygame.MOUSEBUTTONDOWN:
                if self.clicked_point == None:
                    self.clicked_point = self.pix2world(event.pos)
            if event.type == pygame.MOUSEBUTTONUP:
                self.clicked_point = None
                self.pressed_point = None

        if self.is_manual and self.focus_player != None:                
            pressed = pygame.key.get_pressed()

            # Controls for manual mode
            acc = 0
            steer = 0

            if pressed[pygame.K_UP]:
                acc += self.manual_acc_control
            if pressed[pygame.K_DOWN]:
                acc -= self.manual_brake_control
            if pressed[pygame.K_LEFT]:
                steer += self.manual_steer_control
            if pressed[pygame.K_RIGHT]:
                steer -= self.manual_steer_control

            self.__update_player_manual(self.focus_player.id, (acc, steer))

        # By default, camera follows first player if present
        if self.focus_player == None and len(self.players) != 0 and not self.__fly_mode:
            self.__focus_player(self.players[0])
        elif self.focus_player != None:
            s = self.focus_player.current_state
            self.origin = [s[0], s[1], s[2]]
        elif len(self.players) == 0:
            self.origin = self.race_line[0]
        elif self.__fly_mode and self.clicked_point != None and self.pressed_point != None:
            # Update origin with pressed point from previous iteration
            self.origin = [self.origin[i] - self.pix2world(pygame.mouse.get_pos())[i] + self.pressed_point[i] for i in range(2)]

        clicked = pygame.mouse.get_pressed()
        if clicked[0]:
            self.pressed_point = self.pix2world(pygame.mouse.get_pos())
    
    def __request_focus(self, player_index):
        self.__focus_player(self.players[player_index])
        self.is_manual = False
        self.__fly_mode = False

    def reset(self, player_index, race_line_index, d=0.0, dtheta=0.0, v=0.0):
        """
        Parameters
        ----------
        player_index : int
            Index of the player to be reset
        race_line_index : int
            Index on the race line where to reset the player
        d : float
            Transversal distance with respect to race line
        v : float
            Starting speed
        dtheta: float
            Deviation from tangent to the race line (positive counter-clockwise)
        """

        if len(self.players) == 0:
            return

        if race_line_index != (len(self.race_line) - 1):
            next_index = race_line_index + 1
        else:
            next_index = 0
        
        x = self.race_line[race_line_index][0]
        y = self.race_line[race_line_index][1]

        next_x = self.race_line[next_index][0]
        next_y = self.race_line[next_index][1]

        theta = np.arctan2(next_y - y, next_x - x) + dtheta
        self.players[player_index].current_state = [x - d*np.sin(theta), y + d*np.cos(theta), theta, v]
        self.current_time = 0.0

        # FIXME this is a reset for a particular player. Does not make much sense to reset simulator variables here
        self.__queue_persistent_points = []

    def __sleep(self):
        if self.__do_render:
            return self.clock.tick(self.frequency)/1000.0
        else:
            if self.real_time:
                time.sleep(1.0/self.frequency)
            return 1.0/self.frequency