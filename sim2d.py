import pygame
import os
import json_parser
import numpy as np
import csv

class TrajectoryPoint:
    def __init__(self, x=None, y=None, theta=None, v=None):
        self.x = x
        self.y = y
        self.theta = theta
        self.v = v

class Player:
    def __init__(self, player_index, image_path, length, model_fcn):
        self.image = Sim2D.get_image(image_path)
        self.id = -1
        self.length = length
        self.width = self.image.get_height()/self.image.get_width()*length
        self.model_fcn = model_fcn
        self.current_state = None
        self.ref_trajectory = None
        self.index_ref_trajectory = 0

    def init_state(self, state):
        assert isinstance(state, list) or isinstance(state, tuple)
        assert len(state) == 4
        self.current_state = [x for x in state]

    def update_state(self, controls, dT, *argv):
        self.current_state = self.model_fcn(self.current_state, controls, dT, *argv)

    def set_trajectory(self, csv_file):
        trajectory = []

        with open(csv_file, newline='') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                trajectory.append(TrajectoryPoint(float(row['x']), float(row['y']), float(row['theta']), float(row['v'])))

        assert len(trajectory) != 0
        
        # Not temporized
        self.ref_trajectory = trajectory

def unicycle_model(current_state, controls, time_step, *argv):
    if len(current_state) != 4:
        raise Exception("current_state must be of length 4: x,y,theta,v")
    if len(controls) != 2:
        raise Exception("controls must be of length 2: v, omega")

    new_state = current_state
    new_state[0] += current_state[3]*np.cos(current_state[2])*time_step
    new_state[1] += current_state[3]*np.sin(current_state[2])*time_step
    new_state[2] += controls[1]*time_step
    new_state[3] += controls[0]*time_step

    return new_state

class Sim2D:
    def __init__(self):
        self.done = False
        self.screen = pygame.display.set_mode((1200,800))
        pygame.display.set_caption('TazioSim 2D')
        self.clock = pygame.time.Clock()
        self.current_time = 0.0
        self.track_json_path = None
        self.scale = 0.05 # m/pixel
        self.zoom_action = 0.9 # Relative increase in scale on scroll
        self.frequency = 25 # Hz
        self.focus_player = None
        self.origin = (0.0,0.0)
        self.background_color = (255, 255, 255)
        self.track_color = (0,0,0)
        self.font_color = (0,0,0)
        self.track_pix_size = 5

        # Dictionary of vehicles in the form {player_index, (vehicle_img, vehicle_length)}
        self.players = []
        self.__is_updated = []
        pygame.init()

        self.__font = pygame.font.Font('freesansbold.ttf', 32)

    def set_trajectory(self, player_index, csv_file):
        self.players[player_index].set_trajectory(csv_file)

    def update_player(self, player_index, controls, *argv):
        if self.players[player_index].ref_trajectory != None:
            raise Exception("Calling update player on a trajectory following vehicle. Please call update_trajectory")

        self.players[player_index].update_state(controls, 1.0/self.frequency, *argv)

        self.__is_updated[player_index] = True

    def update_trajectory(self, player_index):
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
            print(i,'/',len(traj)-1)
            if i == player.index_ref_trajectory:
                delta_s = np.sqrt(np.square(traj[i+1].y-player.current_state[1]) + np.square(traj[i+1].x-player.current_state[0]))
                a = (np.square(traj[i+1].v) - np.square(player.current_state[3]))/(2*delta_s)
                if np.abs(a) < 0.0001:
                    delta_t = delta_s/traj[i].v
                else:
                    delta_t = (traj[i+1].v-player.current_state[3])/a
                print('debug1:',delta_t,t_step, a, traj[i+1].v-player.current_state[3])
            else:
                delta_s = np.sqrt(np.square(traj[i+1].y-traj[i].y) + np.square(traj[i+1].x-traj[i].x))
                a = (np.square(traj[i+1].v) - np.square(traj[i].v))/(2*delta_s)
                if np.abs(a) < 0.0001:
                    delta_t = delta_s/traj[i].v
                else:
                    delta_t = (traj[i+1].v-traj[i].v)/a
                
                print('debug2:',delta_t,t_step, a, traj[i+1].v-traj[i].v)


            N_steps = int(np.ceil(delta_t/t_step))
            if N_steps == 0:
                raise Exception('t_step too large')

            # Next one will be last iteration
            elif N_steps <= total_points*2 and i == len(traj)-2:
                penultimate_iteration = True

            jmin = len(xnew)
            jmax = len(xnew) + N_steps

            print('ds',delta_s, 'a',a, 'dt',delta_t, 'N_steps',N_steps, 'tot_points', total_points, 'jmin/jmax', jmin,'/',jmax)

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
                    print(player.current_state[1], traj[i+1].y, player.current_state[0], traj[i+1].x, np.arctan2(player.current_state[1]-traj[i+1].y, player.current_state[0]-traj[i+1].x))
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

        print('index:',i)
        player.current_state = [xnew[total_points-1], ynew[total_points-1], thetanew[total_points-1], vnew[total_points-1]]
        self.__is_updated[player_index] = True
        
    def add_player(self, image_path, vehicle_length, model_fcn, init_state):
        self.players.append(Player(len(self.players), image_path, vehicle_length, model_fcn))
        self.players[len(self.players)-1].init_state(init_state)
        self.players[len(self.players)-1].id = len(self.players)
        self.__is_updated.append(False)

        # Return player index
        return len(self.players) - 1

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

    def set_track(self,track_json_path):
        self.track_json_path = track_json_path
        self.race_line, self.ins_line, self.out_line = json_parser.json_parser(track_json_path, 1)
        self.origin = (self.race_line[0][0], self.race_line[0][1])

    def set_model(self, player_index, model_fcn):
        self.players[player_index].model_fcn = model_fcn

    def render_track(self):
        if self.track_json_path == None:
            raise Exception('Please call set_track method before render_track')


        for i in range(len(self.ins_line)):

            if i != len(self.ins_line)-1:
                start_point_px = self.world2pix(self.ins_line[i])
                end_point_px = self.world2pix(self.ins_line[i+1])
            else:
                start_point_px = self.world2pix(self.ins_line[i])
                end_point_px = self.world2pix(self.ins_line[0])

            pygame.draw.line(self.screen, self.track_color, start_point_px, end_point_px, self.track_pix_size)

        for i in range(len(self.out_line)):

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
        time_text ='Time: ' + "{:4.2f}".format(self.current_time)

        player_text_surface = self.__font.render(player_text, True, self.font_color, self.background_color)
        player_text_rect = player_text_surface.get_rect()
        player_text_rect.center = (int(player_text_rect.width/2.0 + 0.05*self.screen.get_width()),int(player_text_rect.height/2.0 + 0.05*self.screen.get_height()))
        self.screen.blit(player_text_surface, player_text_rect)

        time_text_surface = self.__font.render(time_text, True, self.font_color, self.background_color)
        time_text_rect = time_text_surface.get_rect()
        time_text_rect.center = (int(self.screen.get_width() - time_text_rect.width/2.0 - 0.05*self.screen.get_width()),int(time_text_rect.height/2.0 + 0.05*self.screen.get_height()))
        self.screen.blit(time_text_surface, time_text_rect)

    @staticmethod
    def get_image(path):        
        canonicalized_path = path.replace('/', os.sep).replace('\\', os.sep)
        image = pygame.image.load(canonicalized_path)
        return image

    def render(self):
        self.screen.fill(self.background_color)
        self.render_track()
        self.render_vehicles()
        self.render_ui()

    def tick(self):
        # Check if all vehicles were updated
        all_updated = not any([not u for u in self.__is_updated])
        assert all_updated

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.done = True
            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 4:
                    # Scroll up
                    self.scale *= self.zoom_action
                if event.button == 5:
                    # Scroll down
                    self.scale /= self.zoom_action
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_1 and len(self.players) > 0:
                    self.__focus_player(self.players[0])
                if event.key == pygame.K_2 and len(self.players) > 1:
                    self.__focus_player(self.players[1])
                if event.key == pygame.K_3 and len(self.players) > 2:
                    self.__focus_player(self.players[2])
                if event.key == pygame.K_4 and len(self.players) > 3:
                    self.__focus_player(self.players[3])


        # By default, camera follows first player
        if self.focus_player == None and len(self.players) != 0:
            self.__focus_player(self.players[0])
        elif self.focus_player != None:
            self.origin = self.focus_player.current_state

        self.render()
        
        pygame.display.flip()  
        self.current_time += self.__sleep()/1000.0

        # Reset
        for player in self.players:
            player.__is_updated = False

    def __sleep(self):
        return self.clock.tick(self.frequency)

