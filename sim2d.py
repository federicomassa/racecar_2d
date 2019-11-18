import pygame
import os
import json_parser
import numpy

class Player:
    def __init__(self, player_index, image_path, length, model_fcn):
        self.image = Sim2D.get_image(image_path)
        self.id = -1
        self.length = length
        self.width = self.image.get_height()/self.image.get_width()*length
        self.model_fcn = model_fcn
        self.current_pose = None

    def init_pose(self, pose):
        assert isinstance(pose, list) or isinstance(pose, tuple)
        assert len(pose) == 3
        self.current_pose = [x for x in pose]

    def update_pose(self, controls, dT, *argv):
        self.current_pose = self.model_fcn(self.current_pose, controls, dT, *argv)
    
class Trajectory:
    def __init__(self):
        self.points = []

def unicycle_model(current_pose, controls, time_step, *argv):
    if len(current_pose) != 3:
        raise Exception("current_pose must be of length 3: x,y,theta")
    if len(controls) != 2:
        raise Exception("controls must be of length 2: v, omega")

    new_pose = current_pose
    new_pose[0] += controls[0]*numpy.cos(current_pose[2])*time_step
    new_pose[1] += controls[0]*numpy.sin(current_pose[2])*time_step
    new_pose[2] += controls[1]*time_step

    return new_pose

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


    def update_player(self, player_index, controls, *argv):
        self.players[player_index].update_pose(controls, 1.0/self.frequency, *argv)
        self.__is_updated[player_index] = True
        
    def add_player(self, image_path, vehicle_length, model_fcn, init_pose):
        self.players.append(Player(len(self.players), image_path, vehicle_length, model_fcn))
        self.players[len(self.players)-1].init_pose(init_pose)
        self.players[len(self.players)-1].id = len(self.players)
        self.__is_updated.append(False)

        # Return player index
        return len(self.players) - 1

    def __focus_player(self, player):
        try:
            self.focus_player = next(p for p in self.players if p == player)
            self.origin = (player.current_pose[0], player.current_pose[1])
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
            image = pygame.transform.rotate(image, player.current_pose[2]*180.0/numpy.pi)

            # XY coordinates of the top left edge of the vehicle image
            pix_pos = self.world2pix(player.current_pose)
            pix_pos = (pix_pos[0] - int(image.get_width()/2.0), pix_pos[1] - int(image.get_height()/2.0))
            self.screen.blit(image, pix_pos)

    def render_ui(self):
        player_text = 'Player ' + str(self.focus_player.id)
        time_text ='Time: ' + "{:4.2f}".format(self.current_time)

        player_text_surface = self.__font.render(player_text, True, self.font_color, self.background_color)
        time_text_surface = self.__font.render(time_text, True, self.font_color, self.background_color)

        player_text_rect = player_text_surface.get_rect()
        textRect.center = (int(textRect.width/2.0 + 0.05*self.screen.get_width()),int(textRect.height/2.0 + 0.05*self.screen.get_height()))
        self.screen.blit(text_surface, textRect)

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
            self.origin = self.focus_player.current_pose

        self.render()
        
        pygame.display.flip()  
        self.current_time += self.__sleep()/1000.0

        # Reset
        for i in self.__is_updated:
            i = False

    def __sleep(self):
        return self.clock.tick(self.frequency)

