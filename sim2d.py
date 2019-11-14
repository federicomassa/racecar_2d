import pygame
import os
import json_parser

class Sim2D:
    
    def __init__(self):
        self.done = False
        self.screen = pygame.display.set_mode((1200,800))
        self.clock = pygame.time.Clock()

        self.track_json_path = None
        self.scale = 0.05 # m/pixel
        self.zoom_action = 0.9 # Relative increase in scale on scroll
        self.frequency = 25 # Hz
        self.origin = (0.0, 0.0)
        self.background_color = (255, 255, 255)
        self.track_color = (0,0,0)
        self.track_pix_size = 5
        self.vehicle = None
        self.load_vehicle('Acura_NSX_red.png', 4.4)

        pygame.init()

    def load_vehicle(self, image_path, length):
        self.vehicle = (self.__get_image(image_path), length)

    def world2pix(self, world_point):
        return (int((world_point[0] - self.origin[0])/float(self.scale) + self.screen.get_width()/2.0), int((world_point[1] - self.origin[1])/float(self.scale) + self.screen.get_height()/2.0))

    def pix2world(self, pix_point):
        return ((pix_point[0] - self.screen.get_width()/2.0)*float(self.scale) + self.origin[0], (pix_point[1] - self.screen.get_height()/2.0)*float(self.scale) + self.origin[1])

    def set_track(self,track_json_path):
        self.track_json_path = track_json_path
        self.race_line, self.ins_line, self.out_line = json_parser.json_parser(track_json_path, 1)

    def render_track(self):
        if self.track_json_path == None:
            raise Exception('Please call set_track method before render_track')

        self.origin = self.race_line[0]

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
    def render_vehicle(self, world_pos):
        image = pygame.transform.scale(self.vehicle[0], (int(float(self.vehicle[1])/self.scale), int(float(self.vehicle[1])/self.scale*self.vehicle[0].get_height()/self.vehicle[0].get_width())))

        pix_pos = self.world2pix(world_pos)
        pix_pos = (pix_pos[0] - int(image.get_width()/2.0), pix_pos[1] - int(image.get_height()/2.0))
        self.screen.blit(image, pix_pos)



    def __get_image(self, path):        
        canonicalized_path = path.replace('/', os.sep).replace('\\', os.sep)
        image = pygame.image.load(canonicalized_path)
        return image

    def render(self):
        self.screen.fill(self.background_color)
        self.render_track()
        self.render_vehicle(self.race_line[0])

    def update(self):
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


        self.render()
        pygame.display.flip()        

    def sleep(self):
        self.clock.tick(self.frequency)

