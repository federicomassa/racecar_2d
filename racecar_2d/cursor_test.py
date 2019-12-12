import pygame

screen = pygame.display.set_mode((1200,800))
pygame.init()
done = False
clock = pygame.time.Clock()
pygame.mouse.set_cursor(*pygame.cursors.arrow)
visible = True

while not done:
    for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.done = True
            if event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
                visible = not visible
                pygame.mouse.set_visible(visible)

    pressed = pygame.key.get_pressed()
    
    pygame.display.flip()        
    clock.tick(60)
