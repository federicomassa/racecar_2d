import pygame

screen = pygame.display.set_mode((800,600))
pygame.init()
done = False

while not done:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True
        if event.type == pygame.MOUSEBUTTONDOWN:
            print(event.button)