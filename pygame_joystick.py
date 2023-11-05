# Example file showing a basic pygame "game loop"
import pygame

# pygame setup
pygame.init()
# screen = pygame.display.set_mode((1280, 720))
clock = pygame.time.Clock()
running = True
pygame.joystick.init()
joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]

print(f'joysticks:{joysticks}')

while running:
    # poll for events
    # pygame.QUIT event means the user clicked X to close your window
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.JOYBUTTONDOWN and joysticks[0].get_button(0):
            print(f'AAAAA')
    # fill the screen with a color to wipe away anything from last frame
    # screen.fill("purple")

    # RENDER YOUR GAME HERE

    # flip() the display to put your work on screen
    # pygame.display.flip()

    clock.tick(60)  # limits FPS to 60

pygame.quit()