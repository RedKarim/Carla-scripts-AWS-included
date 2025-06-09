import glob
import os
import sys
import carla
import pygame
import numpy as np
from pygame.locals import K_w, K_s, K_a, K_d, K_q, K_ESCAPE, K_SPACE

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major, sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# Initialize Pygame
pygame.init()
screen = pygame.display.set_mode((800, 600))
pygame.display.set_caption("CARLA Coordinate Finder")

# Connect to CARLA
client = carla.Client("localhost", 2000)
client.set_timeout(10.0)
world = client.get_world()

# Get the spectator
spectator = world.get_spectator()

# Camera settings
camera_height = 10.0  # Height above ground
camera_speed = 0.5    # Movement speed
rotation_speed = 1.0  # Rotation speed

# Initialize camera position
transform = spectator.get_transform()
transform.location.z = camera_height
spectator.set_transform(transform)

# Font for displaying coordinates
font = pygame.font.Font(None, 36)

# Main loop
clock = pygame.time.Clock()
running = True

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key in [K_ESCAPE, K_q]):
            running = False
        elif event.type == pygame.KEYDOWN and event.key == K_SPACE:
            # Print current coordinates when space is pressed
            transform = spectator.get_transform()
            print(f"\nCurrent Location:")
            print(f"X: {transform.location.x:.3f}")
            print(f"Y: {transform.location.y:.3f}")
            print(f"Z: {transform.location.z:.3f}")
            print(f"Rotation (Pitch, Yaw, Roll):")
            print(f"Pitch: {transform.rotation.pitch:.3f}")
            print(f"Yaw: {transform.rotation.yaw:.3f}")
            print(f"Roll: {transform.rotation.roll:.3f}")

    # Handle continuous key presses
    keys = pygame.key.get_pressed()
    
    # Get current transform
    transform = spectator.get_transform()
    
    # Calculate forward and right vectors
    forward = transform.get_forward_vector()
    right = transform.get_right_vector()
    
    # Update position based on key presses
    if keys[K_w]:
        transform.location += forward * camera_speed
    if keys[K_s]:
        transform.location -= forward * camera_speed
    if keys[K_a]:
        transform.location -= right * camera_speed
    if keys[K_d]:
        transform.location += right * camera_speed
    
    # Update spectator
    spectator.set_transform(transform)
    
    # Clear screen
    screen.fill((0, 0, 0))
    
    # Display current coordinates
    transform = spectator.get_transform()
    coord_text = f"X: {transform.location.x:.3f}, Y: {transform.location.y:.3f}, Z: {transform.location.z:.3f}"
    rot_text = f"Pitch: {transform.rotation.pitch:.3f}, Yaw: {transform.rotation.yaw:.3f}, Roll: {transform.rotation.roll:.3f}"
    
    coord_surface = font.render(coord_text, True, (255, 255, 255))
    rot_surface = font.render(rot_text, True, (255, 255, 255))
    
    screen.blit(coord_surface, (10, 10))
    screen.blit(rot_surface, (10, 50))
    
    # Display instructions
    instructions = [
        "W/S: Move forward/backward",
        "A/D: Move left/right",
        "SPACE: Print coordinates",
        "Q/ESC: Quit"
    ]
    
    for i, instruction in enumerate(instructions):
        text_surface = font.render(instruction, True, (200, 200, 200))
        screen.blit(text_surface, (10, 100 + i * 30))
    
    pygame.display.flip()
    clock.tick(60)

pygame.quit() 