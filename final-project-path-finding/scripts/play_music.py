import pygame
import time

# Initialize the mixer module in pygame
pygame.mixer.init()

# Load the music file
pygame.mixer.music.load("/home/vlois/catkin_ws/src/intro_robo/robotics-final-project/scripts/music.mp3")

def play():
    # Play the music file indefinitely
    pygame.mixer.music.play(-1)

def stop():
    # Stop the music when the program ends
    pygame.mixer.music.stop()

