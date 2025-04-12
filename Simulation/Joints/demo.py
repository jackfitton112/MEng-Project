#pygame setup for demoing tripteron.

#game where there will be a button to select a random picture (X), the screen will then display the picture and the path
#"pics/X"
#"points/X_points.csv" - need to strip the og file ending - this is the list of points to send to the device
#"graphs/X_path.png" - need to strip the og file ending

#show both the og picture and the graph on the screen with a button to GO or select new random picture
import pygame
import os
import random
import csv
from pygame.locals import *

# Initialize Pygame
pygame.init()

# Set up the display
WIDTH, HEIGHT = 1280,720
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption('Tripteron Demo')

# Load the font for buttons
font = pygame.font.SysFont('Arial', 24)

# Directory paths
pics_dir = "pics/"
graphs_dir = "graphs/"
points_dir = "points/"

# Function to load images and scale them to fit a given width and height
def load_image(path, width, height):
    image = pygame.image.load(path).convert_alpha()
    return pygame.transform.scale(image, (width, height))

# Function to display a button
def draw_button(text, x, y, width, height, color=(100, 100, 200)):
    pygame.draw.rect(screen, color, (x, y, width, height))
    label = font.render(text, True, (255, 255, 255))
    screen.blit(label, (x + 10, y + 10))

# Function to select a random picture from the pics directory
def select_random_picture():
    files = [f for f in os.listdir(pics_dir)]
    return random.choice(files)

# Function to load the path and points data for a selected image
def load_path_and_points(image_name):
    # Assuming the image is named "X.png", strip the extension
    base_name = os.path.splitext(image_name)[0]
    
    # Load the path image
    path_image = load_image(os.path.join(graphs_dir, base_name + '_path.png'), WIDTH // 2, HEIGHT)
    
    # Load the points data (list of points from the CSV)
    points_path = os.path.join(points_dir, base_name + '_points.csv')
    points = load_points_from_csv(points_path)

    return path_image, points

# Function to load points from a CSV file
def load_points_from_csv(file_path):
    points = []
    try:
        with open(file_path, newline='') as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                # Assuming the CSV contains x, y, z values in each row
                points.append([float(val) for val in row])
    except FileNotFoundError:
        print(f"Error: {file_path} not found.")
    return points

# Game loop flag
running = True
selected_image = None
path_image = None
points_data = None

while running:
    # Event handling
    for event in pygame.event.get():
        if event.type == QUIT:
            running = False
        if event.type == MOUSEBUTTONDOWN:
            if go_button.collidepoint(event.pos):
                print("GO button clicked!")
                # Here you can send the points_data to the Tripteron device
                if points_data:
                    print("Sending points data to Tripteron device...")
                    # Placeholder for sending data to the device
                    # send_to_tripteron(points_data)
            elif random_button.collidepoint(event.pos):
                selected_image = select_random_picture()
                path_image, points_data = load_path_and_points(selected_image)
    
    # Fill the screen with a background color
    screen.fill((255, 255, 255))

    # Display the selected image and the path/points graph
    if selected_image:
        # Load and display the selected image on the left
        selected_img = load_image(os.path.join(pics_dir, selected_image), WIDTH // 2, HEIGHT)
        screen.blit(selected_img, (0, 0))
        
        # Display the path image on the right
        if path_image:
            screen.blit(path_image, (WIDTH // 2, 0))
            
        
    # Display the buttons
    button_width, button_height = 200, 50
    go_button = pygame.Rect(WIDTH // 2 - button_width // 2, HEIGHT - 100, button_width, button_height)
    random_button = pygame.Rect(WIDTH // 2 - button_width // 2, HEIGHT - 200, button_width, button_height)

    draw_button("GO", go_button.x, go_button.y, button_width, button_height)
    draw_button("New Random Picture", random_button.x, random_button.y, button_width, button_height)

    # Update the screen
    pygame.display.flip()

# Quit Pygame
pygame.quit()
