import pygame
import cv2
import numpy as np
import random
from class_robot import Robot
from function_track_hands import track_hands
from function_find_background_image import find_background_image
import mediapipe as mp

def assign_destinations_to_robots(robots_list, destinations):
        for i, robot in enumerate(robots_list):
            dest_x, dest_y = destinations[i % len(destinations)]
            robot.set_destination(dest_x, dest_y, time_in_seconds=5, fps=30)
            print(f"Robot {robot.robot_id} → assigned destination: ({dest_x}, {dest_y})")



def main():
    print("Sim Begin")

    pygame.init()
    arena_width, arena_height = 900, 900
    screen = pygame.display.set_mode((arena_width, arena_height))
    pygame.display.set_caption("Robot Arena")

    mp_hands = mp.solutions.hands
    hands = mp_hands.Hands(static_image_mode=True, max_num_hands=2)

    background_image_path = find_background_image()

    robot_radius = 20  # Double the robot radius
    hand_shadow_radius = 75

    n_robots = 5

    robots_list = []

    for robot_id in range(n_robots):
        x = random.randint(5*robot_radius, arena_width - 5*robot_radius)
        y = random.randint(5*robot_radius, arena_height - 5*robot_radius)
        robot = Robot(robot_id, x, y, robot_radius, hand_shadow_radius, robots_list, background_image_path)
        
        robots_list.append(robot)
        destinations = [
        (100, 100),
        (200, 200),
        (300, 400),
        (500, 100),
        (600, 800),
    ]
    assign_destinations_to_robots(robots_list, destinations)

    cap = cv2.VideoCapture(0)

    show_camera_feed = False

    background_image = pygame.image.load(background_image_path) if background_image_path else None

    print("Enter Main Loop")
    while True:
        ret, frame = cap.read()

        if not ret:
            break

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)  # Rotate video by 90 degrees counterclockwise
        frame = cv2.resize(frame, (arena_width, arena_height))  # Resize the video to fit the arena

        hand_coordinates = track_hands(frame, mp_hands, hands)

        screen.fill((255, 255, 255))

        if show_camera_feed:
            pygame_frame = pygame.surfarray.make_surface(frame)
            screen.blit(pygame_frame, (0, 0))
        else:
            if background_image:
                screen.blit(background_image, (0, 0))

        for hand_coord in hand_coordinates:
            pygame.draw.circle(screen, (0, 0, 0), hand_coord, hand_shadow_radius)

        for robot in robots_list:
            robot.update(screen, hand_coordinates, arena_width, arena_height)

            current_pos = (robot.x, robot.y)
            if robot.has_destination:
                print(f"Robot {robot.robot_id} is at {current_pos} → moving to {robot.get_destination()}")
            else:
                print(f"Robot {robot.robot_id} is at {current_pos} → destination reached ✅")

        
        button_width, button_height = 100, 40
        button_rect = pygame.Rect(arena_width - button_width - 10, arena_height - button_height - 10, button_width, button_height)

        button_text = "Feed" if show_camera_feed else "Feed Off"
        pygame.draw.rect(screen, (100, 100, 100), button_rect)
        button_font = pygame.font.Font(None, 24)
        button_label = button_font.render(button_text, True, (255, 255, 255))
        screen.blit(button_label, (button_rect.x + 10, button_rect.y + 10))

        pygame.display.flip()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                cap.release()
                pygame.quit()
                return
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                mouse_pos = pygame.mouse.get_pos()
                if button_rect.collidepoint(mouse_pos):
                    show_camera_feed = not show_camera_feed

    cap.release()
    pygame.quit()

if __name__ == "__main__":
    main()
