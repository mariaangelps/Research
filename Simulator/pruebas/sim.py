import pygame
import math
from class_robot import Robot
from class_source_and_demand import Source, Demand

def distance(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)

def connect_robots(robot_a, robot_b):
    if robot_b not in robot_a.connected_to:
        robot_a.connected_to.append(robot_b)
        print(f"Robot {robot_a.robot_id} connected to Robot {robot_b.robot_id}")
    if robot_a not in robot_b.connected_to:
        robot_b.connected_to.append(robot_a)
        print(f"Robot {robot_b.robot_id} connected to Robot {robot_a.robot_id}")
    robot_a.moving = False
    robot_b.moving = False

def propagate_source_connection(robot):
    if not robot.connected_to_source:
        robot.connected_to_source = True
        for neighbor in robot.connected_to:
            if isinstance(neighbor, Robot):
                propagate_source_connection(neighbor)

def main():
    pygame.init()
    screen = pygame.display.set_mode((1000, 500))
    pygame.display.set_caption("Robot Chain Connection")

    robot_radius = 20
    n_robots = 5
    robots_list = []

    source = Source("Source", 100, 250, 25)
    demand = Demand("Demand", 900, 250, 25)

    positions = [(200, 250), (300, 250), (400, 250), (500, 250), (600, 250)]
    for i in range(n_robots):
        x, y = positions[i]
        robot = Robot(i, x, y, robot_radius)
        robot.set_new_destination()
        robot.steps_before_connect = 100  # Delay before it can connect
        robots_list.append(robot)

    frame_counter = 0
    running = True
    while running:
        screen.fill((255, 255, 255))
        source.draw(screen)
        demand.draw(screen)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Update robot positions
        for robot in robots_list:
            if robot.moving:
                robot.update()

        # Robot 0 connects to source (after delay)
        if distance(robots_list[0], source) < 9999:  # ignore distance
            if robots_list[0].steps_before_connect > 0:
                robots_list[0].steps_before_connect -= 1
            elif source not in robots_list[0].connected_to:
                robots_list[0].connected_to.append(source)
                robots_list[0].moving = False
                print("Robot 0 connected to Source")
                propagate_source_connection(robots_list[0])

        # Robot 4 connects to demand (after delay)
        if distance(robots_list[4], demand) < 9999:
            if robots_list[4].steps_before_connect > 0:
                robots_list[4].steps_before_connect -= 1
            elif demand not in robots_list[4].connected_to:
                robots_list[4].connected_to.append(demand)
                robots_list[4].moving = False
                print("Robot 4 connected to Demand")

        # Connect robots in order, ignoring distance, but only after waiting a bit
        for i in range(1, len(robots_list)):
            curr_robot = robots_list[i]
            prev_robot = robots_list[i - 1]
            if curr_robot.steps_before_connect > 0:
                curr_robot.steps_before_connect -= 1
            else:
                connect_robots(curr_robot, prev_robot)

        # Propagate connection from robot 0
        propagate_source_connection(robots_list[0])

        # Draw connections
        for robot in robots_list:
            for neighbor in robot.connected_to:
                if isinstance(neighbor, Robot):
                    pygame.draw.line(screen, (0, 200, 0), (robot.x, robot.y), (neighbor.x, neighbor.y), 2)
                elif isinstance(neighbor, Source) or isinstance(neighbor, Demand):
                    pygame.draw.line(screen, (0, 0, 0), (robot.x, robot.y), (neighbor.x, neighbor.y), 2)

        # Draw robots
        for robot in robots_list:
            # Robot 0 is green only if connected to source
            if robot.robot_id == 0:
                color = (0, 255, 0) if source in robot.connected_to else (0, 0, 255)
            else:
                color = (0, 255, 0) if robot.connected_to_source else (0, 0, 255)

            robot.draw(screen, color=color)

            font = pygame.font.Font(None, 36)
            text = font.render(str(robot.robot_id), True, (0, 0, 0))
            screen.blit(text, (robot.x - robot_radius / 2, robot.y - robot_radius / 2))

        pygame.display.flip()
        frame_counter += 1

    pygame.quit()

if __name__ == "__main__":
    main()
