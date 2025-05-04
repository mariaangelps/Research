import pygame
import random
import math
from class_robot import Robot
from class_source_and_demand import Source, Demand


def distance(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)


def connect_robots(robot_a, robot_b):
    if robot_b not in robot_a.connected_to:
        robot_a.connected_to.append(robot_b)
        robot_a.connection_progress[robot_b] = 0.0

    if robot_a not in robot_b.connected_to:
        robot_b.connected_to.append(robot_a)
        robot_b.connection_progress[robot_a] = 0.0

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
    connection_distance = 60
    n_robots = 5
    robots_list = []

    source = Source("Source", 100, 250, 25)
    demand = Demand("Demand", 900, 250, 25)

    positions = [(200, 250), (300, 250), (400, 250), (500, 250), (600, 250)]
    for i in range(n_robots):
        x, y = positions[i]
        robot = Robot(i, x, y, robot_radius)
        robot.set_new_destination()
        robot.connection_progress = {}
        robot.waiting = True if i != 0 else False
        robots_list.append(robot)

    demand.connection_progress = {}
    source.connection_progress = {}

    robots_list[0].moving = True

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

        # Robot 0 connects to source
        if distance(robots_list[0], source) < connection_distance:
            if source not in robots_list[0].connected_to:
                robots_list[0].connected_to.append(source)
                robots_list[0].connection_progress[source] = 0.0
                robots_list[0].moving = False
                propagate_source_connection(robots_list[0])

        # Animate connection progress
        for robot in robots_list:
            for neighbor in robot.connected_to:
                if neighbor not in robot.connection_progress:
                    robot.connection_progress[neighbor] = 0.0
                if robot.connection_progress[neighbor] < 1.0:
                    robot.connection_progress[neighbor] += 0.02

        # Connect and move robots in sequence
        for i in range(1, len(robots_list)):
            curr_robot = robots_list[i]
            prev_robot = robots_list[i - 1]

            if (prev_robot.connected_to_source and
                curr_robot.waiting and
                prev_robot in curr_robot.connected_to and
                curr_robot.connection_progress.get(prev_robot, 0.0) >= 1.0):
                curr_robot.waiting = False
                curr_robot.moving = True

            if distance(curr_robot, prev_robot) < connection_distance:
                connect_robots(curr_robot, prev_robot)

        # Re-propagate source connection
        propagate_source_connection(robots_list[0])

        # Connect robot 4 to demand after it's linked to robot 3
        if demand not in robots_list[4].connected_to:
            if robots_list[3] in robots_list[4].connected_to and robots_list[4].connection_progress.get(robots_list[3], 0.0) >= 1.0:
                robots_list[4].connected_to.append(demand)
                robots_list[4].connection_progress[demand] = 0.0

        # Draw connections
        for robot in robots_list:
            for neighbor in robot.connected_to:
                progress = robot.connection_progress.get(neighbor, 1.0)
                x1, y1 = robot.x, robot.y

                if isinstance(neighbor, Robot):
                    x2, y2 = neighbor.x, neighbor.y
                else:
                    x2, y2 = neighbor.x, neighbor.y

                dx = x2 - x1
                dy = y2 - y1
                x_end = x1 + dx * min(progress, 1.0)
                y_end = y1 + dy * min(progress, 1.0)

                color = (0, 200, 0) if isinstance(neighbor, Robot) else (0, 0, 0)
                pygame.draw.line(screen, color, (x1, y1), (x_end, y_end), 2)

        # Draw robots: green if connected to source, else blue
        for robot in robots_list:
            color = (0, 255, 0) if robot.connected_to_source else (0, 0, 255)
            robot.draw(screen, color=color)
            font = pygame.font.Font(None, 36)
            text = font.render(str(robot.robot_id), True, (0, 0, 0))
            screen.blit(text, (robot.x - robot_radius / 2, robot.y - robot_radius / 2))

        pygame.display.flip()

    pygame.quit()


if __name__ == "__main__":
    main()
