import pygame
import random
import math
from class_robot import Robot
from class_source_and_demand import Source, Demand
#SOLO FALTARIA QUE SE ALINEEEN I GUESS
def distance(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)

def connect_robots(robot_a, robot_b):
    new_connection = False
    if robot_b not in robot_a.connected_to:
        robot_a.connected_to.append(robot_b)
        new_connection = True
    if robot_a not in robot_b.connected_to:
        robot_b.connected_to.append(robot_a)
        new_connection = True
    if new_connection:
        print(f"Robots {robot_a.robot_id} and {robot_b.robot_id} are now connected")
    robot_a.moving = False
    robot_b.moving = False

def propagate_source_connection(robot):
    if not robot.connected_to_source:
        robot.connected_to_source = True
        for neighbor in robot.connected_to:
            if isinstance(neighbor, Robot):
                propagate_source_connection(neighbor)

def move_towards(robot, target, speed=1):
    dx = target.x - robot.x
    dy = target.y - robot.y
    dist = math.hypot(dx, dy)
    if dist > 1:
        robot.x += (dx / dist) * speed
        robot.y += (dy / dist) * speed

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

    positions = []
    while len(positions) < n_robots:
        x = random.randint(100, 900)
        y = random.randint(100, 400)
        too_close = any(math.hypot(x - px, y - py) < 80 for px, py in positions)
        if not too_close:
            positions.append((x, y))

    for i in range(n_robots):
        x, y = positions[i]
        robot = Robot(i, x, y, robot_radius)
        if i == 0:
            # Robot 0 irá directamente al Source
            robot.moving = True
        else:
            robot.set_new_destination()
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

        for robot in robots_list:
            if robot.moving:
                if robot.robot_id == 0 and source not in robot.connected_to:
                    move_towards(robot, source, speed=1)
                else:
                    robot.update()

        # Robot 0 se conecta al Source si está cerca
        robot_0 = robots_list[0]
        if distance(robot_0, source) < connection_distance:
            if source not in robot_0.connected_to:
                robot_0.connected_to.append(source)
                robot_0.moving = False
                print("Robot 0 connected to Source")
                propagate_source_connection(robot_0)

        # Conectar robot 4 con demanda tras un tiempo
        if frame_counter > 150:
            robot_4 = robots_list[4]
            if demand not in robot_4.connected_to:
                robot_4.connected_to.append(demand)
                robot_4.moving = False
                print("Robot 4 connected to Demand")

        # Conectar robots si están cerca
        for i in range(1, len(robots_list)):
            curr_robot = robots_list[i]
            prev_robot = robots_list[i - 1]
            if distance(curr_robot, prev_robot) < connection_distance:
                connect_robots(curr_robot, prev_robot)

        # Movimiento hacia el anterior
        if frame_counter > 100:
            for i in range(1, len(robots_list)):
                curr_robot = robots_list[i]
                prev_robot = robots_list[i - 1]
                if prev_robot not in curr_robot.connected_to:
                    move_towards(curr_robot, prev_robot, speed=1)

        # Propagar conexión desde cualquier robot que tenga conexión con el Source
        for robot in robots_list:
            if source in robot.connected_to or any(isinstance(n, Robot) and n.connected_to_source for n in robot.connected_to):
                propagate_source_connection(robot)

        # Dibujar conexiones
        for robot in robots_list:
            for neighbor in robot.connected_to:
                if isinstance(neighbor, Robot):
                    pygame.draw.line(screen, (0, 0, 0), (robot.x, robot.y), (neighbor.x, neighbor.y), 2)
                elif isinstance(neighbor, (Source, Demand)):
                    pygame.draw.line(screen, (0, 0, 0), (robot.x, robot.y), (neighbor.x, neighbor.y), 2)

        # Dibujar robots
        for robot in robots_list:
            color = (0, 255, 0) if robot.connected_to_source else (0, 0, 255)
            robot.draw(screen, color=color)
            font = pygame.font.Font(None, 36)
            text = font.render(str(robot.robot_id), True, (0, 0, 0))
            screen.blit(text, (robot.x - robot_radius / 2, robot.y - robot_radius / 2))


                # Verificar si todos están conectados al Source
        # Verificar si todos están conectados al Source
        all_connected = all(robot.connected_to_source for robot in robots_list)

        if all_connected:
            target_y = source.y
            spacing = 100  # Puedes ajustar este valor para separación horizontal
            start_x = source.x + 50  # Punto de inicio desde la fuente hacia la derecha

            for i, robot in enumerate(sorted(robots_list, key=lambda r: r.robot_id)):
                target_x = start_x + i * spacing
                dx = target_x - robot.x
                dy = target_y - robot.y

                if abs(dx) > 1:
                    robot.x += dx * 0.05  # Movimiento suave en X
                if abs(dy) > 1:
                    robot.y += dy * 0.05  # Movimiento suave en Y


        pygame.display.flip()
        frame_counter += 1

    pygame.quit()

if __name__ == "__main__":
    main()
