import pygame
import random
import math
from class_robot import Robot 

# Arena and simulation parameters
ARENA_WIDTH, ARENA_HEIGHT = 900, 400
ROBOT_RADIUS = 10
N_ROBOTS = 5
CONNECTION_DISTANCE = 80

class Node:
    def __init__(self, name, x, y, color):
        self.name = name
        self.x = x
        self.y = y
        self.color = color

    def draw(self, screen):
        pygame.draw.circle(screen, self.color, (int(self.x), int(self.y)), 12)
        font = pygame.font.Font(None, 24)
        label = font.render(self.name, True, (0, 0, 0))
        screen.blit(label, (self.x + 10, self.y - 10))

def distance(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)

def connect(a, b, connections):
    if (a, b) not in connections and (b, a) not in connections:
        connections.append((a, b))

def propagate_hop_count(source_node, robots, connections):
    visited = set()
    queue = [(source_node, 0)]

    for robot in robots:
        robot.hop_count = None
        robot.connected_to_source = False

    while queue:
        current, hops = queue.pop(0)

        if isinstance(current, Robot):
            current.hop_count = hops
            current.connected_to_source = True

        visited.add(current)

        for a, b in connections:
            neighbor = None
            if a == current and b not in visited:
                neighbor = b
            elif b == current and a not in visited:
                neighbor = a

            if neighbor and neighbor not in visited:
                queue.append((neighbor, hops + 1))

def all_connected_to_source(robots):
    return all(robot.connected_to_source for robot in robots)

def main():
    print("Simulation started")

    pygame.init()
    screen = pygame.display.set_mode((ARENA_WIDTH, ARENA_HEIGHT))
    pygame.display.set_caption("Robot Arena with Hop Count")

    robots_list = []
    for robot_id in range(N_ROBOTS):
        x = random.randint(5 * ROBOT_RADIUS, ARENA_WIDTH - 5 * ROBOT_RADIUS)
        y = random.randint(5 * ROBOT_RADIUS, ARENA_HEIGHT - 5 * ROBOT_RADIUS)
        robot = Robot(robot_id, x, y, ROBOT_RADIUS)
        robots_list.append(robot)

    source = Node("Source", 50, ARENA_HEIGHT // 2, (255, 0, 0))
    demand = Node("Demand", ARENA_WIDTH - 50, ARENA_HEIGHT // 2, (0, 128, 0))

    destinations = [
        (100, 100),
        (200, 200),
        (300, 300),
        (500, 100),
        (600, 300),
    ]

    def assign_destinations(robots_list, destinations):
        for i, robot in enumerate(robots_list):
            if i < len(destinations):
                robot.set_destination(*destinations[i])
                print(f"Robot {robot.robot_id} assigned destination: {destinations[i]}")

    assign_destinations(robots_list, destinations)

    connections = []
    clock = pygame.time.Clock()
    simulation_complete = False
    source_already_connected = False
    demand_already_connected = False
    alignment_in_progress = False
    alignment_targets = []
    running = True

    while running:
        screen.fill((255, 255, 255))

        source.draw(screen)
        demand.draw(screen)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        for robot in robots_list:
            robot.update()

        # Connect robots to nearby robots within a threshold distance
        for i in range(len(robots_list)):
            for j in range(i + 1, len(robots_list)):
                if distance(robots_list[i], robots_list[j]) < CONNECTION_DISTANCE:
                    connect(robots_list[i], robots_list[j], connections)

        # Allow only one robot to connect to the source
        if not source_already_connected:
            for robot in robots_list:
                if distance(robot, source) < CONNECTION_DISTANCE:
                    connect(robot, source, connections)
                    source_already_connected = True
                    print(f"Robot {robot.robot_id} connected to Source")
                    break

        # Allow only one robot to connect to the demand
        if not demand_already_connected:
            for robot in robots_list:
                if distance(robot, demand) < CONNECTION_DISTANCE:
                    connect(robot, demand, connections)
                    demand_already_connected = True
                    print(f"Robot {robot.robot_id} connected to Demand")
                    break

        # Calculate hop counts from the source through the network
        propagate_hop_count(source, robots_list, connections)

        # Check if demand is connected to a robot that is in the source network
        demand_connected = any(
            (a == demand and isinstance(b, Robot) and b.connected_to_source) or
            (b == demand and isinstance(a, Robot) and a.connected_to_source)
            for a, b in connections
        )

        # If all robots are connected to the source and demand is connected, stop movement and align
        if all_connected_to_source(robots_list) and demand_connected and not simulation_complete:
            simulation_complete = True
            for robot in robots_list:
                robot.moving = False

            print("All robots connected. Movement stopped.")
            print("--- Hop Counts ---")
            for robot in robots_list:
                print(f"Robot {robot.robot_id} | Hop Count: {robot.hop_count}")
            print("-------------------")

            print("--- Final Connections ---")
            for a, b in connections:
                name_a = a.robot_id if hasattr(a, 'robot_id') else a.name
                name_b = b.robot_id if hasattr(b, 'robot_id') else b.name
                print(f"{name_a} <--> {name_b}")
            print("-------------------------")


            # Begin alignment of robots in a straight line between source and demand
            alignment_in_progress = True
            alignment_targets = []
            spacing = (demand.x - source.x) / (len(robots_list) + 1)
            y_line = source.y

            for i, robot in enumerate(sorted(robots_list, key=lambda r: r.robot_id)):
                target_x = source.x + (i + 1) * spacing
                alignment_targets.append((robot, target_x, y_line))

        # Gradually move robots to their alignment positions
        if alignment_in_progress:
            all_aligned = True
            for robot, target_x, target_y in alignment_targets:
                dx = target_x - robot.x
                dy = target_y - robot.y
                if abs(dx) > 1:
                    robot.x += dx * 0.05
                    all_aligned = False
                if abs(dy) > 1:
                    robot.y += dy * 0.05
                    all_aligned = False

            if all_aligned:
                print("Robots successfully aligned.")
                alignment_in_progress = False

        # Draw connection lines
        for a, b in connections:
            pygame.draw.line(screen, (0, 0, 0), (a.x, a.y), (b.x, b.y), 2)
            
        # Draw robots, green if connected to source, blue otherwise
        for robot in robots_list:
            color = (0, 200, 0) if robot.connected_to_source else (0, 0, 255)
            robot.draw(screen, color=color)

        

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()

if __name__ == "__main__":
    main()
