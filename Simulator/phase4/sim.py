import pygame
import random
import math
from class_robot import Robot
from class_source_and_demand import Source, Demand

ARENA_WIDTH, ARENA_HEIGHT = 900, 400
ROBOT_RADIUS = 10
N_ROBOTS = 11
CONNECTION_DISTANCE = 120

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

#calculate eucledian distance bw a and b
def distance(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)

def connect(a, b, connections):
    if (a, b) not in connections and (b, a) not in connections:
        connections.append((a, b))

#bfs algortihm to calculate number of hops from start node(source) to each robot
#attr_hop -> hop count
#attr_parent-> robot that sent message to the curr robot
def propagate_local_hop_count(source, robots, connections, attr_hop, attr_parent):
    print(f"\nStarting local token-passing from: {source.name}")
    for robot in robots:
        setattr(robot, attr_hop, None)
        setattr(robot, attr_parent, None)

    queue = [(source, 0, None)]  # (current_node, hop_count, parent)
    visited = set()

    while queue:
        current, hops, parent = queue.pop(0)

        if isinstance(current, Robot):
            if getattr(current, attr_hop) is not None:
                continue  # Ya visitado

            setattr(current, attr_hop, hops)
            setattr(current, attr_parent, parent if isinstance(parent, Robot) else None)

            print(f"\nRobot {current.robot_id} received token | Hop: {hops} | From: {parent.robot_id if isinstance(parent, Robot) else 'Source'}")

        visited.add(current)

        neighbors = []
        for a, b in connections:
            if a == current and b not in visited:
                neighbors.append(b)
            elif b == current and a not in visited:
                neighbors.append(a)

        if isinstance(current, Robot):
            print(f"  Robot {current.robot_id} - Checking range (≤ {CONNECTION_DISTANCE} units) for neighbors...")
            if neighbors:
                for n in neighbors:
                    if isinstance(n, Robot):
                        d = distance(current, n)
                        print(f"  Robot {n.robot_id} at distance: {d:.2f} units")
            else:
                print("     ⚠️ No robots in range.")

        for neighbor in neighbors:
            if isinstance(neighbor, Robot):
                current_hop = getattr(neighbor, attr_hop)
                if current_hop is None:
                    print(f"Passing token to Robot {neighbor.robot_id}")
                    queue.append((neighbor, hops + 1, current))


#check if there is an actual path
def is_path_exists(source, demand, robots, connections):
    visited = set() #keeps track of visited nodes
    queue = [source]
    while queue:
        current = queue.pop(0) #take 1st node in queue 
        if current == demand:
            return True
        visited.add(current)
        #explore nodes neighbors
        for a, b in connections:
            neighbor = b if a == current else a if b == current else None
            if neighbor and neighbor not in visited:
                #append unbisited nodes neoghbor to queuee
                queue.append(neighbor)
    return False

#finds best robot (smallest hop count) direclt connected to the next node(taget)

def trace_path_to(target_node, robots, connections, hop_attr, parent_attr):
    connected_robots = []

    for robot in robots:
        #check if robot connected to trget node
        if any((a == robot and b == target_node) or (b == robot and a == target_node) for a, b in connections):
            hop_value = getattr(robot, hop_attr)

            if hop_value is not None:
                # Add robot and its hop count.
                connected_robots.append((robot, hop_value))

    if connected_robots:
        return min(connected_robots, key=lambda x: x[1])[0]
    return None

def get_path(robot, parent_attr):
    path = []
    while isinstance(robot, Robot):
        path.append(robot)
        robot = getattr(robot, parent_attr)
    return list(reversed(path))

def main():
    pygame.init()
    screen = pygame.display.set_mode((ARENA_WIDTH, ARENA_HEIGHT))
    pygame.display.set_caption("Hop Count - Source & Demand Paths Compared")

    source = Node("Source", 50, ARENA_HEIGHT // 2, (255, 0, 0))
    demand = Node("Demand", ARENA_WIDTH - 50, ARENA_HEIGHT // 2, (0, 128, 0))

    connected = False
    attempts = 0
    # robots created and connecting with each other until there's a valid path from source to demand
    while not connected:
        attempts += 1
        robots = []
        connections = []

        for i in range(N_ROBOTS):
            x = random.randint(60, ARENA_WIDTH - 60)
            y = random.randint(60, ARENA_HEIGHT - 60)
            robots.append(Robot(i, x, y, ROBOT_RADIUS))

        #connect robots when close in distance
        for i in range(len(robots)):
            for j in range(i + 1, len(robots)):
                if distance(robots[i], robots[j]) <= CONNECTION_DISTANCE:
                    connect(robots[i], robots[j], connections)

        for robot in robots:
            if distance(robot, source) <= CONNECTION_DISTANCE:
                connect(robot, source, connections)
            if distance(robot, demand) <= CONNECTION_DISTANCE:
                connect(robot, demand, connections)

        connected = is_path_exists(source, demand, robots, connections)

    print(f"Connected after {attempts} attempts.")

    #hop count and parent robot gets assigned to do the bfs
    propagate_local_hop_count(source, robots, connections, 'hop_from_source', 'parent_from_source')
    propagate_local_hop_count(demand, robots, connections, 'hop_from_demand', 'parent_from_demand')

    #debugging
    print("\n--- DEBUGGING ---")
    for r in robots:
        total_hops = None
        if r.hop_from_source is not None and r.hop_from_demand is not None:
            total_hops = r.hop_from_source + r.hop_from_demand
        print(f"Robot {r.robot_id} | SourceHop: {r.hop_from_source} | DemandHop: {r.hop_from_demand} | TotalHop: {total_hops}")
    print("------------------")

    
    #best connected robot to de demnad or source(smallest)
    last_from_source = trace_path_to(demand, robots, connections, 'hop_from_source', 'parent_from_source')
    last_from_demand = trace_path_to(source, robots, connections, 'hop_from_demand', 'parent_from_demand')

    #Full path from last robot to source or demand
    path_source = get_path(last_from_source, 'parent_from_source') if last_from_source else []
    path_demand = get_path(last_from_demand, 'parent_from_demand') if last_from_demand else []

    #debugging for printing paths
    print("\n>>> Source ➔ Demand robot IDs:")
    print([r.robot_id for r in path_source])
    print("\n>>> Demand ➔ Source robot IDs:")
    print([r.robot_id for r in path_demand])

    ids_source = [r.robot_id for r in path_source]
    ids_demand = [r.robot_id for r in path_demand]

    #debugging to compare if the path from source to demand was the same frm demand to source
    print("\n PATH COMPARISON:")
    if ids_source == ids_demand:
        print("Paths are exactly the same.")
    elif ids_source == list(reversed(ids_demand)):
        print(" Paths are exact inverses.")
    else:
        print("Paths are different.")

    best_path = []
    direction = ""

    len_source = len(path_source)
    len_demand = len(path_demand)
    """
    print("\nPATH COMPARISON:")
    print(f" - Source ➔ Demand length: {len_source}")
    print(f" - Demand ➔ Source length: {len_demand}")
    """
    #cchoose best path based on shortest length or least total hops
    if path_source and path_demand:
        if len_source < len_demand:
            print("Selected Source ➔ Demand (shorter path).")
            best_path = path_source
            direction = "Source ➔ Demand"
        elif len_demand < len_source:
            print("Selected Demand ➔ Source (shorter path).")
            best_path = path_demand
            direction = "Demand ➔ Source"
        else:
            total_hops_source = sum(r.hop_from_source for r in path_source if r.hop_from_source is not None)
            total_hops_demand = sum(r.hop_from_demand for r in path_demand if r.hop_from_demand is not None)
            print("Paths have the same number of steps.")
            print(f" - Total hops Source ➔ Demand: {total_hops_source}")
            print(f" - Total hops Demand ➔ Source: {total_hops_demand}")

            if total_hops_source <= total_hops_demand:
                print("Selected Source ➔ Demand (fewer or equal hops).")
                best_path = path_source
                direction = "Source ➔ Demand"
            else:
                print("Selected Demand ➔ Source (fewer hops).")
                best_path = path_demand
                direction = "Demand ➔ Source"
    elif path_source:
        print("Only Source ➔ Demand path is available.")
        best_path = path_source
        direction = "Source ➔ Demand"
    elif path_demand:
        print("Only Demand ➔ Source path is available.")
        best_path = path_demand
        direction = "Demand ➔ Source"
    else:
        print("No valid path found.")
        return

    print(f"\n>>> BEST PATH ({direction}):")
    for r in best_path:
        h = r.hop_from_source if direction == "Source ➔ Demand" else r.hop_from_demand
        print(f"Robot {r.robot_id} (hop: {h})")

    print("\nExplanation:")
    for r in robots:
        if r.hop_from_source is not None and r not in best_path:
            total_hop = None
            if r.hop_from_source is not None and r.hop_from_demand is not None:
                total_hop = r.hop_from_source + r.hop_from_demand

            print(f"Robot {r.robot_id} received the token but is not part of the final path.")
            parent = r.parent_from_source
            if parent:
                print(f"   ↪ It was reached by Robot {parent.robot_id}, but its path did not lead to the destination.")
            else:
                print(f"   ↪ It did not contribute to the path connecting the Source to the Demand.")

            if total_hop is not None:
                print(f"   Its total hop count was {total_hop}, which was higher than the the other robot in range.")



    running = True
    while running:
        screen.fill((255, 255, 255))
        source.draw(screen)
        demand.draw(screen)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        for a, b in connections:
            pygame.draw.line(screen, (210, 210, 210), (a.x, a.y), (b.x, b.y), 1)

        for i in range(len(path_source) - 1):
            pygame.draw.line(screen, (255, 0, 0), (path_source[i].x, path_source[i].y), (path_source[i+1].x, path_source[i+1].y), 3)

        for i in range(len(path_demand) - 1):
            pygame.draw.line(screen, (0, 100, 255), (path_demand[i].x, path_demand[i].y), (path_demand[i+1].x, path_demand[i+1].y), 3)

        for i in range(len(best_path) - 1):
            pygame.draw.line(screen, (0, 180, 0), (best_path[i].x, best_path[i].y), (best_path[i+1].x, best_path[i+1].y), 5)

        for robot in robots:
            color = (0, 180, 0) if robot in best_path else (0, 100, 255)
            robot.draw(screen, color=color)

        pygame.display.flip()

    pygame.quit()

if __name__ == "__main__":
    main()