import pygame
import random
import math
from collections import deque
from class_robot import Robot
from class_source_and_demand import Source, Demand
import time

# =====================================================================
#                           CONFIG / CONSTANTS
# =====================================================================

ARENA_WIDTH, ARENA_HEIGHT = 800, 300
ROBOT_RADIUS = 6
N_ROBOTS = 120
N_DEMANDS = 25

CONNECTION_DISTANCE = 120
SENSE_RADIUS_R = 200               # big sensing radius (R)
CONNECT_RADIUS_r = CONNECTION_DISTANCE   # connection radius (r)
STEP_MAX = 2.4                  # max movement per frame

K_ATTR_ONPATH = 0.60               # main attraction if robot is on golden network
K_ATTR_OFFPATH = 1.00              # main attraction if robot is off the network
K_LADDER = 0.8                     # on-path "ladder" to neighbors with lower hops

RECOMPUTE_EVERY = 10               # recompute pivot + paths every N frames
SOURCE_NODE = None                 # set in main()


ALIGN_DOT_THRESHOLD = 0.0

NETWORK_LEASH = int(CONNECTION_DISTANCE * 1.15)  # keep close to network
BRANCH_K = 0.9                     # push toward network when aligning
DIRECT_K = 1.0                     # push directly toward demand when not aligned

# ---------------------------------------------------------------------
#               Spacing / Virtual Repulsion parameters
# ---------------------------------------------------------------------
DESIRED_GAP       = int(CONNECTION_DISTANCE * 0.60)  # target spacing (< comms range)
REPULSION_RANGE   = int(CONNECTION_DISTANCE * 0.80)  # react before near the edge
K_REPULSION       = 1.10                              # strength of pairwise push
MAX_REPULSION_CAP = 1.20                              # cap so repulsion doesn’t dominate
MICRO_SEP         = 2*ROBOT_RADIUS + 2                # hard overlap guard

# Optional: node halo repulsion (keeps a small buffer around source/demands)
NODE_HALO_EXTRA   = 6
NODE_HALO_K       = 0.6

# ---- Anchor/Helper behavior ----
ANCHOR_LOCK_RADIUS = 0.0  # 0.0 means fully locked (no drift)

# Spring spacing between directly connected robot neighbors
K_SPRING_ATTR   = 0.25                         # pull if farther than desired
K_SPRING_REPEL  = 0.90                         # push if closer than desired
SPRING_DESIRED  = DESIRED_GAP                  # target spacing
SPRING_MAX_DIST = CONNECTION_DISTANCE          # only pull within comms

# === Debug reach logs ===
DEMAND_FIRST_REACH = {}   # demand_name -> (robot_id, frame) when first reached
DEMAND_REACHERS   = {}    # demand_name -> set(robot_id) that reached

# obstacles removed entirely
obstacles = []  # keep empty so nothing references it


# =====================================================================
#                             BASIC TYPES
# =====================================================================

class Node:
    def __init__(self, name, x, y, color, radius=9):
        self.name = name
        self.x = x
        self.y = y
        self.color = color
        self.radius = radius
        self.anchor_robot = None  # first robot that touches becomes the anchor

    def draw(self, screen):
        pygame.draw.circle(screen, self.color, (int(self.x), int(self.y)), self.radius)
        font = pygame.font.Font(None, 24)
        label = font.render(self.name, True, (0, 0, 0))
        screen.blit(label, (self.x + 10, self.y - 10))


# =====================================================================
#                             UTILITIES
# =====================================================================

def distance(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)

def connect(a, b, connections):
    if (a, b) not in connections and (b, a) not in connections:
        connections.append((a, b))

def neighbors_of(node, connections):
    """Generator over neighbors of 'node' in undirected 'connections'."""
    for a, b in connections:
        if a == node:
            yield b
        elif b == node:
            yield a

def bfs_shortest_path(start, goal, connections):
    """Unweighted shortest path by hops in the undirected graph; includes start & goal."""
    if start == goal:
        return [start]
    q = deque([start])
    parent = {start: None}
    while q:
        cur = q.popleft()
        for nxt in neighbors_of(cur, connections):
            if nxt not in parent:
                parent[nxt] = cur
                if nxt == goal:
                    # reconstruct path
                    path = [goal]
                    while path[-1] is not None:
                        prev = parent[path[-1]]
                        if prev is None:
                            break
                        path.append(prev)
                    path.reverse()
                    return path
                q.append(nxt)
    return []

def is_path_exists(source, demands, robots, connections):
    """True if ALL demand nodes are reachable from Source."""
    target = set(demands)
    reached = set()
    seen = {source}
    q = deque([source])

    while q:
        cur = q.popleft()
        if cur in target:
            reached.add(cur)
            if len(reached) == len(target):
                return True
        # expand neighbors
        for a, b in connections:
            nxt = None
            if a == cur:
                nxt = b
            elif b == cur:
                nxt = a
            if nxt is not None and nxt not in seen:
                seen.add(nxt)
                q.append(nxt)
    return False

def clamp_step(dx, dy, max_step):
    mag = math.hypot(dx, dy)
    if mag == 0:
        return 0.0, 0.0
    if mag <= max_step:
        return dx, dy
    s = max_step / mag
    return dx * s, dy * s

def rebuild_connections(robots, source, demands):
    """Rebuild edges based on current positions."""
    connections = []
    # robot–robot
    for i in range(len(robots)):
        for j in range(i + 1, len(robots)):
            if distance(robots[i], robots[j]) <= CONNECTION_DISTANCE:
                connect(robots[i], robots[j], connections)
    # robot–source / robot–demands
    for rb in robots:
        if distance(rb, source) <= CONNECTION_DISTANCE:
            connect(rb, source, connections)
        for d in demands:
            if distance(rb, d) <= CONNECTION_DISTANCE:
                connect(rb, d, connections)
    return connections


# =====================================================================
#                          HOP / METRIC COMPUTATION
# =====================================================================

def bfs_hops_from(start_node, robots, connections):
    """
    Returns {robot: hops} representing hop distance from 'start_node' to each
    reachable robot using BFS over undirected edges. Unreachable -> None.
    """
    INF = None
    hops = {r: INF for r in robots}
    visited = set([start_node])
    q = deque([(start_node, 0)])

    while q:
        cur, d = q.popleft()
        # record hop for robots
        if isinstance(cur, Robot):
            if hops[cur] is None or d < hops[cur]:
                hops[cur] = d
        # expand neighbors
        for a, b in connections:
            nxt = None
            if a == cur and b not in visited:
                nxt = b
            elif b == cur and a not in visited:
                nxt = a
            if nxt is not None:
                visited.add(nxt)
                q.append((nxt, d + 1))
    return hops

def compute_all_hops_and_totals(source, demands, robots, connections):
    """
    For each robot r:
      r.hop_from_source
      r.demand_hops = {d.name: hops}
      r.total_overall = hop_from_source + sum(hops from each demand)
                        (None if unreachable from source or from any demand).
    """
    # From source
    hop_src = bfs_hops_from(source, robots, connections)

    # From each demand
    hop_demands = {}  # name -> {robot: hops}
    for d in demands:
        hop_demands[d.name] = bfs_hops_from(d, robots, connections)

    # Attach
    for r in robots:
        r.hop_from_source = hop_src[r]
        r.demand_hops = {dn: hop_demands[dn][r] for dn in hop_demands}

        # total_overall
        parts = []
        if r.hop_from_source is not None:
            parts.append(r.hop_from_source)
        else:
            r.total_overall = None
            continue
        for dn, hv in r.demand_hops.items():
            if hv is None:
                r.total_overall = None
                break
            parts.append(hv)
        else:
            r.total_overall = sum(parts)

def robot_neighbors(rb, connections):
    """Return set of robot neighbors directly connected to rb."""
    neigh = set()
    for a, b in connections:
        if a is rb and isinstance(b, Robot):
            neigh.add(b)
        elif b is rb and isinstance(a, Robot):
            neigh.add(a)
    return neigh

def robot_metric(rb, metric="total"):
    """
    Metric selector:
      - "total": r.total_overall (DEFAULT)
      - "source": r.hop_from_source
      - "max_demand": max over demands hops
    """
    if metric == "total":
        return getattr(rb, "total_overall", None)
    elif metric == "source":
        return getattr(rb, "hop_from_source", None)
    elif metric == "max_demand":
        if not hasattr(rb, "demand_hops") or not rb.demand_hops:
            return None
        vals = [v for v in rb.demand_hops.values() if v is not None]
        return max(vals) if vals else None
    else:
        return getattr(rb, "total_overall", None)

def is_local_minimum(rb, connections, metric="total"):
    #True if rb.metric is strictly less than all neighbor robots' metrics.
    mv = robot_metric(rb, metric)
    if mv is None:
        return False
    for nb in robot_neighbors(rb, connections):
        nv = robot_metric(nb, metric)
        if nv is None:
            continue
        if mv > nv:
            return False
    return True

def find_local_minima(robots, connections, metric="total"):
    return [r for r in robots if is_local_minimum(r, connections, metric=metric)]

def pivot_key(r):
    """
    Pivot tie-breaker order:
      1) Lowest total_overall
      2) Closest to the source (lowest hop_from_source)
      3) Lowest robot_id
    """
    total = robot_metric(r, "total")
    src = getattr(r, "hop_from_source", None)
    if total is None:
        total = float("inf")
    if src is None:
        src = 10**9
    return (total, src, r.robot_id)


# =====================================================================
#                          PATH & CHOICE HELPERS
# =====================================================================

def unconnected_demands(demands, demand_paths):
    """Demands that don’t yet have a path."""
    out = []
    for d in demands:
        p = demand_paths.get(d, [])
        if not p:
            out.append(d)
    return out

def nearest_on_network(rb, robots_in_union):
    """Nearest node (Robot/Node) on the joint network to robot rb."""
    best, bd = None, float('inf')
    for n in robots_in_union:
        d = math.hypot(rb.x - n.x, rb.y - n.y)
        if d < bd:
            best, bd = n, d
    return best, bd

def nearest_demand_and_dist(p, demands):
    best = (None, float('inf'))
    for d in demands:
        dist = math.hypot(p.x - d.x, p.y - d.y)
        if dist < best[1]:
            best = (d, dist)
    return best  # (demand, distance)

def decide_branch_or_direct(rb, robots_in_union, demands, demand_paths):
    """
        if dot( dir_to_nearby_network_robot , dir_to_closest_unconnected_demand ) > 0:
            choose "find_branch"
        else:
            choose "connect_direct"
    """
    # unconnected demands
    unconn = [d for d in demands if not demand_paths.get(d)]
    if not unconn:
        return None

    # closest unconnected demand
    dem = min(unconn, key=lambda d: math.hypot(d.x - rb.x, d.y - rb.y))
    v_dem = (dem.x - rb.x, dem.y - rb.y)
    mag_d = math.hypot(*v_dem)
    if mag_d < 1e-9:
        return ("connect_direct", None, dem)

    # nearest network robot
    if not robots_in_union:
        return ("connect_direct", None, dem)
    net = min(robots_in_union, key=lambda r: math.hypot(r.x - rb.x, r.y - rb.y))
    v_net = (net.x - rb.x, net.y - rb.y)
    mag_n = math.hypot(*v_net)
    if mag_n < 1e-9:
        return ("connect_direct", None, dem)

    u_dem = (v_dem[0] / mag_d, v_dem[1] / mag_d)
    u_net = (v_net[0] / mag_n, v_net[1] / mag_n)
    dot = u_net[0]*u_dem[0] + u_net[1]*u_dem[1]

    if dot > ALIGN_DOT_THRESHOLD:  # strictly > 0
        return ("find_branch", net, dem)
    else:
        return ("connect_direct", None, dem)

def leash_to_network(rb, robots_in_union, max_dist=NETWORK_LEASH):
    #Corrective vector to pull rb toward network if it drifts too far."""
    net, d = nearest_on_network(rb, robots_in_union)
    if net is None or d <= max_dist:
        return (0.0, 0.0)
    dx, dy = (net.x - rb.x), (net.y - rb.y)
    if d < 1e-9:
        return (0.0, 0.0)
    excess = d - max_dist
    fx = (dx / d) * min(1.0, excess / max_dist)
    fy = (dy / d) * min(1.0, excess / max_dist)
    return (fx, fy)

def propagate_local_hop_count(start_node, robots, connections, attr_hop, attr_parent, debug=False):
    """
    Writes on each robot the minimum hop count from start_node using BFS.
    Non-robot Node objects relay without increasing hop.
    """
    for r in robots:
        setattr(r, attr_hop, None)
        setattr(r, attr_parent, None)

    q = deque()
    seen = set([start_node])

    # seed: neighbors of start_node
    for nb in neighbors_of(start_node, connections):
        if isinstance(nb, Robot):
            setattr(nb, attr_hop, 1)
            setattr(nb, attr_parent, start_node if isinstance(start_node, Robot) else None)
            q.append(nb)
            seen.add(nb)
        else:
            q.append(nb)
            seen.add(nb)

    while q:
        cur = q.popleft()
        cur_hop = getattr(cur, attr_hop, None) if isinstance(cur, Robot) else None
        for nxt in neighbors_of(cur, connections):
            if nxt not in seen:
                seen.add(nxt)
                q.append(nxt)
            if isinstance(cur, Robot) and isinstance(nxt, Robot):
                candidate = (cur_hop if cur_hop is not None else 0) + 1
                prev = getattr(nxt, attr_hop)
                if prev is None or candidate < prev:
                    setattr(nxt, attr_hop, candidate)
                    setattr(nxt, attr_parent, cur)


# =====================================================================
#                     VIRTUAL REPULSION & SPRING SPACING
# =====================================================================

def pairwise_repulsion(rb, robots):
    """
    Virtual repulsion: sum push-away vectors from neighbors that are too close.
    - Zero beyond DESIRED_GAP.
    - Strong near-contact kick when overlapping.
    """
    fx = fy = 0.0
    for nb in robots:
        if nb is rb:
            continue
        dx = rb.x - nb.x
        dy = rb.y - nb.y
        adx = abs(dx); ady = abs(dy)
        if adx > REPULSION_RANGE or ady > REPULSION_RANGE:
            continue

        dist = math.hypot(dx, dy)
        if dist <= 1e-9:
            # identical position → tiny random nudge
            jitter = 0.5
            fx += (random.random()-0.5) * jitter
            fy += (random.random()-0.5) * jitter
            continue

        # Hard overlap guard
        if dist < MICRO_SEP:
            ux, uy = dx/dist, dy/dist
            kick = K_REPULSION * (MICRO_SEP - dist) / MICRO_SEP
            fx += 3.0 * kick * ux
            fy += 3.0 * kick * uy
            continue

        # Soft linear ramp inside desired spacing
        if dist < DESIRED_GAP:
            ux, uy = dx/dist, dy/dist
            mag = K_REPULSION * (1.0 - dist / DESIRED_GAP)
            fx += mag * ux
            fy += mag * uy

    return fx, fy

def node_repulsion(rb, nodes, desired=MICRO_SEP+NODE_HALO_EXTRA, k=NODE_HALO_K):
    #Small buffer around source/demand nodes (optional)."""
    fx = fy = 0.0
    for n in nodes:
        dx, dy = rb.x - n.x, rb.y - n.y
        dist = math.hypot(dx, dy)
        if dist < 1e-9:
            continue
        if dist < desired:
            ux, uy = dx/dist, dy/dist
            mag = k * (1.0 - dist/desired)
            fx += mag * ux
            fy += mag * uy
    return fx, fy

def edge_spacing_force(rb, connections):
    """
    Spring-like spacing only to DIRECTLY CONNECTED robot neighbors.
    - If dist < SPRING_DESIRED: push away (repel)
    - If SPRING_DESIRED <= dist < SPRING_MAX_DIST: pull together (cohere)
    """
    fx = fy = 0.0
    # collect robot neighbors
    neigh = []
    for a, b in connections:
        if a is rb and isinstance(b, Robot):
            neigh.append(b)
        elif b is rb and isinstance(a, Robot):
            neigh.append(a)

    for nb in neigh:
        dx, dy = (nb.x - rb.x), (nb.y - rb.y)
        dist = math.hypot(dx, dy)
        if dist < 1e-9:
            continue

        ux, uy = dx/dist, dy/dist

        if dist < SPRING_DESIRED:
            # too close → push away
            mag = K_SPRING_REPEL * (1.0 - dist/SPRING_DESIRED)
            fx -= mag * ux
            fy -= mag * uy
        elif dist < SPRING_MAX_DIST:
            # too far but still in comms → pull together
            mag = K_SPRING_ATTR * ((dist - SPRING_DESIRED) / (SPRING_MAX_DIST - SPRING_DESIRED + 1e-6))
            fx += mag * ux
            fy += mag * uy
        # else: beyond comms ⇒ no spring pull
    return fx, fy


# =====================================================================
#                      MOVEMENT / ATTRACTION LOGIC
# =====================================================================

def check_and_color_robots(robots, demands, frame=None):
    """
    - First robot to touch a demand becomes its ANCHOR: locked in place.
    - Later robots that touch same demand become HELPERS: purple but still free to move.
    """
    global DEMAND_FIRST_REACH, DEMAND_REACHERS

    for r in robots:
        # If already an anchor, nothing to do
        if getattr(r, "anchor_locked", False):
            continue

        # Detect demand contact
        touched = None
        for d in demands:
            d_rad = getattr(d, "radius", 12)
            if math.hypot(r.x - d.x, r.y - d.y) <= d_rad + ROBOT_RADIUS:
                touched = d
                break

        if touched is None:
            continue

        # Mark purple
        r.at_demand = True

        # If the demand has no anchor yet -> lock this robot as anchor
        if getattr(touched, "anchor_robot", None) is None:
            touched.anchor_robot = r
            r.anchor_locked = True   # this one will no longer move
        else:
            # helper purple (still moves)
            r.anchor_locked = False
            r.helper_mode = True

        # Logs
        if frame is not None:
            nm = touched.name
            DEMAND_REACHERS.setdefault(nm, set())
            DEMAND_FIRST_REACH.setdefault(nm, None)
            if r.robot_id not in DEMAND_REACHERS[nm]:
                DEMAND_REACHERS[nm].add(r.robot_id)
                if DEMAND_FIRST_REACH[nm] is None:
                    DEMAND_FIRST_REACH[nm] = (r.robot_id, frame)
                    print(f"[REACHED-FIRST] {nm} reached by Robot {r.robot_id} at frame {frame}")
                else:
                    print(f"[REACHED] {nm} also reached by Robot {r.robot_id} at frame {frame}")
                if all(DEMAND_FIRST_REACH[k] is not None for k in DEMAND_FIRST_REACH):
                    summary = {k: f"robot {v[0]} @frame {v[1]}" for k, v in DEMAND_FIRST_REACH.items()}
                    print("[SUMMARY] First reach per demand:", summary)

def apply_sink_attraction(robots, demands, robots_in_union, connections, current_frame=None):
    """
    Move robots using combined fields:
    - Skip only ANCHOR robots (helpers keep moving even if purple).
    - On-path: target best demand by hops + ladder + branch/direct + leash.
    - Off-path: attracted to nearest network node (fallback: Source).
    - Repulsion + edge springs + node halo enforce spacing.
    """
    global SOURCE_NODE, demand_paths
    if SOURCE_NODE is None:
        return

    for rb in robots:
        # Skip ONLY the locked anchor robots
        if getattr(rb, "anchor_locked", False):
            continue

        # === 1) Main target and base attraction ===
        if rb in robots_in_union:
            target_d, _ = best_demand_for_robot(rb, demands)
            if target_d is None:
                target_d, _ = nearest_demand_and_dist(rb, demands)
            target = target_d
            k_main = K_ATTR_ONPATH
        else:
            nt, _ = nearest_on_network(rb, robots_in_union)
            target = nt if nt is not None else SOURCE_NODE
            k_main = K_ATTR_OFFPATH

        # Base donut attraction
        fx = fy = 0.0
        dist = math.hypot(target.x - rb.x, target.y - rb.y)
        if CONNECT_RADIUS_r < dist < SENSE_RADIUS_R:
            vx, vy = (target.x - rb.x), (target.y - rb.y)
            fx = k_main * vx / (dist + 1e-6)
            fy = k_main * vy / (dist + 1e-6)

        # Slight nudge when off-path and already within r
        if rb not in robots_in_union and dist <= CONNECT_RADIUS_r:
            vx, vy = (target.x - rb.x), (target.y - rb.y)
            fx += 0.18 * vx / (dist + 1e-6)
            fy += 0.18 * vy / (dist + 1e-6)

        # === 2) On-path helpers: ladder + branch/direct + leash ===
        if rb in robots_in_union:
            d_best, _ = best_demand_for_robot(rb, demands)
            dname = d_best.name if d_best else None
            nxt = neighbor_with_lower_hop_to_demand(rb, dname, connections)

            # ladder toward neighbor with lower hop-to-demand
            if nxt is not None:
                dx, dy = (nxt.x - rb.x), (nxt.y - rb.y)
                mag = math.hypot(dx, dy)
                if mag > 1e-9:
                    fx += K_LADDER * dx / mag
                    fy += K_LADDER * dy / mag
            else:
                # fallback: point directly at best demand
                if d_best is not None:
                    vx2, vy2 = (d_best.x - rb.x), (d_best.y - rb.y)
                    dist2 = math.hypot(vx2, vy2)
                    if dist2 > 1e-6:
                        fx += K_LADDER * vx2 / dist2
                        fy += K_LADDER * vy2 / dist2

            # branch or direct using your dot>0 rule
            decision = decide_branch_or_direct(rb, robots_in_union, demands, demand_paths)
            if decision is not None:
                mode, net_target, dem_target = decision
                if mode == "find_branch" and net_target is not None and dem_target is not None:
                    dxn, dyn = net_target.x - rb.x, net_target.y - rb.y
                    dn = math.hypot(dxn, dyn)
                    if dn > 1e-9:
                        fx += BRANCH_K * dxn / dn
                        fy += BRANCH_K * dyn / dn
                    dxd, dyd = dem_target.x - rb.x, dem_target.y - rb.y
                    dd = math.hypot(dxd, dyd)
                    if dd > 1e-9:
                        fx += 0.35 * dxd / dd
                        fy += 0.35 * dyd / dd
                elif mode == "connect_direct" and dem_target is not None:
                    dxd, dyd = dem_target.x - rb.x, dem_target.y - rb.y
                    dd = math.hypot(dxd, dyd)
                    if dd > 1e-9:
                        fx += DIRECT_K * dxd / dd
                        fy += DIRECT_K * dyd / dd

            # network leash
            lx, ly = leash_to_network(rb, robots_in_union, NETWORK_LEASH)
            fx += lx
            fy += ly

        # === 3) Repulsion & spacing ===
        # (a) pairwise repulsion (collision preventer)
        rfx, rfy = pairwise_repulsion(rb, robots)

        # cap repulsion relative to attraction so mission pull stays primary
        rep_mag = math.hypot(rfx, rfy)
        att_mag = math.hypot(fx, fy)
        if rep_mag > 1e-9:
            cap = MAX_REPULSION_CAP * (att_mag + 1e-6)
            if rep_mag > cap:
                s = cap / rep_mag
                rfx *= s; rfy *= s
        fx += rfx
        fy += rfy

        # (b) edge springs: too close -> repel; too far (within comms) -> pull
        sfx, sfy = edge_spacing_force(rb, connections)
        fx += sfx
        fy += sfy

        # (c) optional small halo around nodes
        nrx, nry = node_repulsion(rb, [SOURCE_NODE] + demands,
                                  desired=MICRO_SEP + NODE_HALO_EXTRA,
                                  k=NODE_HALO_K)
        fx += nrx
        fy += nry

        # === 4) Clamp + light damping (reduce jitter) ===
        dx, dy = clamp_step(fx, fy, STEP_MAX)

        # keep simple velocity memory on robot
        if not hasattr(rb, "vx"):
            rb.vx, rb.vy = 0.0, 0.0
        alpha = 0.65   # weight for new acceleration
        beta  = 0.20   # damping on previous velocity
        rb.vx = beta*rb.vx + alpha*dx
        rb.vy = beta*rb.vy + alpha*dy

        # final clamp to STEP_MAX
        mv = math.hypot(rb.vx, rb.vy)
        if mv > STEP_MAX:
            s = STEP_MAX / mv
            rb.vx *= s; rb.vy *= s

        # apply move within bounds
        rb.x = max(0, min(ARENA_WIDTH,  rb.x + rb.vx))
        rb.y = max(0, min(ARENA_HEIGHT, rb.y + rb.vy))


# =====================================================================
#                       BEST DEMAND / NEIGHBOR CHOICES
# =====================================================================

def best_demand_for_robot(rb, demands):
    """
    Returns (demand, hops) minimizing hops from that demand to robot.
    (None, None) if undefined.
    """
    if not hasattr(rb, "demand_hops") or not rb.demand_hops:
        return (None, None)
    best_nm, best_h = None, None
    for d in demands:
        hv = rb.demand_hops.get(d.name, None)
        if hv is None:
            continue
        if best_h is None or hv < best_h:
            best_nm, best_h = d.name, hv
    if best_nm is None:
        return (None, None)
    for d in demands:
        if d.name == best_nm:
            return (d, best_h)
    return (None, None)

def neighbor_with_lower_hop_to_demand(rb, demand_name, connections):
    if demand_name is None:
        return None
    my_h = rb.demand_hops.get(demand_name, None)
    if my_h is None:
        return None

    cand = []
    for nb in robot_neighbors(rb, connections):
        dh = getattr(nb, "demand_hops", {}).get(demand_name, None)
        if dh is not None and dh < my_h:
            dgeom = math.hypot(nb.x - rb.x, nb.y - rb.y)
            cand.append((dh, dgeom, nb.robot_id, nb))
    if not cand:
        return None
    cand.sort(key=lambda t: (t[0], t[1], t[2]))
    return cand[0][3]


# =====================================================================
#                               MAIN
# =====================================================================

def spawn_demands_random(n, arena_w, arena_h, source, min_sep=45, margin=40, max_tries=5000):
    pts = []
    tries = 0
    while len(pts) < n and tries < max_tries:
        tries += 1
        x = random.randint(margin, arena_w - margin)
        y = random.randint(margin, arena_h - margin)
        if math.hypot(x - source.x, y - source.y) < min_sep * 1.5:
            continue
        ok = True
        for (px, py) in pts:
            if math.hypot(x - px, y - py) < min_sep:
                ok = False
                break
        if ok:
            pts.append((x, y))
    while len(pts) < n:
        x = random.randint(margin, arena_w - margin)
        y = random.randint(margin, arena_h - margin)
        pts.append((x, y))
    return pts

def main():
    pygame.init()
    clock = pygame.time.Clock()
    screen = pygame.display.set_mode((ARENA_WIDTH, ARENA_HEIGHT))
    pygame.display.set_caption("Fast joint network (Source ↔ Pivot ↔ Demands)")

    # ---- Nodes ----
    source = Node("Source", 50, ARENA_HEIGHT // 2, (255, 0, 0))
    global SOURCE_NODE
    SOURCE_NODE = source

    # ---- Demands ----
    demands = []
    demand_positions = spawn_demands_random(
        N_DEMANDS, ARENA_WIDTH, ARENA_HEIGHT, source,
        min_sep=45, margin=40
    )
    for i, (dx, dy) in enumerate(demand_positions):
        name = f"D{i+1}"
        demands.append(Node(name, dx, dy, (0, 128, 0)))

    # init reach logs
    global DEMAND_FIRST_REACH, DEMAND_REACHERS
    DEMAND_FIRST_REACH = {d.name: None for d in demands}
    DEMAND_REACHERS    = {d.name: set()  for d in demands}

    # ---- Robots ----
    robots = []
    for i in range(N_ROBOTS):
        x = random.randint(80, ARENA_WIDTH - 80)
        y = random.randint(60, ARENA_HEIGHT - 60)
        robots.append(Robot(i, x, y, ROBOT_RADIUS))

    # ---- Build connections ONCE (simple O(n²)) ----
    connections = rebuild_connections(robots, source, demands)

    # If graph doesn’t connect everything, reseed a couple of times
    attempts = 1
    MAX_ATTEMPTS = 3
    while not is_path_exists(source, demands, robots, connections) and attempts < MAX_ATTEMPTS:
        attempts += 1
        robots = []
        for i in range(N_ROBOTS):
            x = random.randint(80, ARENA_WIDTH - 80)
            y = random.randint(60, ARENA_HEIGHT - 60)
            robots.append(Robot(i, x, y, ROBOT_RADIUS))
        for r in robots:
            r.settled = False
        connections = rebuild_connections(robots, source, demands)

    # --- DEBUG SNAPSHOT ---
    print("\n--- DEBUG: Robots in range ---")
    for robot in robots:
        in_range = [other.robot_id for other in robots
                    if robot != other and distance(robot, other) <= CONNECTION_DISTANCE]
        print(f" Robot {robot.robot_id} -> {in_range}")

    print("\n--- DEBUG: Source direct connections ---")
    direct_from_source = [robot.robot_id for robot in robots if distance(source, robot) <= CONNECTION_DISTANCE]
    print(f"Source -> {direct_from_source}")

    print("\n--- DEBUG: Demand direct connections ---")
    for d in demands:
        direct_from_d = [r.robot_id for r in robots if distance(d, r) <= CONNECTION_DISTANCE]
        print(f"{d.name} -> {direct_from_d}")

    # ---- Heavy math ONCE ----
    compute_all_hops_and_totals(source, demands, robots, connections)
    for r in robots:
        vals = [v for v in getattr(r, 'demand_hops', {}).values() if v is not None]
        r.hop_from_demand = min(vals) if vals else None

    # Choose pivot: prefer local minima by total_overall, then tie-breaker
    local_mins = find_local_minima(robots, connections, metric="total")
    if local_mins:
        pivot = min(local_mins, key=pivot_key)
    else:
        candidates = [r for r in robots if getattr(r, "total_overall", None) is not None]
        pivot = min(candidates, key=pivot_key) if candidates else None

    # Initial joint paths
    global demand_paths
    if pivot:
        path_S = bfs_shortest_path(source, pivot, connections)
        demand_paths = {d: bfs_shortest_path(d, pivot, connections) for d in demands}
    else:
        path_S = []
        demand_paths = {d: [] for d in demands}

    def draw_poly(path, color=(255, 0, 0), width=3):
        for i in range(len(path) - 1):
            pygame.draw.line(screen, color, (path[i].x, path[i].y), (path[i+1].x, path[i+1].y), width)

    def draw_pivot_badge(screen, pivot, radius=12):
        if pivot is None:
            return
        pygame.draw.circle(screen, (255, 215, 0), (int(pivot.x), int(pivot.y)), radius + 6, 3)
        font = pygame.font.Font(None, 20)
        txt = font.render(f"PIVOT {pivot.robot_id}", True, (120, 90, 0))
        screen.blit(txt, (pivot.x + 10, pivot.y - 18))

    # ---- MAIN LOOP ----
    running = True
    prev_pivot = pivot
    frame = 0

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # recompute robots_in_union from current paths
        robots_in_union = set(n for n in path_S + sum(demand_paths.values(), []) if isinstance(n, Robot))

        # 1) Move robots by combined fields (attraction + repulsion + springs)
        apply_sink_attraction(robots, demands, robots_in_union, connections, frame)

        # 2) Rebuild connections with updated positions
        connections = rebuild_connections(robots, source, demands)

        # 3) Recompute pivot + paths periodically
        if frame % RECOMPUTE_EVERY == 0:
            compute_all_hops_and_totals(source, demands, robots, connections)
            for r in robots:
                vals = [v for v in getattr(r, 'demand_hops', {}).values() if v is not None]
                r.hop_from_demand = min(vals) if vals else None

            local_mins = find_local_minima(robots, connections, metric="total")
            if local_mins:
                pivot = min(local_mins, key=pivot_key)
            else:
                candidates = [r for r in robots if getattr(r, "total_overall", None) is not None]
                pivot = min(candidates, key=pivot_key) if candidates else None

            if pivot:
                path_S = bfs_shortest_path(source, pivot, connections)
                demand_paths = {d: bfs_shortest_path(d, pivot, connections) for d in demands}
            else:
                path_S = []
                demand_paths = {d: [] for d in demands}

        # 4) Range visually detected (color arrivals + anchor/helper)
        check_and_color_robots(robots, demands, frame)

        # 5) Draw
        screen.fill((255, 255, 255))
        source.draw(screen)
        for d in demands:
            d.draw(screen)

        # draw joint tree only (faster)
        draw_poly(path_S, (255, 0, 0), 3)
        for d, p in demand_paths.items():
            draw_poly(p, (255, 0, 0), 3)

        # colors: anchor deep purple, helper purple, on-chain green, off-chain blue
        robots_in_union = set(n for n in path_S + sum(demand_paths.values(), []) if isinstance(n, Robot))
        for rb in robots:
            if getattr(rb, "anchor_locked", False):
                color = (90, 0, 90)           # dark purple (anchor)
            elif getattr(rb, "at_demand", False):
                color = (128, 0, 128)         # helper purple (still moves)
            elif rb in robots_in_union:
                color = (0, 200, 0)           # green on golden chain
            else:
                color = (0, 100, 255)         # blue off chain
            rb.draw(screen, color=color)

        draw_pivot_badge(screen, pivot)

        pygame.display.flip()
        clock.tick(30)
        frame += 1

    pygame.quit()


if __name__ == "__main__":
    main()
