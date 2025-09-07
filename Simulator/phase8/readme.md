Multi-Robot Network Simulator

This simulator models a swarm of robots that self-organize to form a communication network between a Source and multiple Demand nodes. It combines graph-based hop-count routing with virtual attraction forces so the network evolves dynamically.

🖼️ Concept

- Robots are randomly placed agents (blue dots).

- Source (red node) and Demands (green nodes) are fixed endpoints.

- Connections form when robots are within the connection radius r.

- A Pivot robot is selected to minimize total hop counts (Source→Pivot + Pivot→Demands).

- Golden network (red lines) = Source→Pivot path + Pivot→Demands paths.

- Robots on the golden network (green) feel stronger attraction toward demands, pulling the chain closer together.

⚙️ Parameters
Parameter	Meaning
CONNECTION_DISTANCE (r)	Distance threshold for real links between robots/nodes
SENSE_RADIUS_R (R)	Larger radius for virtual attraction forces only
STEP_MAX	Max movement per frame (pixels)
K_ATTR_ONPATH	Attraction gain for robots on the golden network
K_ATTR_OFFPATH	Attraction gain for robots off the golden network
RECOMPUTE_EVERY	Frames between recomputation of pivot and golden paths

🔑 Algorithm Flow

1. Initialization
    Place Source and Demand nodes.
    Randomly distribute N_ROBOTS.
    Build initial connections (robots within r).

2. Hop-count analysis

    BFS from Source → each robot’s hop_from_source.
    BFS from each Demand → demand_hops[Di].
    Compute total_overall = Src + Σ(Di).

3. Pivot selection

    Find local minima (robots better than all neighbors).
    Pick pivot using pivot_key:
    Lowest total_overall
    Closest to Source
    Lowest robot_id (tie-break)

4. Golden network
    Source→Pivot shortest path.
    Pivot→Demand shortest paths.
    Mark robots in these paths as on-path (green).

5. Dynamic loop 
    Apply sink attraction (r < D < R).
    Robots move (clamped by STEP_MAX).
    Rebuild connections.

    Every RECOMPUTE_EVERY frames:
       -Recompute hops and pivot.
       -Rebuild golden paths.
       -Log pivot changes.

📊 Debugging & Outputs

- Hop table: shows hop counts from Source and each Demand + totals (saved in hop_table.txt).
- Pivot logs: prints whenever pivot changes:
    Previous vs new pivot (id, totals, src hops, demand hops).
    Top candidates with tie-break keys (total, src, id).

- Pivot badge: gold halo + label above pivot in visualization.

📌 Example Behavior

- Pivot starts at the best candidate (lowest total hops).
- As robots move under demand attraction, connections change.
- Pivot may “handoff” to neighbors with slightly better totals.
- Golden network updates accordingly, pulling robots closer to demands.

🚀 Next Steps

- Improve force model: sum forces from all demands (not just nearest) so pivot drifts between sinks.
- Add pivot stability (hysteresis) to avoid flickering between equal candidates.
- Export pivot_history.csv for analysis: (frame, pivot_id, total, src, d1, d2, …).