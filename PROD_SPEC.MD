<Avenue-Park: SF Production Engineering Spec>

This document justifies the architectural decisions made for the Avenue-Park MVP. This is not a "how-to" guide; it is a "why-this" justification for the backend transition to San Francisco (SF).

<Core Engineering Decisions>

>1.1 Decision: Finite Horizon MDP vs. Simple Sorting

Decision: We use a Finite Horizon Markov Decision Process (MDP) solved via Beam Search.
Justification: Parking is not a destination; it is a stochastic sequence. A simple sort by "proximity" fails because it ignores the probability of occupancy p_i. If the closest spot has a 90% chance of being full, the "Expected Time" to park there is actually higher than a spot 50 meters further with a 50% chance.
Algorithm Choice: Beam Search (Width: 100-1000).

Why: The state space of possible parking chains is $n!$ (factorial). For 20 spots, that's 2.4*10^18 paths. Beam search provides a near-optimal solution in O(k*n*B) time where B is the beam width.

>1.2 Decision: "Split-Edge" Graph Theory

Decision: Dynamically inject nodes into the OSMnx graph (routing.py).

Data Structure: nx.MultiDiGraph (directed multigraph).
Justification: Standard routing maps coordinates to the "nearest node" (intersection). In SF's dense one-way grid, mapping a spot to the wrong intersection could suggest a route that requires a 10-minute detour or an illegal U-turn. By splitting the edge (u, v) into (u, spot) and (spot, v), we ensure travel times are accurate to the meter.

>1.3 Decision: Asymmetric Topology (Progress Projection)

Decision: Project 2D coordinates onto a 1D "Avenue" vector in transformation.py.
Justification: Distance matrices n^2 are symmetric in Euclidean space but highly asymmetric in SF traffic.

Backtracking Penalty: We implement a 1,800s (30m) penalty for moving "backwards" on the same side of a street.

Crossing Penalty: We implement a 1,200s (20m) penalty for crossing a major avenue.
Why: This "soft constraint" approach is computationally cheaper than calculating $n^2$ true shortest paths with turn restrictions for every frame of the optimization.

>1.4 Decision: Session-Based Feature Normalization

Decision: Dynamic max-bounds calculation M_drive, M_walk, M_phi in algorithms.py.
Justification: We are optimizing a multi-objective cost function: Cost = lambda_w * Walk + lambda_e * Exit + Drive.

Seconds and Meters are not comparable. Without normalization, a 100m walk (100 units) would outweigh a 30s drive (30 units) regardless of intent.

By dividing by the session maximums, we convert all metrics into a [0, 1] scalar range, allowing the lambda weights to function as true relative priorities.

<Bottlenecks & Known Issues>

>2.1 The O(n^2) Shortest Path Problem

Problem: To build the trans_matrix, we must calculate the shortest path between every pair of spots. For 50 spots, that's 2,500 Dijkstra calls.
Solution: Current implementation uses a local distance_cache.json.
Possible Optimizations: 
 - Spatial Pruning: Only calculate transitions for spots within a specific "Forward-Facing Cone". 
 - Contraction Hierarchies: Use a high-performance routing engine (like OSRM or GraphHopper) instead of pure NetworkX.
 - Clustering spots, calculating only a few nodes to the neighboring clusters and full nodes within clusters