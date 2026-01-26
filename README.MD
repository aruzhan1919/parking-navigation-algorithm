Avenue-Park: File-by-File Technical Specification

This document provides a detailed breakdown of the internal modules for the Avenue-Park SF Parking Engine.

📁 Core Logic Modules

1. algorithms.py

Purpose: Contains the optimization solvers.

<BaseAlgorithm>: Handles dynamic normalization ($M_{drive}, M_{walk}, M_{phi}$). This is critical for SF because a long walk in the Sunset District is different from a long walk in the Financial District.

<FiniteHorizonMDP>: Uses Beam Search to solve the multi-step parking sequence. It models the Decision Tree of parking: if Spot A fails (with probability $1-p_i$), what is the next best move?

# Might give a shorter chain if it's confident enough in its chances 

<DifferentialMDP>: Same principles as Standard MDP but used differences instead of the normal metrics, calculating how the next spot help us get "better" position

# Will give a chain exactly of the required length

>>>IMPORTANT: The trans_matrix uses a progress check from the topology module to prune illegal backtracking moves, penalties for U-turns might need adjustment

2. routing.py

Purpose: Interface with OpenStreetMap (via OSMnx) and Graph Management.

<augment_graph_with_spots>: Implements Split-Edge Theory. It breaks a single road segment into multiple sub-segments to place a parking spot node exactly where it exists in the real world.

<get_route>: Calculates the physical path and travel time. 

>>>IMPORTANT: Uses OSM speed limits to calculate time, but needs to adapt real time and forecasted traffic to be accurate

Production Note: For SF, the PLACE_NAME and GRAPH_FILENAME must be updated. This module should eventually interface with a live traffic API.

3. transformation.py

Purpose: Coordinate projection and traffic logic.

<StateAdapter>: Projects 2D GPS coordinates onto a 1D vector (Avenue Progress).

<Asymmetry Enforcer>: If a user is at Spot A and wants to go to Spot B, but Spot B is "behind" them on a one-way street or requires an illegal U-turn, this module injects a 30-minute penalty to force the algorithm to find a legal route.

<Coordinate Scaling>: Uses math.cos(math.radians(lat)) to correct for the Earth's curvature. Important for SF (37.7°N).

4. utils.py

Purpose: The source of truth for the Expected Time formula.

<calculate_metrics>: Implements the math: $E[T] = P(Success) \cdot T_{success} + P(Fail) \cdot T_{fail}$.

Logic: It correctly distinguishes between "Search Time" (time spent looking at a full spot) and "Exit Time" (time spent driving away after successful parking).

📁 Data & Integration Modules

5. scenarios.py

Purpose: Configuration of the $\lambda$ weights.

Different weights for walking vs. driving based on user intent (e.g., a "Delivery" driver values walk time $10x$ more than a "Resident").

6. locations.py

Purpose: Pull up locations, starting point, destination point and a reference point

For this specific code we use pre-defined locations for visual testing purposes. For production should be aut0mated

7. app.py

Purpose: Flask API Entry point.

Handles the request lifecycle, interacts with the distance cache, and converts the algorithmic output into GeoJSON segments for the frontend.

# Just to see visually how it performs

8. stats.py & visualizer.py

Purpose: Debugging and Analysis.

>stats.py analyzes the distance_cache.json to ensure the routing engine isn't producing "hallucinated" travel times.

>visualizer.py generates Folium maps for visual verification of the "Split-Edge" logic.

9. manual_spots.json & distance_cache.json

Purpose: quick testing

In the absence of a reliable parking data API in Astana, all parking places were manually inserted by coordinates. The distances between spots and locations are stored in distance_cache.json. We can use our own router engine or use Distance Matrix third services for calculating the actual values, they do not account for traffic. More details in PROD_SPEC.MD