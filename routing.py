"""
routing.py
==========
Road graph loading, spot injection (Split-Edge), and route calculation.

Key responsibilities:
1. Load OSM road graph for Astana
2. Inject parking spots into graph edges (Split-Edge technique)
3. Calculate routes with turn penalties and intersection costs
4. Count pedestrian street crossings for walk penalty
5. Build primary + alternative routes for UI display

Dependencies: osmnx, networkx, shapely, geopy, requests
"""

import math
import os
from collections import defaultdict

import networkx as nx
import osmnx as ox
import requests
from geopy.distance import geodesic
from shapely.geometry import LineString, Point

# ==========================================
# CONFIGURATION
# ==========================================

PLACE_NAME = "Astana, Kazakhstan"
GRAPH_FILENAME = "astana_drive.graphml"

# PENALTIES (Seconds)
COST_U_TURN = 20.0
COST_LEFT = 10.0
COST_RIGHT = 5.0
COST_STRAIGHT = 0.0
# COST_U_TURN = 0.0
# COST_LEFT = 0.0
# COST_RIGHT = 0.0
# COST_STRAIGHT = 0.0

G = None
G_TURN = None

BUSY_STREETS_1 = [
    # "Мәңгілік Ел даңғылы",
    # "Тұран даңғылы",
    # "Қабанбай Батыр даңғылы",
    # "Түркістан көшесі",
]

BUSY_STREETS_2 = [
    # "Сарайшық көшесі",
    # "Қайым Мұхамедханов көшесі",
    # "Мухамедханова",
    # "Сығанақ көшесі",
    # "проспект Республики",
]

BUSY_STREETS_3 = [
    # "улица Шокана Валиханова",
    # "Тәуелсіздік даңғылы",
    # "Арай көшесі",
    # "Шәкәрім Құдайбердіұлы даңғылы",
]

HARD_INTERSECTIONS = [
    ("Тұран даңғылы", "Коргалжын шоссе"),
    ("Достық көшесі", "Ақмешіт көшесі"),
    ("Тұран даңғылы", "Сарайшық көшесі"),
    ("Достық көшесі", "Түркістан көшесі"),
    ("Сарайшық көшесі", "Ақмешіт көшесі"),
    ("Қабанбай Батыр даңғылы", "Сарайшық көшесі"),
    ("Қабанбай Батыр даңғылы", "Қонаев көшесі"),
    ("Қабанбай Батыр даңғылы", "Достық көшесі"),
    ("Достық көшесі", "Сауран көшесі"),
    ("Қонаев көшесі", "Түркістан көшесі"),
    ("Қабанбай Батыр даңғылы", "Коргалжын шоссе"),
    ("Тәуелсіздік даңғылы", "Бауыржан Момышұлы даңғылы"),
    ("улица Кенесары", "проспект Женис"),
    ("проспект Республики", "улица Сейфуллина"),
    ("проспект Республики", "улица Сакена Сейфуллина"),
    ("улица Шокана Валиханова", "улица Сейфуллина"),
    ("улица Шокана Валиханова", "улица Сакена Сейфуллина"),
]

# ==========================================
# GRAPH INITIALIZATION
# ==========================================


def initialize_graph():
    """
    Load the Astana road graph from disk or download from OSM.

    - Adds edge speeds, travel times, and bearings via osmnx
    - Adds travel_time_new = travel_time * traffic_multiplier per edge
    - Singleton: does nothing if G is already loaded

    NOTE: Currently always downloads fresh from OSM (disk cache disabled).
    To enable disk cache, uncomment the os.path.exists block below.
    """
    global G, G_TURN
    if G is not None:
        return

    # Load from disk if available (faster), otherwise download from OSM
    if os.path.exists(GRAPH_FILENAME):
        G = ox.load_graphml(GRAPH_FILENAME)
    else:
        G = ox.graph_from_place(PLACE_NAME, network_type="drive")
        G = ox.add_edge_speeds(G)
        G = ox.add_edge_travel_times(G)
        G = ox.add_edge_bearings(G)
        ox.save_graphml(G, GRAPH_FILENAME)

    # Apply traffic multipliers per street name
    for u, v, k, data in G.edges(keys=True, data=True):
        base = data.get("travel_time", 1.0)
        name = data.get("name")
        multiplier = 1.0

        if name:
            names = name if isinstance(name, list) else [name]
            if any(n in BUSY_STREETS_1 for n in names):
                multiplier = 1.4
            elif any(n in BUSY_STREETS_2 for n in names):
                multiplier = 1.3
            elif any(n in BUSY_STREETS_3 for n in names):
                multiplier = 1.2

        data["travel_time_new"] = base * multiplier


def augment_graph_with_spots(spots):
    """
    Realizes the 'Split-Edge' theory.
    Transforms the graph by injecting spot-nodes into existing edges.
    """
    global G
    initialize_graph()  # убеждаемся, что карта дорог загружена

    # Work on a copy to keep the base graph clean
    G_aug = G.copy()

    # 1. Find nearest edges for all spots
    lats = [s["coords"][0] for s in spots]
    lngs = [s["coords"][1] for s in spots]
    nearest_edges = ox.nearest_edges(
        G_aug, lngs, lats
    )  # На какой именно улице она лежит ближе всего?

    ########## new #########
    # u, v, k = nearest_edges
    # print(u, v, k)
    # print(G.get_edge_data(u, v, k))
    ########## new #########
    # 2. Group spots by the edge they belong to
    edge_to_spots = {}
    for i, edge in enumerate(nearest_edges):
        ######### CHANGE ##########
        spots[i]["_edge"] = edge
        if edge not in edge_to_spots:
            edge_to_spots[edge] = []
        edge_to_spots[edge].append(spots[i])

    for (u, v, k), spot_list in edge_to_spots.items():
        edge_data = G_aug.get_edge_data(u, v, k)
        if not edge_data:
            continue
        ########### CHANGE ################
        bearing = edge_data.get("bearing", 0)

        edge_geom = edge_data.get("geometry", None)  ### we have None values too
        total_len = edge_data["length"]
        total_time = edge_data["travel_time_new"]  # CHANGE from travel_time

        # Calculate distance along edge for each spot
        spot_offsets = []
        for s in spot_list:
            p = Point(s["coords"][1], s["coords"][0])
            ########### CHANGE ##########
            s["_bearing"] = bearing
            ##### CHANGE ###############
            if edge_geom is None:
                # build straight-line geometry from u to v
                u_x, u_y = G_aug.nodes[u]["x"], G_aug.nodes[u]["y"]
                v_x, v_y = G_aug.nodes[v]["x"], G_aug.nodes[v]["y"]
                edge_geom = LineString([(u_x, u_y), (v_x, v_y)])
            dist_along = edge_geom.project(p)
            ##### CHANGE ###############
            spot_offsets.append((s["id"], dist_along, s["coords"]))

        # Sort spots by distance along the edge
        spot_offsets.sort(key=lambda x: x[1])

        # 3. Split the edge
        curr_u = u
        last_offset = 0

        for spot_id, offset, coords in spot_offsets:
            new_node = f"spot_{spot_id}"
            ###### CHANGE 1 ##########
            proj_point = edge_geom.interpolate(offset)

            G_aug.add_node(
                new_node,
                x=proj_point.x,
                y=proj_point.y,
                is_manual=True,  # new
            )
            ###### CHANGE 1 ##########
            # store the real graph node for this spot
            for s in spot_list:
                if s["id"] == spot_id:
                    s["_node_id"] = new_node
                    s["_proj_coords"] = (proj_point.y, proj_point.x)
                    break
            ####### change ############
            # Calculate segment metrics
            seg_len = offset - last_offset
            ratio = seg_len / total_len if total_len > 0 else 0
            seg_time = total_time * ratio

            # Add the segment from current position to this spot
            G_aug.add_edge(
                curr_u,
                new_node,
                length=seg_len,
                travel_time_new=seg_time,  # CHANGE from travel_time
                highway=edge_data.get("highway"),
            )

            curr_u = new_node
            last_offset = offset

        # Final segment to the original end intersection
        final_len = total_len - last_offset
        ratio = final_len / total_len if total_len > 0 else 0
        G_aug.add_edge(
            curr_u,
            v,
            length=final_len,
            travel_time_new=total_time * ratio,  # CHANGE from travel_time
            highway=edge_data.get("highway"),
        )

        # Remove the original long edge
        G_aug.remove_edge(u, v, k)

    return G_aug


# ==========================================
# ROUTE CALCULATION
# ==========================================


def _resolve_to_node(ref, target_G):
    """
    Resolve a route endpoint to a graph node id.

    Accepts:
        - str: used directly as node id
        - dict with '_node_id': spot node id
        - dict with 'coords': find nearest node
        - tuple (lat, lon): find nearest node
    """
    if isinstance(ref, str):
        return ref
    if isinstance(ref, dict):
        if "_node_id" in ref:
            return ref["_node_id"]
        if "coords" in ref:
            return ox.nearest_nodes(target_G, ref["coords"][1], ref["coords"][0])
    return ox.nearest_nodes(target_G, ref[1], ref[0])


# def get_route(u_coords, v_coords, custom_G=None):
#     target_G = custom_G if custom_G else G

#     # --- Resolve nodes ---
#     # u_node = (
#     #     u_coords
#     #     if isinstance(u_coords, str)
#     #     else ox.nearest_nodes(target_G, u_coords[1], u_coords[0])
#     # )
#     # v_node = (
#     #     v_coords
#     #     if isinstance(v_coords, str)
#     #     else ox.nearest_nodes(target_G, v_coords[1], v_coords[0])
#     # )
#     u_node = _resolve_to_node(u_coords, target_G)
#     v_node = _resolve_to_node(v_coords, target_G)
#     if isinstance(u_node, str) and u_node not in target_G.nodes:
#         raise KeyError(f"u_node {u_node} not found")
#     if isinstance(v_node, str) and v_node not in target_G.nodes:
#         raise KeyError(f"v_node {v_node} not found")

#     try:
#         path = nx.shortest_path(target_G, u_node, v_node, weight="travel_time_new")

#         path_coords = [[target_G.nodes[n]["y"], target_G.nodes[n]["x"]] for n in path]

#         # --- Edge travel time ---
#         time = 0.0
#         for i in range(len(path) - 1):
#             edge_data = target_G.get_edge_data(path[i], path[i + 1])
#             if isinstance(edge_data, dict):
#                 edge_data = list(edge_data.values())[0]
#                 time += edge_data.get("travel_time_new", 1.0)

#         # --- Turn + Intersection penalties ---
#         turn_penalty = 0.0
#         # path_without_spots = [n for n in path if not isinstance(n, str)]
#         path_without_spots = [
#             n for n in path if not target_G.nodes[n].get("is_manual", False)
#         ]

#         for i in range(1, len(path_without_spots) - 1):
#             n1, n2, n3 = (
#                 path_without_spots[i - 1],
#                 path_without_spots[i],
#                 path_without_spots[i + 1],
#             )

#             edge1 = G.get_edge_data(n1, n2)
#             edge2 = G.get_edge_data(n2, n3)

#             if not edge1 or not edge2:
#                 continue

#             if isinstance(edge1, dict):
#                 edge1 = list(edge1.values())[0]
#             if isinstance(edge2, dict):
#                 edge2 = list(edge2.values())[0]

#             # ---- Turn type ----
#             b1 = edge1.get("bearing", 0)
#             b2 = edge2.get("bearing", 0)
#             angle = (b2 - b1 + 180) % 360 - 180

#             if abs(angle) < 20:
#                 turn_type = "straight"
#                 base_turn = COST_STRAIGHT
#             elif abs(angle) > 150:
#                 turn_type = "uturn"
#                 base_turn = COST_U_TURN
#             elif angle > 0:
#                 turn_type = "right"
#                 base_turn = COST_RIGHT
#             else:
#                 turn_type = "left"
#                 base_turn = COST_LEFT

#             # ---- Detect hard intersection ----
#             names1 = edge1.get("name")
#             names2 = edge2.get("name")

#             if not names1 or not names2:
#                 is_hard = False
#             else:
#                 if not isinstance(names1, list):
#                     names1 = [names1]
#                 if not isinstance(names2, list):
#                     names2 = [names2]

#                 is_hard = any(
#                     (a, b) in HARD_INTERSECTIONS or (b, a) in HARD_INTERSECTIONS
#                     for a in names1
#                     for b in names2
#                 )

#             # ---- Detect traffic signal ----
#             has_signal = G.nodes[n2].get("highway") == "traffic_signals"

#             # ---- Signal penalty depends on turn type ----
#             signal_penalty = 0
#             if has_signal:
#                 if turn_type == "right":
#                     signal_penalty = 5
#                 elif turn_type == "straight":
#                     signal_penalty = 10
#                 elif turn_type == "left":
#                     signal_penalty = 20
#                 elif turn_type == "uturn":
#                     signal_penalty = 25

#             # ---- Hard intersection amplifies delay ----
#             extra_penalty = 0

#             if is_hard:
#                 if has_signal:
#                     signal_penalty *= 2.0
#                 else:
#                     penalties = {
#                         "right": 5,
#                         "straight": 10,
#                         "left": 20,
#                         "uturn": 40,
#                     }
#                     extra_penalty = penalties[turn_type]

#             turn_penalty += base_turn + signal_penalty + extra_penalty

#         return {
#             "path": path_coords,
#             "travel_time": time + turn_penalty,
#             "turn_penalty": turn_penalty,
#             "nodes": path,
#         }


#     except Exception:
#         return {
#             "path": [],
#             "travel_time": float("inf"),
#             "turn_penalty": float("inf"),
#             "nodes": [],
#         }
def get_route(u_coords, v_coords, custom_G=None):
    """
    Compute shortest path between two points using Dijkstra on travel_time_new.
    Also computes turn penalties for the path.

    Args:
        u_coords: start — (lat, lon), spot dict, or node id string
        v_coords: end — same formats
        custom_G: use this graph instead of global G (e.g. G_aug with spots)

    Returns:
        dict:
            path: list of [lat, lon] coordinates for map display
            travel_time: float (seconds) — edge time + turn penalties
            turn_penalty: float (seconds) — turn penalties only
            nodes: list of graph node ids along the path
    """
    target_G = custom_G if custom_G else G

    u_node = _resolve_to_node(u_coords, target_G)
    v_node = _resolve_to_node(v_coords, target_G)

    if isinstance(u_node, str) and u_node not in target_G.nodes:
        raise KeyError(f"u_node {u_node} not found in graph")
    if isinstance(v_node, str) and v_node not in target_G.nodes:
        raise KeyError(f"v_node {v_node} not found in graph")

    try:
        path = nx.shortest_path(target_G, u_node, v_node, weight="travel_time_new")

        # Build path coordinates and sum edge travel times
        path_coords = []
        time = 0.0

        for i in range(len(path) - 1):
            a, b = path[i], path[i + 1]
            edge_dict = target_G.get_edge_data(a, b)
            if not edge_dict:
                continue

            # MultiDiGraph: pick edge with minimum travel_time_new
            if isinstance(edge_dict, dict) and all(
                isinstance(v, dict) for v in edge_dict.values()
            ):
                edge_data = min(
                    edge_dict.values(),
                    key=lambda d: d.get("travel_time_new", float("inf")),
                )
            else:
                edge_data = edge_dict

            time += edge_data.get("travel_time_new", 1.0)

            geom = edge_data.get("geometry")
            if geom is not None:
                coords = [[lat, lon] for lon, lat in geom.coords]
            else:
                coords = [
                    [target_G.nodes[a]["y"], target_G.nodes[a]["x"]],
                    [target_G.nodes[b]["y"], target_G.nodes[b]["x"]],
                ]

            path_coords.extend(coords[1:] if path_coords else coords)

        # Compute turn penalties (skip spot nodes — they have no bearing data)
        turn_penalty = 0.0
        path_without_spots = [
            n for n in path if not target_G.nodes[n].get("is_manual", False)
        ]

        for i in range(1, len(path_without_spots) - 1):
            n1, n2, n3 = (
                path_without_spots[i - 1],
                path_without_spots[i],
                path_without_spots[i + 1],
            )

            edge1 = G.get_edge_data(n1, n2)
            edge2 = G.get_edge_data(n2, n3)
            if not edge1 or not edge2:
                continue

            if isinstance(edge1, dict):
                edge1 = list(edge1.values())[0]
            if isinstance(edge2, dict):
                edge2 = list(edge2.values())[0]

            # Normalize bearing difference to [-180, +180]
            b1 = edge1.get("bearing", 0)
            b2 = edge2.get("bearing", 0)
            angle = (b2 - b1 + 180) % 360 - 180

            if abs(angle) < 20:
                turn_type, base_turn = "straight", COST_STRAIGHT
            elif abs(angle) > 150:
                turn_type, base_turn = "uturn", COST_U_TURN
            elif angle > 0:
                turn_type, base_turn = "right", COST_RIGHT
            else:
                turn_type, base_turn = "left", COST_LEFT

            # Check for hard intersection
            names1 = edge1.get("name")
            names2 = edge2.get("name")
            is_hard = False
            if names1 and names2:
                names1 = names1 if isinstance(names1, list) else [names1]
                names2 = names2 if isinstance(names2, list) else [names2]
                is_hard = any(
                    (a, b) in HARD_INTERSECTIONS or (b, a) in HARD_INTERSECTIONS
                    for a in names1
                    for b in names2
                )

            # Traffic signal penalty
            has_signal = G.nodes[n2].get("highway") == "traffic_signals"
            signal_penalty = 0
            if has_signal:
                signal_penalty = {"right": 5, "straight": 10, "left": 20, "uturn": 25}[
                    turn_type
                ]

            # Hard intersection doubles signal penalty or adds fixed penalty
            extra_penalty = 0
            if is_hard:
                if has_signal:
                    signal_penalty *= 2.0
                else:
                    extra_penalty = {
                        "right": 5,
                        "straight": 10,
                        "left": 20,
                        "uturn": 40,
                    }[turn_type]

            turn_penalty += base_turn + signal_penalty + extra_penalty

        return {
            "path": path_coords,
            "travel_time": time + turn_penalty,
            "turn_penalty": turn_penalty,
            "nodes": path,
        }

    except Exception:
        return {
            "path": [],
            "travel_time": float("inf"),
            "turn_penalty": float("inf"),
            "nodes": [],
        }


# ==========================================
# PEDESTRIAN CROSSING COUNT
# ==========================================


def normalize_street_name(name):
    """Normalize OSM street name to lowercase string for reliable comparison."""
    if name is None:
        return None
    if isinstance(name, list):
        name = " | ".join(sorted(str(x).strip().lower() for x in name if x))
    else:
        name = str(name).strip().lower()
    return name or None


def _point_side_relative_to_edge(point_latlon, G, edge):
    """
    Returns signed cross product indicating which side of the edge the point is on.
    > 0: left of edge (u→v direction)
    < 0: right of edge
    = 0: on the edge line
    """
    u, v, k = edge
    x1, y1 = G.nodes[u]["x"], G.nodes[u]["y"]
    x2, y2 = G.nodes[v]["x"], G.nodes[v]["y"]
    px, py = point_latlon[1], point_latlon[0]
    return (x2 - x1) * (py - y1) - (y2 - y1) * (px - x1)


def count_crossings(spot_coords, dest_coords, G, ignore_edge=None, eps=1e-10):
    """
    Count the number of unique named streets a pedestrian must cross
    walking in a straight line from spot to destination.

    Args:
        spot_coords: (lat, lon) of parking spot
        dest_coords: (lat, lon) of destination
        G: base road graph (not G_aug)
        ignore_edge: (u, v, k) — the spot's own edge, handled specially
        eps: tolerance for side determination

    Returns:
        int: number of unique street crossings
    """
    walk_line = LineString(
        [
            (spot_coords[1], spot_coords[0]),
            (dest_coords[1], dest_coords[0]),
        ]
    )

    crossed_streets = set()
    spot_street_name = None
    opposite_sides = False

    # Step 1: Check if spot and dest are on opposite sides of spot's own street
    if ignore_edge is not None and ignore_edge in G.edges:
        spot_data = G.edges[ignore_edge]
        spot_street_name = normalize_street_name(spot_data.get("name"))
        spot_sign = _point_side_relative_to_edge(spot_coords, G, ignore_edge)
        dest_sign = _point_side_relative_to_edge(dest_coords, G, ignore_edge)
        if abs(spot_sign) > eps and abs(dest_sign) > eps and spot_sign * dest_sign < 0:
            opposite_sides = True

    # Step 2: Find all named streets the walk line crosses
    for u, v, k, data in G.edges(keys=True, data=True):
        geom = data.get("geometry")
        if geom is None:
            x1, y1 = G.nodes[u]["x"], G.nodes[u]["y"]
            x2, y2 = G.nodes[v]["x"], G.nodes[v]["y"]
            geom = LineString([(x1, y1), (x2, y2)])

        if not walk_line.crosses(geom):
            continue

        highway = data.get("highway", "")
        if isinstance(highway, list):
            highway = highway[0] if highway else ""

        # Skip non-pedestrian-relevant road types
        if highway in {"service", "parking_aisle", "driveway"}:
            continue

        crossed_name = normalize_street_name(data.get("name"))
        if crossed_name is None:
            continue

        # Don't count spot's own street if on same side
        if not opposite_sides and spot_street_name is not None:
            if crossed_name == spot_street_name:
                continue

        crossed_streets.add(crossed_name)

    return len(crossed_streets)


# ==========================================
# ALTERNATIVE ROUTE GENERATION
# ==========================================


def _get_edge_data_for_path_pair(target_G, u, v):
    """Get first edge data dict for edge (u, v) in a MultiDiGraph."""
    edge_data = target_G.get_edge_data(u, v)
    if not edge_data:
        return None
    if isinstance(edge_data, dict):
        first_val = list(edge_data.values())[0]
        if isinstance(first_val, dict):
            return first_val
    return edge_data


def get_street_lengths_on_route(route_nodes, target_G=None):
    """
    Returns total length (meters) per street name along the route.

    Returns:
        dict: {normalized_street_name: total_length_meters}
    """
    target_G = target_G if target_G is not None else G
    street_lengths = defaultdict(float)

    if not route_nodes or len(route_nodes) < 2:
        return street_lengths

    for a, b in zip(route_nodes[:-1], route_nodes[1:]):
        data = _get_edge_data_for_path_pair(target_G, a, b)
        if not data:
            continue
        street_name = normalize_street_name(data.get("name"))
        if not street_name:
            continue
        street_lengths[street_name] += float(data.get("length", 0.0))

    return dict(street_lengths)


def get_main_street_from_route(route_nodes, target_G=None):
    """
    Returns the street with the longest total distance on the route.

    Returns:
        dict:
            street: str or None
            length: float (meters)
            all_streets: dict of all street lengths
    """
    target_G = target_G if target_G is not None else G
    street_lengths = get_street_lengths_on_route(route_nodes, target_G=target_G)

    if not street_lengths:
        return {"street": None, "length": 0.0, "all_streets": {}}

    main_street, total_len = max(street_lengths.items(), key=lambda x: x[1])
    return {"street": main_street, "length": total_len, "all_streets": street_lengths}


def build_penalized_graph(
    penalized_street=None, street_penalty_factor=5.0, custom_G=None
):
    """
    Returns a copy of the graph with travel_time_new multiplied by street_penalty_factor
    for all edges on penalized_street. Used to force alternative route generation.
    """
    target_G = custom_G if custom_G else G
    G_tmp = target_G.copy()

    if penalized_street is None:
        return G_tmp

    for u, v, k, data in G_tmp.edges(keys=True, data=True):
        if normalize_street_name(data.get("name")) == penalized_street:
            data["travel_time_new"] = (
                float(data.get("travel_time_new", 1.0)) * street_penalty_factor
            )

    return G_tmp


def get_primary_and_alternative_route(
    u_coords, v_coords, custom_G=None, street_penalty_factor=5.0
):
    """
    Compute primary route, then generate an alternative by penalizing the main street.

    Returns:
        dict:
            primary: route dict
            alternative: route dict (different street if possible)
            main_street: str — name of dominant street on primary route
    """
    target_G = custom_G if custom_G else G

    primary = get_route(u_coords, v_coords, custom_G=target_G)

    if not primary["nodes"]:
        return {
            "primary": primary,
            "alternative": {
                "path": [],
                "travel_time": float("inf"),
                "turn_penalty": float("inf"),
                "nodes": [],
            },
            "main_street": None,
        }

    main_info = get_main_street_from_route(primary["nodes"], target_G=target_G)
    main_street = main_info["street"]

    if main_street is None:
        return {"primary": primary, "alternative": primary, "main_street": None}

    G_penalized = build_penalized_graph(
        penalized_street=main_street,
        street_penalty_factor=street_penalty_factor,
        custom_G=target_G,
    )
    alternative = get_route(u_coords, v_coords, custom_G=G_penalized)

    return {"primary": primary, "alternative": alternative, "main_street": main_street}
