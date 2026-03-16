import networkx as nx  # Graph logic: routing, shortest paths on road networks
import osmnx as ox  # Download real road networks from OpenStreetMap
import os  # File and path handling (check/save/load files)
import pickle  # Save/load Python objects (cache graphs, data)
import math  # Geometry and numeric calculations (angles, distances)
from functools import lru_cache  # Cache function results to avoid recomputation
from shapely.geometry import Point, LineString  # Represent locations as map points
from shapely.ops import nearest_points  # Find closest geometry (e.g., spot → road)
from itertools import islice

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


def initialize_graph():  # запуск карты
    global G, G_TURN
    if G is not None:
        return  # Если карта уже есть -> ничего не делаем.
    if os.path.exists(GRAPH_FILENAME):
        G = ox.load_graphml(GRAPH_FILENAME)
    else:
        G = ox.graph_from_place(PLACE_NAME, network_type="drive")
        ox.save_graphml(G, GRAPH_FILENAME)

    # Add travel times if missing
    G = ox.add_edge_speeds(G)
    G = ox.add_edge_travel_times(G)
    G = ox.add_edge_bearings(G)
    # ---- Add travel_time_new ----
    for u, v, k, data in G.edges(keys=True, data=True):
        base = data.get("travel_time", 1.0)
        name = data.get("name")

        multiplier = 1.0

        if name:
            if isinstance(name, list):
                names = name
            else:
                names = [name]

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
            )
            ###### CHANGE 1 ##########

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


# def get_route(u_coords, v_coords, custom_G=None):
#     target_G = custom_G if custom_G else G

#     # --- Resolve u ---
#     if isinstance(u_coords, str):
#         u_node = u_coords
#     else:
#         u_node = ox.nearest_nodes(target_G, u_coords[1], u_coords[0])

#     # --- Resolve v ---
#     if isinstance(v_coords, str):
#         v_node = v_coords
#     else:
#         v_node = ox.nearest_nodes(target_G, v_coords[1], v_coords[0])

#     if isinstance(u_node, str) and u_node not in target_G.nodes:
#         raise KeyError(f"u_node {u_node} not found in graph")

#     if isinstance(v_node, str) and v_node not in target_G.nodes:
#         raise KeyError(f"v_node {v_node} not found in graph")

#     try:
#         path = nx.shortest_path(
#             target_G, u_node, v_node, weight="travel_time_new"
#         )  ##CHANGED from travel_time

#         path_coords = [[target_G.nodes[n]["y"], target_G.nodes[n]["x"]] for n in path]

#         time = 0.0
#         for i in range(len(path) - 1):
#             edge_data = target_G.get_edge_data(path[i], path[i + 1])
#             if isinstance(edge_data, dict):
#                 edge_data = list(edge_data.values())[0]
#             time += edge_data.get("travel_time_new", 1.0)  ##CHANGED from travel_time

#         # --- Turn penalties ---
#         turn_penalty = 0.0
#         path_without_spots = [n for n in path if not isinstance(n, str)]
#         for i in range(1, len(path_without_spots) - 1):
#             n1, n2, n3 = (
#                 path_without_spots[i - 1],
#                 path_without_spots[i],
#                 path_without_spots[i + 1],
#             )

#             edge1 = G.get_edge_data(n1, n2)
#             edge2 = G.get_edge_data(n2, n3)

#             if edge1 is None or edge2 is None:
#                 continue

#             if isinstance(edge1, dict):
#                 edge1 = list(edge1.values())[0]
#             if isinstance(edge2, dict):
#                 edge2 = list(edge2.values())[0]

#             b1 = edge1.get("bearing", 0)
#             b2 = edge2.get("bearing", 0)
#             angle = (b2 - b1 + 180) % 360 - 180
#             if abs(angle) < 20:
#                 turn_penalty += COST_STRAIGHT
#             elif abs(angle) > 150:
#                 turn_penalty += COST_U_TURN
#             elif angle > 0:
#                 turn_penalty += COST_RIGHT
#             else:
#                 turn_penalty += COST_LEFT

#         return {
#             "path": path_coords,
#             "travel_time": time + turn_penalty,
#             "nodes": path,
#         }  ### added nodes key and value
#     except Exception:
#         return {"path": [], "travel_time": float("inf"), "nodes": []}


def get_route(u_coords, v_coords, custom_G=None):
    target_G = custom_G if custom_G else G

    # --- Resolve nodes ---
    u_node = (
        u_coords
        if isinstance(u_coords, str)
        else ox.nearest_nodes(target_G, u_coords[1], u_coords[0])
    )
    v_node = (
        v_coords
        if isinstance(v_coords, str)
        else ox.nearest_nodes(target_G, v_coords[1], v_coords[0])
    )

    if isinstance(u_node, str) and u_node not in target_G.nodes:
        raise KeyError(f"u_node {u_node} not found")
    if isinstance(v_node, str) and v_node not in target_G.nodes:
        raise KeyError(f"v_node {v_node} not found")

    try:
        path = nx.shortest_path(target_G, u_node, v_node, weight="travel_time_new")

        path_coords = [[target_G.nodes[n]["y"], target_G.nodes[n]["x"]] for n in path]

        # --- Edge travel time ---
        time = 0.0
        for i in range(len(path) - 1):
            edge_data = target_G.get_edge_data(path[i], path[i + 1])
            if isinstance(edge_data, dict):
                edge_data = list(edge_data.values())[0]
            time += edge_data.get("travel_time_new", 1.0)

        # --- Turn + Intersection penalties ---
        turn_penalty = 0.0
        path_without_spots = [n for n in path if not isinstance(n, str)]

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

            # ---- Turn type ----
            b1 = edge1.get("bearing", 0)
            b2 = edge2.get("bearing", 0)
            angle = (b2 - b1 + 180) % 360 - 180

            if abs(angle) < 20:
                turn_type = "straight"
                base_turn = COST_STRAIGHT
            elif abs(angle) > 150:
                turn_type = "uturn"
                base_turn = COST_U_TURN
            elif angle > 0:
                turn_type = "right"
                base_turn = COST_RIGHT
            else:
                turn_type = "left"
                base_turn = COST_LEFT

            # ---- Detect hard intersection ----
            names1 = edge1.get("name")
            names2 = edge2.get("name")

            if not names1 or not names2:
                is_hard = False
            else:
                if not isinstance(names1, list):
                    names1 = [names1]
                if not isinstance(names2, list):
                    names2 = [names2]

                is_hard = any(
                    (a, b) in HARD_INTERSECTIONS or (b, a) in HARD_INTERSECTIONS
                    for a in names1
                    for b in names2
                )

            # ---- Detect traffic signal ----
            has_signal = G.nodes[n2].get("highway") == "traffic_signals"

            # ---- Signal penalty depends on turn type ----
            signal_penalty = 0
            if has_signal:
                if turn_type == "right":
                    signal_penalty = 5
                elif turn_type == "straight":
                    signal_penalty = 10
                elif turn_type == "left":
                    signal_penalty = 20
                elif turn_type == "uturn":
                    signal_penalty = 25

            # ---- Hard intersection amplifies delay ----
            extra_penalty = 0

            if is_hard:
                if has_signal:
                    signal_penalty *= 2.0
                else:
                    penalties = {
                        "right": 5,
                        "straight": 10,
                        "left": 20,
                        "uturn": 40,
                    }
                    extra_penalty = penalties[turn_type]

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


def count_crossings(spot_coords, dest_coords, G, ignore_edge=None):
    walk_line = LineString(
        [
            (spot_coords[1], spot_coords[0]),
            (dest_coords[1], dest_coords[0]),
        ]
    )

    crossed = set()

    for u, v, k, data in G.edges(keys=True, data=True):
        if ignore_edge is not None and (u, v, k) == ignore_edge:
            continue

        geom = data.get("geometry")
        if geom is None:
            x1, y1 = G.nodes[u]["x"], G.nodes[u]["y"]
            x2, y2 = G.nodes[v]["x"], G.nodes[v]["y"]
            geom = LineString([(x1, y1), (x2, y2)])

        if not walk_line.crosses(geom):
            continue

        highway = data.get("highway", "")
        if highway in ["service", "parking_aisle", "driveway"]:
            continue

        # count only crossings that are near traffic signals
        has_signal = (
            G.nodes[u].get("highway") == "traffic_signals"
            or G.nodes[v].get("highway") == "traffic_signals"
        )

        if has_signal:
            crossed.add((u, v, k))

    return len(crossed)


# def evaluate_path_with_penalties(path, target_G):
#     base_time = 0.0

#     for i in range(len(path) - 1):
#         edge_data = target_G.get_edge_data(path[i], path[i + 1])
#         if isinstance(edge_data, dict):
#             edge_data = list(edge_data.values())[0]
#         base_time += edge_data.get("travel_time_new", 1.0)

#     turn_penalty = 0.0
#     path_clean = [n for n in path if not isinstance(n, str)]

#     for i in range(1, len(path_clean) - 1):
#         n1, n2, n3 = path_clean[i - 1], path_clean[i], path_clean[i + 1]

#         edge1 = target_G.get_edge_data(n1, n2)
#         edge2 = target_G.get_edge_data(n2, n3)

#         if not edge1 or not edge2:
#             continue

#         if isinstance(edge1, dict):
#             edge1 = list(edge1.values())[0]
#         if isinstance(edge2, dict):
#             edge2 = list(edge2.values())[0]

#         b1 = edge1.get("bearing", 0)
#         b2 = edge2.get("bearing", 0)
#         angle = (b2 - b1 + 180) % 360 - 180

#         if abs(angle) < 20:
#             turn_type = "straight"
#             base_turn = COST_STRAIGHT
#         elif abs(angle) > 150:
#             turn_type = "uturn"
#             base_turn = COST_U_TURN
#         elif angle > 0:
#             turn_type = "right"
#             base_turn = COST_RIGHT
#         else:
#             turn_type = "left"
#             base_turn = COST_LEFT

#         has_signal = target_G.nodes[n2].get("highway") == "traffic_signals"

#         signal_penalty = 0
#         if has_signal:
#             if turn_type == "right":
#                 signal_penalty = 5
#             elif turn_type == "straight":
#                 signal_penalty = 20
#             elif turn_type == "left":
#                 signal_penalty = 30
#             elif turn_type == "uturn":
#                 signal_penalty = 35

#         turn_penalty += base_turn + signal_penalty

#     return {
#         "nodes": path,
#         "path": [[target_G.nodes[n]["y"], target_G.nodes[n]["x"]] for n in path],
#         "base_time": base_time,
#         "penalty_time": turn_penalty,
#         "total_time": base_time + turn_penalty,
#     }


# def get_k_best_routes(u_coords, v_coords, custom_G=None, k_generate=10, k_return=5):

#     target_G = custom_G if custom_G else G

#     u_node = (
#         u_coords
#         if isinstance(u_coords, str)
#         else ox.nearest_nodes(target_G, u_coords[1], u_coords[0])
#     )

#     v_node = (
#         v_coords
#         if isinstance(v_coords, str)
#         else ox.nearest_nodes(target_G, v_coords[1], v_coords[0])
#     )

#     try:
#         generator = nx.shortest_simple_paths(
#             target_G, u_node, v_node, weight="travel_time_new"
#         )

#         candidates = list(islice(generator, k_generate))

#         results = []

#         for path in candidates:
#             res = evaluate_path_with_penalties(path, target_G)
#             results.append(res)

#         results.sort(key=lambda x: x["total_time"])

#         return results[:k_return]

#     except:
#         return []
