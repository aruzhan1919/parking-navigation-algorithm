import networkx as nx  # Graph logic: routing, shortest paths on road networks
import osmnx as ox  # Download real road networks from OpenStreetMap
import os  # File and path handling (check/save/load files)
import pickle  # Save/load Python objects (cache graphs, data)
import math  # Geometry and numeric calculations (angles, distances)
from functools import lru_cache  # Cache function results to avoid recomputation
from shapely.geometry import Point, LineString  # Represent locations as map points
from shapely.ops import nearest_points  # Find closest geometry (e.g., spot → road)

# ==========================================
# CONFIGURATION
# ==========================================
PLACE_NAME = "Astana, Kazakhstan"
GRAPH_FILENAME = "astana_drive.graphml"

# PENALTIES (Seconds)
# COST_U_TURN = 40.0
# COST_LEFT = 25.0
# COST_RIGHT = 10.0
# COST_STRAIGHT = 0.0
COST_U_TURN = 0.0
COST_LEFT = 0.0
COST_RIGHT = 0.0
COST_STRAIGHT = 0.0

G = None
G_TURN = None


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
        total_time = edge_data["travel_time"]

        # Calculate distance along edge for each spot
        spot_offsets = []
        for s in spot_list:
            p = Point(s["coords"][1], s["coords"][0])
            ########### CHANGE ##########
            s["_bearing"] = bearing
            # if edge_geom:
            #     dist_along = edge_geom.project(p)
            # else:
            #     # Fallback to simple linear interp if no geometry
            #     dist_along = 0  # Simplified
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
            # G_aug.add_node(new_node, x=coords[1], y=coords[0])

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
                travel_time=seg_time,
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
            travel_time=total_time * ratio,
            highway=edge_data.get("highway"),
        )

        # Remove the original long edge
        G_aug.remove_edge(u, v, k)

    return G_aug


# def _bearing(lat1, lon1, lat2, lon2):
#     dLon = math.radians(lon2 - lon1)
#     lat1 = math.radians(lat1)
#     lat2 = math.radians(lat2)

#     y = math.sin(dLon) * math.cos(lat2)
#     x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(
#         dLon
#     )

#     brng = math.degrees(math.atan2(y, x))
#     return (brng + 360) % 360


def get_route(u_coords, v_coords, custom_G=None):
    target_G = custom_G if custom_G else G

    # --- Resolve u ---
    if isinstance(u_coords, str):
        u_node = u_coords
    else:
        u_node = ox.nearest_nodes(target_G, u_coords[1], u_coords[0])

    # --- Resolve v ---
    if isinstance(v_coords, str):
        v_node = v_coords
    else:
        v_node = ox.nearest_nodes(target_G, v_coords[1], v_coords[0])

    if isinstance(u_node, str) and u_node not in target_G.nodes:
        raise KeyError(f"u_node {u_node} not found in graph")

    if isinstance(v_node, str) and v_node not in target_G.nodes:
        raise KeyError(f"v_node {v_node} not found in graph")

    try:
        path = nx.shortest_path(target_G, u_node, v_node, weight="travel_time")

        path_coords = [[target_G.nodes[n]["y"], target_G.nodes[n]["x"]] for n in path]

        time = 0.0
        for i in range(len(path) - 1):
            edge_data = target_G.get_edge_data(path[i], path[i + 1])
            if isinstance(edge_data, dict):
                edge_data = list(edge_data.values())[0]
            time += edge_data.get("travel_time", 1.0)
        # --- Turn penalties ---
        turn_penalty = 0.0

        # for i in range(1, len(path) - 1):
        #     n1, n2, n3 = path[i - 1], path[i], path[i + 1]

        #     lat1, lon1 = target_G.nodes[n1]["y"], target_G.nodes[n1]["x"]
        #     lat2, lon2 = target_G.nodes[n2]["y"], target_G.nodes[n2]["x"]
        #     lat3, lon3 = target_G.nodes[n3]["y"], target_G.nodes[n3]["x"]

        #     b1 = _bearing(lat1, lon1, lat2, lon2)
        #     b2 = _bearing(lat2, lon2, lat3, lon3)

        #     angle = (b2 - b1 + 180) % 360 - 180

        #     if abs(angle) < 20:
        #         turn_penalty += COST_STRAIGHT
        #     elif abs(angle) > 150:
        #         turn_penalty += COST_U_TURN
        #     elif angle > 0:
        #         turn_penalty += COST_RIGHT
        #     else:
        #         turn_penalty += COST_LEFT
        path_without_spots = [n for n in path if not isinstance(n, str)]
        for i in range(1, len(path_without_spots) - 1):
            n1, n2, n3 = (
                path_without_spots[i - 1],
                path_without_spots[i],
                path_without_spots[i + 1],
            )

            edge1 = G.get_edge_data(n1, n2)
            edge2 = G.get_edge_data(n2, n3)

            if edge1 is None or edge2 is None:
                continue

            if isinstance(edge1, dict):
                edge1 = list(edge1.values())[0]
            if isinstance(edge2, dict):
                edge2 = list(edge2.values())[0]

            b1 = edge1.get("bearing", 0)
            b2 = edge2.get("bearing", 0)
            angle = (b2 - b1 + 180) % 360 - 180
            if abs(angle) < 20:
                turn_penalty += COST_STRAIGHT
            elif abs(angle) > 150:
                turn_penalty += COST_U_TURN
            elif angle > 0:
                turn_penalty += COST_RIGHT
            else:
                turn_penalty += COST_LEFT
        return {
            "path": path_coords,
            "travel_time": time + turn_penalty,
            "nodes": path,
        }  ### added nodes key and value
    except Exception:
        return {"path": [], "travel_time": float("inf"), "nodes": []}
