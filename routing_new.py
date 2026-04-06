"""
routing.py
==========
Road graph loading and route calculation.

PRIMARY: 2GIS Routing API (real traffic, accurate geometry)
FALLBACK: OSM/NetworkX graph (if 2GIS fails)

OSM graph is still loaded for:
- count_crossings (pedestrian street crossing detection)
- augment_graph_with_spots (split-edge, spot injection)
- fallback routing

get_route() now:
1. Tries 2GIS first
2. Falls back to OSM if 2GIS fails
3. Computes turn_penalty from polyline geometry
4. Returns same dict format as before
"""

import math
import os
from collections import defaultdict

import networkx as nx
import osmnx as ox
import requests
from geopy.distance import geodesic
from shapely.geometry import LineString, Point

from twogis_routing import get_route_2gis

# ==========================================
# CONFIGURATION
# ==========================================

PLACE_NAME = "Astana, Kazakhstan"
GRAPH_FILENAME = "astana_drive.graphml"

# Turn penalty constants (seconds) — applied to polyline-computed turns
COST_U_TURN = 20.0
COST_LEFT = 10.0
COST_RIGHT = 5.0
COST_STRAIGHT = 0.0

G = None  # base OSM graph
G_TURN = None


BUSY_STREETS_1 = []
BUSY_STREETS_2 = []
BUSY_STREETS_3 = []

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
# GRAPH INITIALIZATION (OSM — for fallback + crossings)
# ==========================================


def initialize_graph():
    """
    Load the Astana road graph from disk or download from OSM.
    Used for: crossings, spot augmentation, fallback routing.
    """
    global G, G_TURN
    if G is not None:
        return

    if os.path.exists(GRAPH_FILENAME):
        G = ox.load_graphml(GRAPH_FILENAME)
    else:
        G = ox.graph_from_place(PLACE_NAME, network_type="drive")
        G = ox.add_edge_speeds(G)
        G = ox.add_edge_travel_times(G)
        G = ox.add_edge_bearings(G)
        ox.save_graphml(G, GRAPH_FILENAME)

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


# ==========================================
# TURN PENALTY FROM POLYLINE
# ==========================================


def _compute_turn_penalty_from_path(path_coords):
    """
    Compute turn penalty from a list of [lat, lon] coordinates.

    Calculates bearing change at each intermediate point and
    classifies each turn as straight / right / left / u-turn.

    Args:
        path_coords: [[lat, lon], ...]

    Returns:
        float: total turn penalty in seconds
    """
    if len(path_coords) < 3:
        return 0.0

    def bearing(p1, p2):
        """Compute compass bearing from p1 to p2 (both [lat, lon])."""
        lat1, lon1 = math.radians(p1[0]), math.radians(p1[1])
        lat2, lon2 = math.radians(p2[0]), math.radians(p2[1])
        dlon = lon2 - lon1
        x = math.sin(dlon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(
            lat2
        ) * math.cos(dlon)
        b = math.degrees(math.atan2(x, y))
        return (b + 360) % 360

    # Sample every N points to avoid noise from dense GPS-like coordinates
    SAMPLE = max(1, len(path_coords) // 20)
    sampled = path_coords[::SAMPLE]
    if sampled[-1] != path_coords[-1]:
        sampled.append(path_coords[-1])

    total_penalty = 0.0

    for i in range(1, len(sampled) - 1):
        b1 = bearing(sampled[i - 1], sampled[i])
        b2 = bearing(sampled[i], sampled[i + 1])
        angle = (b2 - b1 + 180) % 360 - 180

        if abs(angle) < 20:
            total_penalty += COST_STRAIGHT
        elif abs(angle) > 150:
            total_penalty += COST_U_TURN
        elif angle > 0:
            total_penalty += COST_RIGHT
        else:
            total_penalty += COST_LEFT

    return total_penalty


# ==========================================
# SPOT AUGMENTATION (OSM — unchanged)
# ==========================================


def augment_graph_with_spots(spots):
    """
    Split road edges by injecting spot nodes into the OSM graph.
    Used for: spot-node resolution in postprocess_chain (legacy).
    Now less critical since routing is coordinate-based via 2GIS.
    """
    global G
    initialize_graph()

    G_aug = G.copy()

    lats = [s["coords"][0] for s in spots]
    lngs = [s["coords"][1] for s in spots]
    nearest_edges = ox.nearest_edges(G_aug, lngs, lats)

    edge_to_spots = {}
    for i, edge in enumerate(nearest_edges):
        spots[i]["_edge"] = edge
        edge_to_spots.setdefault(edge, []).append(spots[i])

    def build_edge_geometry(graph, a, b, edge_data):
        geom = edge_data.get("geometry")
        if geom is None:
            ax, ay = graph.nodes[a]["x"], graph.nodes[a]["y"]
            bx, by = graph.nodes[b]["x"], graph.nodes[b]["y"]
            geom = LineString([(ax, ay), (bx, by)])
        return geom

    def project_spots_onto_edge(graph, a, b, edge_data, spot_list, reverse=False):
        edge_geom = build_edge_geometry(graph, a, b, edge_data)
        geom_len = edge_geom.length
        total_len = float(edge_data.get("length", 0.0))

        out = []
        for s in spot_list:
            p = Point(s["coords"][1], s["coords"][0])
            proj_dist_geom = edge_geom.project(p)
            ratio = (proj_dist_geom / geom_len) if geom_len > 0 else 0.0
            ratio = max(0.0, min(1.0, ratio))
            if reverse:
                ratio = 1.0 - ratio
            offset_m = ratio * total_len
            proj_point = edge_geom.interpolate(proj_dist_geom)
            out.append({"spot": s, "offset_m": offset_m, "proj_point": proj_point})

        out.sort(key=lambda x: x["offset_m"])
        return out

    def add_split_chain(graph, a, b, k_edge, edge_data, projected_spots):
        total_len = float(edge_data.get("length", 0.0))
        total_time = float(edge_data.get("travel_time_new", 0.0))
        curr = a
        last_offset_m = 0.0

        for item in projected_spots:
            s = item["spot"]
            new_node = f"spot_{s['id']}"
            proj_point = item["proj_point"]
            offset_m = item["offset_m"]

            if new_node not in graph:
                graph.add_node(new_node, x=proj_point.x, y=proj_point.y, is_manual=True)
                s["_node_id"] = new_node
                s["_proj_coords"] = (proj_point.y, proj_point.x)
                s["_bearing"] = edge_data.get("bearing", 0)

            seg_len = max(0.0, offset_m - last_offset_m)
            seg_ratio = (seg_len / total_len) if total_len > 0 else 0.0
            seg_time = total_time * seg_ratio

            graph.add_edge(
                curr,
                new_node,
                length=seg_len,
                travel_time_new=seg_time,
                highway=edge_data.get("highway"),
                name=edge_data.get("name"),
            )

            curr = new_node
            last_offset_m = offset_m

        final_len = max(0.0, total_len - last_offset_m)
        final_ratio = (final_len / total_len) if total_len > 0 else 0.0
        graph.add_edge(
            curr,
            b,
            length=final_len,
            travel_time_new=total_time * final_ratio,
            highway=edge_data.get("highway"),
            name=edge_data.get("name"),
        )

        if graph.has_edge(a, b, k_edge):
            graph.remove_edge(a, b, k_edge)

    processed = set()

    for (u, v, k), spot_list in list(edge_to_spots.items()):
        if (u, v, k) in processed:
            continue

        edge_data = G_aug.get_edge_data(u, v, k)
        if not edge_data:
            continue

        combined_spots = list(spot_list)
        reverse_keys = []
        if G_aug.has_edge(v, u):
            reverse_keys = list(G_aug[v][u].keys())
            for k_rev in reverse_keys:
                combined_spots.extend(edge_to_spots.get((v, u, k_rev), []))

        unique_spots = []
        seen_ids = set()
        for s in combined_spots:
            if s["id"] not in seen_ids:
                seen_ids.add(s["id"])
                unique_spots.append(s)

        forward_projected = project_spots_onto_edge(
            G_aug, u, v, edge_data, unique_spots, reverse=False
        )
        add_split_chain(G_aug, u, v, k, edge_data, forward_projected)
        processed.add((u, v, k))

        for k_rev in reverse_keys:
            if (v, u, k_rev) in processed:
                continue
            rev_edge_data = G_aug.get_edge_data(v, u, k_rev)
            if not rev_edge_data:
                continue
            reverse_projected = project_spots_onto_edge(
                G_aug, v, u, rev_edge_data, unique_spots, reverse=True
            )
            add_split_chain(G_aug, v, u, k_rev, rev_edge_data, reverse_projected)
            processed.add((v, u, k_rev))

    return G_aug


# ==========================================
# MAIN ROUTE FUNCTION — 2GIS PRIMARY, OSM FALLBACK
# ==========================================


def _coords_from_ref(ref, custom_G=None):
    """
    Resolve a route endpoint to (lat, lon) coordinates.

    Accepts:
        - (lat, lon) tuple/list
        - str node id  → look up in graph
        - dict with 'coords' or '_proj_coords'
    """
    if isinstance(ref, (list, tuple)) and len(ref) == 2:
        # Check if it's a plain coordinate pair (floats)
        if isinstance(ref[0], float) or isinstance(ref[1], float):
            return (float(ref[0]), float(ref[1]))
        # Could be [lat, lon] as ints
        return (float(ref[0]), float(ref[1]))

    if isinstance(ref, str):
        # Node ID in graph
        target_G = custom_G if custom_G else G
        if target_G and ref in target_G.nodes:
            node = target_G.nodes[ref]
            return (node["y"], node["x"])
        return None

    if isinstance(ref, dict):
        if "_proj_coords" in ref:
            return tuple(ref["_proj_coords"])
        if "coords" in ref:
            return tuple(ref["coords"])

    return None


def _get_osm_route(u_coords, v_coords, custom_G=None):
    """
    OSM/NetworkX fallback routing. Returns same dict format.
    """
    target_G = custom_G if custom_G else G
    if target_G is None:
        return {
            "path": [],
            "travel_time": float("inf"),
            "turn_penalty": 0.0,
            "nodes": [],
        }

    try:
        u_node = ox.nearest_nodes(target_G, u_coords[1], u_coords[0])
        v_node = ox.nearest_nodes(target_G, v_coords[1], v_coords[0])

        path = nx.shortest_path(target_G, u_node, v_node, weight="travel_time_new")

        path_coords = []
        time = 0.0

        for i in range(len(path) - 1):
            a, b = path[i], path[i + 1]
            edge_dict = target_G.get_edge_data(a, b)
            if not edge_dict:
                continue

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

        turn_penalty = _compute_turn_penalty_from_path(path_coords)

        return {
            "path": path_coords,
            "travel_time": time + turn_penalty,
            "turn_penalty": turn_penalty,
            "nodes": path,
            "source": "osm",
        }

    except Exception as e:
        print(f"[OSM fallback] failed: {e}")
        return {
            "path": [],
            "travel_time": float("inf"),
            "turn_penalty": 0.0,
            "nodes": [],
        }


def get_route(u_ref, v_ref, custom_G=None):
    """
    Compute route between two points.

    PRIMARY:  2GIS Routing API (real traffic, accurate geometry)
    FALLBACK: OSM NetworkX graph

    Args:
        u_ref: start — (lat, lon), spot dict, node id string, or list
        v_ref: end   — same formats
        custom_G: OSM graph to use for fallback and coord resolution

    Returns:
        dict:
            path:         [[lat, lon], ...]
            travel_time:  float (seconds, includes turn penalty)
            turn_penalty: float (seconds)
            nodes:        list (empty for 2GIS, populated for OSM fallback)
    """
    # Resolve references to (lat, lon) coordinates
    u_coords = _coords_from_ref(u_ref, custom_G)
    v_coords = _coords_from_ref(v_ref, custom_G)

    if u_coords is None or v_coords is None:
        print(f"[get_route] Could not resolve coords: u={u_ref}, v={v_ref}")
        return {
            "path": [],
            "travel_time": float("inf"),
            "turn_penalty": 0.0,
            "nodes": [],
        }

    # 1. Try 2GIS
    result = get_route_2gis(u_coords, v_coords)

    if result and result["path"]:
        # Compute turn penalty from returned geometry
        turn_penalty = _compute_turn_penalty_from_path(result["path"])
        result["turn_penalty"] = turn_penalty
        result["travel_time"] = result["travel_time"] + turn_penalty
        return result

    # 2. OSM fallback
    print(f"[get_route] 2GIS failed, using OSM fallback: {u_coords} → {v_coords}")
    return _get_osm_route(u_coords, v_coords, custom_G)


# ==========================================
# PEDESTRIAN CROSSING COUNT (OSM — unchanged)
# ==========================================


def normalize_street_name(name):
    if name is None:
        return None
    if isinstance(name, list):
        name = " | ".join(sorted(str(x).strip().lower() for x in name if x))
    else:
        name = str(name).strip().lower()
    return name or None


def _point_side_relative_to_edge(point_latlon, G, edge):
    u, v, k = edge
    x1, y1 = G.nodes[u]["x"], G.nodes[u]["y"]
    x2, y2 = G.nodes[v]["x"], G.nodes[v]["y"]
    px, py = point_latlon[1], point_latlon[0]
    return (x2 - x1) * (py - y1) - (y2 - y1) * (px - x1)


def count_crossings(spot_coords, dest_coords, G, ignore_edge=None, eps=1e-10):
    walk_line = LineString(
        [
            (spot_coords[1], spot_coords[0]),
            (dest_coords[1], dest_coords[0]),
        ]
    )

    crossed_streets = set()
    spot_street_name = None
    opposite_sides = False

    if ignore_edge is not None and G.has_edge(*ignore_edge):
        spot_data = G.edges[ignore_edge]
        spot_street_name = normalize_street_name(spot_data.get("name"))
        spot_sign = _point_side_relative_to_edge(spot_coords, G, ignore_edge)
        dest_sign = _point_side_relative_to_edge(dest_coords, G, ignore_edge)
        if abs(spot_sign) > eps and abs(dest_sign) > eps and spot_sign * dest_sign < 0:
            opposite_sides = True

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
        if highway in {"service", "parking_aisle", "driveway"}:
            continue

        crossed_name = normalize_street_name(data.get("name"))
        if crossed_name is None:
            continue

        if not opposite_sides and spot_street_name is not None:
            if crossed_name == spot_street_name:
                continue

        crossed_streets.add(crossed_name)

    return len(crossed_streets)


# ==========================================
# ALTERNATIVE ROUTE GENERATION
# ==========================================


def _get_edge_data_for_path_pair(target_G, u, v):
    edge_data = target_G.get_edge_data(u, v)
    if not edge_data:
        return None
    if isinstance(edge_data, dict):
        first_val = list(edge_data.values())[0]
        if isinstance(first_val, dict):
            return min(
                edge_data.values(), key=lambda d: d.get("travel_time_new", float("inf"))
            )
    return edge_data


def get_street_lengths_on_route(route_nodes, target_G=None):
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
    target_G = target_G if target_G is not None else G
    street_lengths = get_street_lengths_on_route(route_nodes, target_G=target_G)

    if not street_lengths:
        return {"street": None, "length": 0.0, "all_streets": {}}

    main_street, total_len = max(street_lengths.items(), key=lambda x: x[1])
    return {"street": main_street, "length": total_len, "all_streets": street_lengths}


def build_penalized_graph(
    penalized_street=None, street_penalty_factor=5.0, custom_G=None
):
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
    Get primary route via 2GIS (jam) and alternative via 2GIS (shortest).
    Falls back to OSM-based penalized street approach if 2GIS fails.
    """
    from twogis_routing import get_route_2gis

    # Primary: real-time traffic
    primary_raw = get_route_2gis(u_coords, v_coords, route_type="jam")
    if primary_raw and primary_raw["path"]:
        turn_penalty = _compute_turn_penalty_from_path(primary_raw["path"])
        primary_raw["turn_penalty"] = turn_penalty
        primary_raw["travel_time"] += turn_penalty
        primary = primary_raw
    else:
        primary = _get_osm_route(u_coords, v_coords, custom_G)

    # Alternative: shortest distance
    alt_raw = get_route_2gis(u_coords, v_coords, route_type="shortest")
    if alt_raw and alt_raw["path"]:
        turn_penalty = _compute_turn_penalty_from_path(alt_raw["path"])
        alt_raw["turn_penalty"] = turn_penalty
        alt_raw["travel_time"] += turn_penalty
        alternative = alt_raw
    else:
        # OSM penalized fallback
        target_G = custom_G if custom_G else G
        if primary.get("nodes"):
            main_info = get_main_street_from_route(primary["nodes"], target_G=target_G)
            main_street = main_info["street"]
        else:
            main_street = None

        if main_street:
            G_penalized = build_penalized_graph(
                penalized_street=main_street,
                street_penalty_factor=street_penalty_factor,
                custom_G=target_G,
            )
            alternative = _get_osm_route(u_coords, v_coords, G_penalized)
        else:
            alternative = primary

    # Derive main_street from OSM nodes if available, else None
    main_street = None
    if primary.get("nodes") and (custom_G or G):
        target_G = custom_G if custom_G else G
        main_info = get_main_street_from_route(primary["nodes"], target_G=target_G)
        main_street = main_info["street"]

    return {
        "primary": primary,
        "alternative": alternative,
        "main_street": main_street,
    }
