from flask import Flask, request, jsonify, render_template
import os, time, json, math
import numpy as np
from jinja2 import FileSystemLoader
from datetime import datetime, timedelta
import pytz
from geopy.distance import geodesic
import requests as http_requests
import routing
import pandas as pd

from scenarios import SCENARIOS
from locations import LOCATIONS
from transformation import StateAdapter
from algorithms import (
    FiniteHorizonMDP,
    HeuristicLookahead,
    MDP_Difference,
    TwoStageMDP,
)
from twogis_routing import get_routes_2gis_both, get_route_2gis

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
STATIC_DIR = os.path.join(BASE_DIR, "static")
MANUAL_FILE = os.path.join(BASE_DIR, "manual_spots_new.json")
DISTANCE_CACHE_FILE = os.path.join(BASE_DIR, "distance_cache.json")

if not os.path.exists(STATIC_DIR):
    os.makedirs(STATIC_DIR, exist_ok=True)

app = Flask(__name__, static_folder=STATIC_DIR, template_folder=BASE_DIR)
app.jinja_loader = FileSystemLoader(BASE_DIR)
routing.initialize_graph()


def traffic_multiplier_for_time(dt_local):
    h = dt_local.hour + dt_local.minute / 60
    is_weekend = dt_local.weekday() >= 5

    if not is_weekend:
        if 7.5 <= h <= 9:
            return 1.65
        if 9 < h <= 12:
            return 1.25
        if 12 < h <= 14:
            return 1.40
        if 14 < h <= 18:
            return 1.25
        if 18 < h <= 19.5:
            return 1.80
        if 19.5 < h < 22:
            return 1.30
        return 1.00
    else:
        if 10 <= h < 13:
            return 1.35
        if 13 <= h < 18:
            return 1.25
        if 18 <= h < 21.5:
            return 1.45
        return 1.00


def get_weather_multipliers(lat, lon):
    try:
        url = (
            f"https://api.open-meteo.com/v1/forecast"
            f"?latitude={lat}&longitude={lon}"
            f"&current=weathercode,windspeed_10m,temperature_2m"
            f"&forecast_days=1"
        )
        r = http_requests.get(url, timeout=3)
        c = r.json()["current"]
        code = c["weathercode"]
        wind = c["windspeed_10m"]
        temp = c["temperature_2m"]

        mult = 1.0

        if code in range(51, 68) or code in range(80, 83):
            mult = 1.3
        elif code in range(71, 78) or code in range(85, 87):
            mult = 1.4
        elif code >= 95:
            mult = 1.8

        if wind > 50:
            mult = max(mult, 1.8)
        elif wind > 35:
            mult = max(mult, 1.5)
        elif wind > 20:
            mult = max(mult, 1.2)

        if temp < -15 and temp > -20:
            mult = max(mult, 1.3)
        elif temp <= -20:
            mult = max(mult, 1.6)

        if (code in range(71, 78) or code in range(85, 87)) and wind > 20:
            mult = max(mult, 1.6)

        return mult

    except Exception:
        return 1.0


def load_json(path, default):
    if os.path.exists(path):
        try:
            with open(path, "r") as f:
                return json.load(f)
        except:
            return default
    return default


def save_json(path, data):
    with open(path, "w") as f:
        json.dump(data, f)


MANUAL_SPOTS = load_json(MANUAL_FILE, [])
DISTANCE_CACHE = {}


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/get_locations")
def get_locations():
    return jsonify(LOCATIONS)


@app.route("/get_manual_spots")
def get_manual():
    return jsonify(MANUAL_SPOTS)


@app.route("/add_manual_spot", methods=["POST"])
def add_manual_spot():
    data = request.json
    spot = {
        "id": f"manual_{int(time.time() * 1000)}",
        "coords": (data["lat"], data["lng"]),
        "street": "Manual Entry",
        "phi_exit_seconds": int(data.get("phi", 60)),
        "p_i": float(data.get("pi", 0.8)),
    }
    MANUAL_SPOTS.append(spot)
    save_json(MANUAL_FILE, MANUAL_SPOTS)
    return jsonify({"status": "success", "spot": spot})


@app.route("/delete_spot", methods=["POST"])
def delete_spot():
    spot_id = request.json.get("id")
    global MANUAL_SPOTS
    MANUAL_SPOTS = [s for s in MANUAL_SPOTS if s["id"] != spot_id]
    save_json(MANUAL_FILE, MANUAL_SPOTS)
    return jsonify({"status": "success"})


@app.route("/update_spot", methods=["POST"])
def update_spot():
    data = request.json
    for s in MANUAL_SPOTS:
        if s["id"] == data.get("id"):
            s["phi_exit_seconds"] = int(data.get("phi", 60))
            s["p_i"] = float(data.get("pi", 0.8))
            break
    save_json(MANUAL_FILE, MANUAL_SPOTS)
    return jsonify({"status": "success"})


def parse_coord(c):
    if isinstance(c, (list, tuple)) and len(c) >= 2:
        return (float(c[0]), float(c[1]))
    return (0.0, 0.0)


def restrict_spots_by_walk_time(spots, dest, limit=25, walk_speed_mps=1.3):
    ranked = sorted(
        spots, key=lambda s: geodesic(s["coords"], dest).meters / walk_speed_mps
    )
    return ranked[:limit]


def build_display_segments_2gis(
    chain, state, start, dest, ref, G_used, traffic_mult_drive
):
    """
    Build display segments using 2GIS Routing API for map polylines.
    Falls back to OSM routing if 2GIS fails for any leg.

    Returns: (segments, details)
    """
    segments = []
    details = []
    curr_origin_coords = start

    for i, idx in enumerate(chain):
        spot = state["spots"][idx]
        spot_node_id = f"spot_{spot['id']}"
        spot_coords = spot["coords"]

        # Try 2GIS for this drive leg
        twogis_res = get_route_2gis(curr_origin_coords, spot_coords)

        if twogis_res and twogis_res["path"]:
            leg_path = twogis_res["path"]
            leg_drive_time = twogis_res["travel_time"] * traffic_mult_drive
            routing_source = "2gis"
        else:
            # Fallback to OSM
            osm_res = routing.get_route(
                curr_origin_coords
                if i == 0
                else f"spot_{state['spots'][chain[i - 1]]['id']}",
                spot_node_id,
                custom_G=G_used,
            )
            leg_path = osm_res["path"]
            leg_drive_time = osm_res["travel_time"] * traffic_mult_drive
            routing_source = "osm"

        phi_prev = state["spots"][chain[i - 1]]["phi_exit_seconds"] if i > 0 else 0
        total_arrival_time = leg_drive_time + phi_prev

        segments.append(
            {
                "coords": leg_path,
                "type": "drive_transition",
                "label": spot.get("street", "Spot"),
                "source": routing_source,
            }
        )

        details.append(
            {
                "order": i + 1,
                "street": spot.get("street", "Spot"),
                "coords": spot["coords"],
                "p": spot.get("p_i", 0.8),
                "phi": spot.get("phi_exit_seconds", 60),
                "drive_leg": leg_drive_time,
                "phi_prev": phi_prev,
                "arrival_time": total_arrival_time,
                "walk_b": state["walk_fn"](spot["coords"]),
                "exit_ref": state["drive_fn"](("spot", spot), ("node", "ref"))[
                    "travel_time"
                ],
            }
        )

        curr_origin_coords = spot_coords

    # Exit leg: last spot → ref
    last_spot_coords = state["spots"][chain[-1]]["coords"]
    last_spot_node_id = f"spot_{state['spots'][chain[-1]]['id']}"

    twogis_exit = get_route_2gis(last_spot_coords, ref)
    if twogis_exit and twogis_exit["path"]:
        exit_path = twogis_exit["path"]
    else:
        osm_exit = routing.get_route(last_spot_node_id, ref, custom_G=G_used)
        exit_path = osm_exit["path"]

    segments.append({"coords": exit_path, "type": "drive_exit"})

    # Walk leg: last spot → dest (always straight line)
    segments.append(
        {
            "coords": [state["spots"][chain[-1]]["coords"], dest],
            "type": "walk",
        }
    )

    return segments, details


def build_solution(
    start,
    dest,
    ref,
    scenario_key,
    algo_choice,
    exit_multiplier,
    routing_mode="normal",
    penalized_street=None,
    street_penalty_factor=5.0,
    use_2gis_display=True,
):
    DISTANCE_CACHE.clear()

    weather_mult = get_weather_multipliers(start[0], start[1])

    if not MANUAL_SPOTS:
        return {"status": "error", "message": "No spots defined."}

    G_aug = routing.augment_graph_with_spots(MANUAL_SPOTS)

    G_used = G_aug
    if routing_mode == "penalized" and penalized_street:
        G_used = routing.build_penalized_graph(
            penalized_street=penalized_street,
            street_penalty_factor=street_penalty_factor,
            custom_G=G_aug,
        )

    scenario_cfg = SCENARIOS.get(scenario_key, SCENARIOS["Short Stay (10-40m)"])
    params = scenario_cfg["params"].copy()

    traffic_mult_drive = params.get("traffic_multiplier_drive", 1.4)
    traffic_mult_exit = params.get("traffic_multiplier_exit", 1.4)

    if scenario_key == "Custom":
        params = {
            "lambda_w": 1.5,
            "lambda_e": 1.0,
            "lambda_tr": 1.0,
            "lambda_turns_arr": 1.0,
            "lambda_turns_exit": 1.0,
            "lambda_cross": 1.0,
            "k": 5,
        }

    params["lambda_w"] *= weather_mult
    params["lambda_cross"] *= weather_mult

    def node_key(x):
        if x[0] == "spot":
            return f"spot_{x[1]['id']}"
        if x[0] == "node" and x[1] == "start":
            return f"start_{start[0]:.6f}_{start[1]:.6f}"
        if x[0] == "node" and x[1] == "ref":
            return f"ref_{ref[0]:.6f}_{ref[1]:.6f}"
        return f"{x[0]}_{x[1]}"

    def drive_fn(u, v):
        is_exit_leg = v[0] == "node" and v[1] == "ref"
        mult = traffic_mult_exit if is_exit_leg else traffic_mult_drive

        u_id = node_key(u)
        v_id = node_key(v)

        route_mode_key = routing_mode
        street_key = penalized_street if penalized_street else "none"
        cache_key = (
            f"{u_id}_to_{v_id}_mult_{mult}_mode_{route_mode_key}_street_{street_key}"
        )

        if cache_key in DISTANCE_CACHE:
            return DISTANCE_CACHE[cache_key]

        u_t = (
            f"spot_{u[1]['id']}"
            if u[0] == "spot"
            else (start if u[1] == "start" else ref)
        )
        v_t = (
            f"spot_{v[1]['id']}"
            if v[0] == "spot"
            else (ref if v[1] == "ref" else start)
        )

        res = routing.get_route(u_t, v_t, custom_G=G_used)

        if not res["nodes"]:
            result = {
                "travel_time": float("inf"),
                "turn_penalty": float("inf"),
            }
            DISTANCE_CACHE[cache_key] = result
            return result

        result = {
            "travel_time": res["travel_time"] * mult,
            "turn_penalty": res["turn_penalty"] * mult,
        }
        DISTANCE_CACHE[cache_key] = result
        return result

    restricted_spots = restrict_spots_by_walk_time(MANUAL_SPOTS, dest, limit=40)

    adapter = StateAdapter(
        start,
        dest,
        ref,
        restricted_spots,
        traffic_multiplier=traffic_mult_drive,
    )

    state = adapter.get_state()
    state["drive_fn"] = drive_fn
    state["exit_multiplier"] = exit_multiplier
    state["G_aug"] = G_used

    solver_class = (
        MDP_Difference
        if algo_choice == "MDP_Difference"
        else (
            FiniteHorizonMDP
            if algo_choice == "MDP"
            else (TwoStageMDP if algo_choice == "TwoStage" else HeuristicLookahead)
        )
    )
    solver = solver_class(state)
    chain, metrics, _ = solver.solve(**params)

    if not chain:
        return {"status": "error", "message": "Pathfinding failed."}

    if hasattr(solver, "DEBUG_DATA") and solver.DEBUG_DATA:
        df = pd.DataFrame(solver.DEBUG_DATA)
        df.to_csv("debug_distribution.csv", index=False)

    # Build display segments — use 2GIS if enabled, else OSM
    if use_2gis_display:
        segments, details = build_display_segments_2gis(
            chain, state, start, dest, ref, G_used, traffic_mult_drive
        )
    else:
        # Original OSM-based segment building
        segments = []
        details = []
        curr_origin_id = start

        for i, idx in enumerate(chain):
            spot = state["spots"][idx]
            spot_node_id = f"spot_{spot['id']}"

            res = routing.get_route(curr_origin_id, spot_node_id, custom_G=G_used)

            leg_drive_time = res["travel_time"] * traffic_mult_drive
            phi_prev = state["spots"][chain[i - 1]]["phi_exit_seconds"] if i > 0 else 0
            total_arrival_time = leg_drive_time + phi_prev

            segments.append(
                {
                    "coords": res["path"],
                    "type": "drive_transition",
                    "label": spot.get("street", "Spot"),
                }
            )

            details.append(
                {
                    "order": i + 1,
                    "street": spot.get("street", "Spot"),
                    "coords": spot["coords"],
                    "p": spot.get("p_i", 0.8),
                    "phi": spot.get("phi_exit_seconds", 60),
                    "drive_leg": leg_drive_time,
                    "phi_prev": phi_prev,
                    "arrival_time": total_arrival_time,
                    "walk_b": state["walk_fn"](spot["coords"]),
                    "exit_ref": state["drive_fn"](("spot", spot), ("node", "ref"))[
                        "travel_time"
                    ],
                }
            )

            curr_origin_id = spot_node_id

        last_spot_id = f"spot_{state['spots'][chain[-1]]['id']}"
        res_ex = routing.get_route(last_spot_id, ref, custom_G=G_used)

        segments.append({"coords": res_ex["path"], "type": "drive_exit"})
        segments.append(
            {
                "coords": [state["spots"][chain[-1]]["coords"], dest],
                "type": "walk",
            }
        )

    success_prob = 1.0 - np.prod([1 - state["spots"][idx]["p_i"] for idx in chain])

    return {
        "status": "success",
        "metrics": {
            "expected_total_sec": metrics[0],
            "expected_drive_sec": metrics[1],
            "expected_walk_sec": metrics[2],
            "expected_exit_sec": metrics[3],
            "expected_cross_sec": metrics[4],
            "success_probability": metrics[5],
        },
        "details": details,
        "segments": segments,
        "chain": chain,
        "routing_mode": routing_mode,
        "penalized_street": penalized_street,
    }


@app.route("/solve", methods=["POST"])
def solve():
    data = request.json
    start = parse_coord(data.get("start"))
    dest = parse_coord(data.get("dest"))
    ref = parse_coord(data.get("ref"))
    exit_multiplier = float(data.get("exit_multiplier", 1.0))
    scenario_key = data.get("scenario", "Short Stay (10-40m)")
    algo_choice = data.get("algo", "MDP_Difference")
    street_penalty_factor = float(data.get("street_penalty_factor", 5.0))

    if not MANUAL_SPOTS:
        return jsonify({"status": "error", "message": "No spots defined."})

    route_pair = routing.get_primary_and_alternative_route(
        start,
        dest,
        custom_G=routing.G,
        street_penalty_factor=street_penalty_factor,
    )

    main_street = route_pair["main_street"]

    # Option 1: normal routing, 2GIS display
    option1 = build_solution(
        start=start,
        dest=dest,
        ref=ref,
        scenario_key=scenario_key,
        algo_choice=algo_choice,
        exit_multiplier=exit_multiplier,
        routing_mode="normal",
        penalized_street=None,
        street_penalty_factor=street_penalty_factor,
        use_2gis_display=True,
    )

    if option1["status"] != "success":
        return jsonify(option1)

    # Option 2: penalized main street, 2GIS display
    if main_street:
        option2 = build_solution(
            start=start,
            dest=dest,
            ref=ref,
            scenario_key=scenario_key,
            algo_choice=algo_choice,
            exit_multiplier=exit_multiplier,
            routing_mode="penalized",
            penalized_street=main_street,
            street_penalty_factor=street_penalty_factor,
            use_2gis_display=True,
        )
    else:
        option2 = {
            "status": "success",
            "metrics": option1["metrics"],
            "details": option1["details"],
            "segments": option1["segments"],
            "chain": option1["chain"],
            "routing_mode": "normal",
            "penalized_street": None,
        }

    print("METRICS:", option1["metrics"])

    return jsonify(
        {
            "status": "success",
            "main_street_from_base_route": main_street,
            "solutions": [
                {"name": "Option 1", **option1},
                {"name": "Option 2", **option2},
            ],
        }
    )


if __name__ == "__main__":
    app.run(debug=True, port=5000)


# from flask import Flask, request, jsonify, render_template
# import os, time, json, math
# import numpy as np
# from jinja2 import FileSystemLoader
# from geopy.distance import geodesic
# import requests as http_requests
# import routing
# import pandas as pd

# from scenarios import SCENARIOS
# from locations import LOCATIONS
# from transformation import StateAdapter
# from algorithms_new import (
#     FiniteHorizonMDP,
#     HeuristicLookahead,
#     MDP_Difference,
#     TwoStageMDP,
# )

# BASE_DIR = os.path.dirname(os.path.abspath(__file__))
# STATIC_DIR = os.path.join(BASE_DIR, "static")
# MANUAL_FILE = os.path.join(BASE_DIR, "manual_spots_new.json")

# if not os.path.exists(STATIC_DIR):
#     os.makedirs(STATIC_DIR, exist_ok=True)

# app = Flask(__name__, static_folder=STATIC_DIR, template_folder=BASE_DIR)
# app.jinja_loader = FileSystemLoader(BASE_DIR)
# routing.initialize_graph()


# def get_weather_multipliers(lat, lon):
#     try:
#         url = (
#             f"https://api.open-meteo.com/v1/forecast"
#             f"?latitude={lat}&longitude={lon}"
#             f"&current=weathercode,windspeed_10m,temperature_2m"
#             f"&forecast_days=1"
#         )
#         r = http_requests.get(url, timeout=3)
#         c = r.json()["current"]
#         code = c["weathercode"]
#         wind = c["windspeed_10m"]
#         temp = c["temperature_2m"]
#         mult = 1.0
#         if code in range(51, 68) or code in range(80, 83):
#             mult = 1.3
#         elif code in range(71, 78) or code in range(85, 87):
#             mult = 1.4
#         elif code >= 95:
#             mult = 1.8
#         if wind > 50:
#             mult = max(mult, 1.8)
#         elif wind > 35:
#             mult = max(mult, 1.5)
#         elif wind > 20:
#             mult = max(mult, 1.2)
#         if temp < -15 and temp > -20:
#             mult = max(mult, 1.3)
#         elif temp <= -20:
#             mult = max(mult, 1.6)
#         if (code in range(71, 78) or code in range(85, 87)) and wind > 20:
#             mult = max(mult, 1.6)
#         return mult
#     except Exception:
#         return 1.0


# def load_json(path, default):
#     if os.path.exists(path):
#         try:
#             with open(path, "r") as f:
#                 return json.load(f)
#         except:
#             return default
#     return default


# def save_json(path, data):
#     with open(path, "w") as f:
#         json.dump(data, f)


# MANUAL_SPOTS = load_json(MANUAL_FILE, [])
# DISTANCE_CACHE = {}


# @app.route("/")
# def index():
#     return render_template("index.html")


# @app.route("/get_locations")
# def get_locations():
#     return jsonify(LOCATIONS)


# @app.route("/get_manual_spots")
# def get_manual():
#     return jsonify(MANUAL_SPOTS)


# @app.route("/add_manual_spot", methods=["POST"])
# def add_manual_spot():
#     data = request.json
#     spot = {
#         "id": f"manual_{int(time.time() * 1000)}",
#         "coords": (data["lat"], data["lng"]),
#         "street": "Manual Entry",
#         "phi_exit_seconds": int(data.get("phi", 60)),
#         "p_i": float(data.get("pi", 0.8)),
#     }
#     MANUAL_SPOTS.append(spot)
#     save_json(MANUAL_FILE, MANUAL_SPOTS)
#     return jsonify({"status": "success", "spot": spot})


# @app.route("/delete_spot", methods=["POST"])
# def delete_spot():
#     spot_id = request.json.get("id")
#     global MANUAL_SPOTS
#     MANUAL_SPOTS = [s for s in MANUAL_SPOTS if s["id"] != spot_id]
#     save_json(MANUAL_FILE, MANUAL_SPOTS)
#     return jsonify({"status": "success"})


# @app.route("/update_spot", methods=["POST"])
# def update_spot():
#     data = request.json
#     for s in MANUAL_SPOTS:
#         if s["id"] == data.get("id"):
#             s["phi_exit_seconds"] = int(data.get("phi", 60))
#             s["p_i"] = float(data.get("pi", 0.8))
#             break
#     save_json(MANUAL_FILE, MANUAL_SPOTS)
#     return jsonify({"status": "success"})


# def parse_coord(c):
#     if isinstance(c, (list, tuple)) and len(c) >= 2:
#         return (float(c[0]), float(c[1]))
#     return (0.0, 0.0)


# def restrict_spots_by_walk_time(spots, dest, limit=40, walk_speed_mps=1.3):
#     ranked = sorted(
#         spots, key=lambda s: geodesic(s["coords"], dest).meters / walk_speed_mps
#     )
#     return ranked[:limit]


# def make_drive_fn(start, ref, traffic_mult_drive, traffic_mult_exit, distance_cache):
#     """
#     drive_fn using routing.get_route() → 2GIS primary, OSM fallback.
#     All optimization calls go through here.
#     """

#     def _get_coords(x):
#         if x[0] == "node":
#             return tuple(start) if x[1] == "start" else tuple(ref)
#         if x[0] == "spot":
#             return tuple(x[1]["coords"])
#         return None

#     def drive_fn(u, v):
#         is_exit_leg = v[0] == "node" and v[1] == "ref"
#         mult = traffic_mult_exit if is_exit_leg else traffic_mult_drive

#         u_id = f"spot_{u[1]['id']}" if u[0] == "spot" else f"{u[1]}"
#         v_id = f"spot_{v[1]['id']}" if v[0] == "spot" else f"{v[1]}"
#         cache_key = f"{u_id}_to_{v_id}_mult_{mult:.2f}"

#         if cache_key in distance_cache:
#             return distance_cache[cache_key]

#         u_coords = _get_coords(u)
#         v_coords = _get_coords(v)

#         if u_coords is None or v_coords is None:
#             result = {"travel_time": float("inf"), "turn_penalty": float("inf")}
#             distance_cache[cache_key] = result
#             return result

#         res = routing.get_route(u_coords, v_coords)

#         if not res["path"]:
#             result = {"travel_time": float("inf"), "turn_penalty": float("inf")}
#             distance_cache[cache_key] = result
#             return result

#         result = {
#             "travel_time": res["travel_time"] * mult,
#             "turn_penalty": res["turn_penalty"] * mult,
#         }
#         distance_cache[cache_key] = result
#         return result

#     return drive_fn


# def build_solution(
#     start,
#     dest,
#     ref,
#     scenario_key,
#     algo_choice,
#     exit_multiplier,
#     route_type="jam",
# ):
#     distance_cache = {}
#     weather_mult = get_weather_multipliers(start[0], start[1])

#     if not MANUAL_SPOTS:
#         return {"status": "error", "message": "No spots defined."}

#     scenario_cfg = SCENARIOS.get(scenario_key, SCENARIOS["Short Stay (10-40m)"])
#     params = scenario_cfg["params"].copy()

#     traffic_mult_drive = params.get("traffic_multiplier_drive", 1.4)
#     traffic_mult_exit = params.get("traffic_multiplier_exit", 1.4)

#     if scenario_key == "Custom":
#         params = {
#             "lambda_w": 1.5,
#             "lambda_e": 1.0,
#             "lambda_tr": 1.0,
#             "lambda_turns_arr": 1.0,
#             "lambda_turns_exit": 1.0,
#             "lambda_cross": 1.0,
#             "k": 5,
#         }

#     params["lambda_w"] *= weather_mult
#     params["lambda_cross"] *= weather_mult

#     drive_fn = make_drive_fn(
#         start=start,
#         ref=ref,
#         traffic_mult_drive=traffic_mult_drive,
#         traffic_mult_exit=traffic_mult_exit,
#         distance_cache=distance_cache,
#     )

#     restricted_spots = restrict_spots_by_walk_time(MANUAL_SPOTS, dest, limit=40)

#     # OSM augmentation still needed for crossings computation in StateAdapter
#     G_aug = routing.augment_graph_with_spots(restricted_spots)

#     adapter = StateAdapter(
#         start, dest, ref, restricted_spots, traffic_multiplier=traffic_mult_drive
#     )
#     state = adapter.get_state()
#     state["drive_fn"] = drive_fn
#     state["exit_multiplier"] = exit_multiplier
#     state["G_aug"] = G_aug

#     solver_class = (
#         MDP_Difference
#         if algo_choice == "MDP_Difference"
#         else FiniteHorizonMDP
#         if algo_choice == "MDP"
#         else TwoStageMDP
#         if algo_choice == "TwoStage"
#         else HeuristicLookahead
#     )
#     solver = solver_class(state)
#     chain, metrics, _ = solver.solve(**params)

#     if not chain:
#         return {"status": "error", "message": "Pathfinding failed."}

#     # Build display segments — all via routing.get_route() → 2GIS
#     segments = []
#     details = []
#     curr_coords = tuple(start)

#     for i, idx in enumerate(chain):
#         spot = state["spots"][idx]
#         spot_coords = tuple(spot["coords"])

#         res = routing.get_route(curr_coords, spot_coords)
#         leg_drive_time = res["travel_time"] * traffic_mult_drive
#         phi_prev = state["spots"][chain[i - 1]]["phi_exit_seconds"] if i > 0 else 0

#         segments.append(
#             {
#                 "coords": res["path"],
#                 "type": "drive_transition",
#                 "label": spot.get("street", "Spot"),
#                 "source": res.get("source", "unknown"),
#             }
#         )

#         details.append(
#             {
#                 "order": i + 1,
#                 "street": spot.get("street", "Spot"),
#                 "coords": list(spot["coords"]),
#                 "p": spot.get("p_i", 0.8),
#                 "phi": spot.get("phi_exit_seconds", 60),
#                 "drive_leg": leg_drive_time,
#                 "phi_prev": phi_prev,
#                 "arrival_time": leg_drive_time + phi_prev,
#                 "walk_b": state["walk_fn"](spot["coords"]),
#                 "exit_ref": state["drive_fn"](("spot", spot), ("node", "ref"))[
#                     "travel_time"
#                 ],
#             }
#         )

#         curr_coords = spot_coords

#     last_spot_coords = tuple(state["spots"][chain[-1]]["coords"])
#     res_exit = routing.get_route(last_spot_coords, tuple(ref))
#     segments.append(
#         {
#             "coords": res_exit["path"],
#             "type": "drive_exit",
#             "source": res_exit.get("source", "unknown"),
#         }
#     )
#     segments.append(
#         {
#             "coords": [list(last_spot_coords), list(dest)],
#             "type": "walk",
#         }
#     )

#     success_prob = 1.0 - np.prod([1 - state["spots"][idx]["p_i"] for idx in chain])

#     return {
#         "status": "success",
#         "metrics": {
#             "expected_total_sec": metrics[0],
#             "expected_drive_sec": metrics[1],
#             "expected_walk_sec": metrics[2],
#             "expected_exit_sec": metrics[3],
#             "expected_cross_sec": metrics[4],
#             "success_probability": metrics[5],
#         },
#         "details": details,
#         "segments": segments,
#         "chain": chain,
#         "route_type": route_type,
#     }


# @app.route("/solve", methods=["POST"])
# def solve():
#     data = request.json
#     start = parse_coord(data.get("start"))
#     dest = parse_coord(data.get("dest"))
#     ref = parse_coord(data.get("ref"))
#     exit_multiplier = float(data.get("exit_multiplier", 1.0))
#     scenario_key = data.get("scenario", "Short Stay (10-40m)")
#     algo_choice = data.get("algo", "MDP_Difference")

#     if not MANUAL_SPOTS:
#         return jsonify({"status": "error", "message": "No spots defined."})

#     route_pair = routing.get_primary_and_alternative_route(start, dest)
#     main_street = route_pair.get("main_street")

#     # Option 1: traffic-aware (jam)
#     option1 = build_solution(
#         start=start,
#         dest=dest,
#         ref=ref,
#         scenario_key=scenario_key,
#         algo_choice=algo_choice,
#         exit_multiplier=exit_multiplier,
#         route_type="jam",
#     )

#     if option1["status"] != "success":
#         return jsonify(option1)

#     # Option 2: shortest distance
#     option2 = build_solution(
#         start=start,
#         dest=dest,
#         ref=ref,
#         scenario_key=scenario_key,
#         algo_choice=algo_choice,
#         exit_multiplier=exit_multiplier,
#         route_type="shortest",
#     )

#     if option2["status"] != "success":
#         option2 = {**option1, "route_type": "shortest"}

#     print("METRICS:", option1["metrics"])

#     return jsonify(
#         {
#             "status": "success",
#             "main_street_from_base_route": main_street,
#             "solutions": [
#                 {"name": "Option 1 (Traffic)", **option1},
#                 {"name": "Option 2 (Shortest)", **option2},
#             ],
#         }
#     )


# if __name__ == "__main__":
#     app.run(debug=True, port=5001)
