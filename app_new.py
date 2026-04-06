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
from twogis_routing import get_route_osrm

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
STATIC_DIR = os.path.join(BASE_DIR, "static")
MANUAL_FILE = os.path.join(BASE_DIR, "manual_spots_new.json")

if not os.path.exists(STATIC_DIR):
    os.makedirs(STATIC_DIR, exist_ok=True)

app = Flask(__name__, static_folder=STATIC_DIR, template_folder=BASE_DIR)
app.jinja_loader = FileSystemLoader(BASE_DIR)
routing.initialize_graph()

# ─────────────────────────────────────────────────────────────────────────────
# OSRM cache — persists across both Option 1 and Option 2 builds
# cleared once per /solve request (not per build_solution call)
# ─────────────────────────────────────────────────────────────────────────────
_OSRM_CACHE: dict = {}


def get_route_for_optimization(origin_coords, dest_coords, traffic_mult: float = 1.0):
    """
    Base routing for optimization AND display.

    Uses OSRM for realistic road geometry and travel time.
    Applies traffic_mult on top of OSRM duration.
    Falls back to OSM/NetworkX if OSRM fails.

    Returns:
        travel_time:  float (seconds, multiplied)
        turn_penalty: float (0.0 from OSRM, real value from OSM fallback)
        path:         [[lat, lon], ...]
        source:       "osrm" | "osm_fallback"
    """
    key = (
        round(origin_coords[0], 6),
        round(origin_coords[1], 6),
        round(dest_coords[0], 6),
        round(dest_coords[1], 6),
    )

    if key not in _OSRM_CACHE:
        _OSRM_CACHE[key] = get_route_osrm(origin_coords, dest_coords)

    osrm = _OSRM_CACHE[key]

    if osrm is not None:
        return {
            "travel_time": osrm["travel_time"] * traffic_mult,
            "turn_penalty": 0.0,
            "path": osrm["path"],
            "source": "osrm",
        }

    # Fallback to OSM/NetworkX
    print(f"[OSRM] failed {origin_coords}→{dest_coords}, using OSM fallback")
    osm = routing.get_route(origin_coords, dest_coords)
    return {
        "travel_time": osm["travel_time"] * traffic_mult,
        "turn_penalty": osm["turn_penalty"] * traffic_mult,
        "path": osm["path"],
        "source": "osm_fallback",
    }


# ─────────────────────────────────────────────────────────────────────────────
# TRAFFIC / WEATHER
# ─────────────────────────────────────────────────────────────────────────────


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
        if -20 < temp < -15:
            mult = max(mult, 1.3)
        elif temp <= -20:
            mult = max(mult, 1.6)
        if (code in range(71, 78) or code in range(85, 87)) and wind > 20:
            mult = max(mult, 1.6)
        return mult
    except Exception:
        return 1.0


# ─────────────────────────────────────────────────────────────────────────────
# JSON HELPERS
# ─────────────────────────────────────────────────────────────────────────────


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


# ─────────────────────────────────────────────────────────────────────────────
# ROUTES
# ─────────────────────────────────────────────────────────────────────────────


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


# ─────────────────────────────────────────────────────────────────────────────
# HELPERS
# ─────────────────────────────────────────────────────────────────────────────


def parse_coord(c):
    if isinstance(c, (list, tuple)) and len(c) >= 2:
        return (float(c[0]), float(c[1]))
    return (0.0, 0.0)


def restrict_spots_by_walk_time(spots, dest, limit=25, walk_speed_mps=1.3):
    ranked = sorted(
        spots, key=lambda s: geodesic(s["coords"], dest).meters / walk_speed_mps
    )
    return ranked[:limit]


# ─────────────────────────────────────────────────────────────────────────────
# DISPLAY SEGMENTS
# ─────────────────────────────────────────────────────────────────────────────


def build_display_segments(chain, state, start, dest, ref, traffic_mult_drive):
    """
    Build map polylines and drawer detail rows.
    Reuses OSRM results already cached during optimization — no extra requests.
    """
    segments = []
    details = []
    curr_coords = tuple(start)

    for i, idx in enumerate(chain):
        spot = state["spots"][idx]
        spot_coords = tuple(spot["coords"])

        # get_route_for_optimization reuses _OSRM_CACHE — no new HTTP call
        route = get_route_for_optimization(
            curr_coords, spot_coords, traffic_mult=traffic_mult_drive
        )

        leg_drive_time = route["travel_time"]
        phi_prev = state["spots"][chain[i - 1]]["phi_exit_seconds"] if i > 0 else 0

        segments.append(
            {
                "coords": route["path"],
                "type": "drive_transition",
                "label": spot.get("street", "Spot"),
                "source": route["source"],
            }
        )

        details.append(
            {
                "order": i + 1,
                "street": spot.get("street", "Spot"),
                "coords": list(spot_coords),
                "p": spot.get("p_i", 0.8),
                "phi": spot.get("phi_exit_seconds", 60),
                "drive_leg": leg_drive_time,
                "phi_prev": phi_prev,
                "arrival_time": leg_drive_time + phi_prev,
                "walk_b": state["walk_fn"](spot["coords"]),
                "exit_ref": state["drive_fn"](("spot", spot), ("node", "ref"))[
                    "travel_time"
                ],
            }
        )

        curr_coords = spot_coords

    # Exit leg: last spot → ref
    last_coords = tuple(state["spots"][chain[-1]]["coords"])
    exit_route = get_route_for_optimization(last_coords, tuple(ref), traffic_mult=1.0)
    segments.append(
        {
            "coords": exit_route["path"],
            "type": "drive_exit",
            "source": exit_route["source"],
        }
    )

    # Walk: last spot → dest (straight line)
    segments.append(
        {
            "coords": [list(last_coords), list(dest)],
            "type": "walk",
        }
    )

    return segments, details


# ─────────────────────────────────────────────────────────────────────────────
# CORE SOLVER
# ─────────────────────────────────────────────────────────────────────────────


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
):
    if not MANUAL_SPOTS:
        return {"status": "error", "message": "No spots defined."}

    weather_mult = get_weather_multipliers(start[0], start[1])

    # OSM graph augmented with spot nodes (still needed for crossings)
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

    # ── drive_fn — OSRM base + traffic multiplier ─────────────────────────
    DISTANCE_CACHE = {}

    def drive_fn(u, v):
        is_exit_leg = v[0] == "node" and v[1] == "ref"
        mult = traffic_mult_exit if is_exit_leg else traffic_mult_drive

        u_id = f"spot_{u[1]['id']}" if u[0] == "spot" else u[1]
        v_id = f"spot_{v[1]['id']}" if v[0] == "spot" else v[1]
        cache_key = f"{u_id}_to_{v_id}_mult_{mult:.3f}_{routing_mode}"

        if cache_key in DISTANCE_CACHE:
            return DISTANCE_CACHE[cache_key]

        # Resolve to (lat, lon)
        if u[0] == "node":
            u_coords = tuple(start) if u[1] == "start" else tuple(ref)
        else:
            u_coords = tuple(u[1]["coords"])

        if v[0] == "node":
            v_coords = tuple(ref) if v[1] == "ref" else tuple(start)
        else:
            v_coords = tuple(v[1]["coords"])

        # OSRM gives us real road travel time — apply multiplier on top
        route = get_route_for_optimization(u_coords, v_coords, traffic_mult=mult)

        if route["travel_time"] == float("inf"):
            result = {"travel_time": float("inf"), "turn_penalty": float("inf")}
        else:
            result = {
                "travel_time": route["travel_time"],
                "turn_penalty": route["turn_penalty"],
            }

        DISTANCE_CACHE[cache_key] = result
        return result

    # ── Restrict spots, build state ───────────────────────────────────────
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

    # ── Run algorithm ─────────────────────────────────────────────────────
    solver_class = (
        MDP_Difference
        if algo_choice == "MDP_Difference"
        else FiniteHorizonMDP
        if algo_choice == "MDP"
        else TwoStageMDP
        if algo_choice == "TwoStage"
        else HeuristicLookahead
    )
    solver = solver_class(state)
    chain, metrics, _ = solver.solve(**params)

    if not chain:
        return {"status": "error", "message": "Pathfinding failed."}

    # ── Build display segments (reuses OSRM cache) ────────────────────────
    segments, details = build_display_segments(
        chain, state, start, dest, ref, traffic_mult_drive
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


# ─────────────────────────────────────────────────────────────────────────────
# /solve ENDPOINT
# ─────────────────────────────────────────────────────────────────────────────


@app.route("/solve", methods=["POST"])
def solve():
    global _OSRM_CACHE
    _OSRM_CACHE = {}  # clear once per request, shared by Option1 + Option2

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

    # Detect main street from OSM graph (used to generate Option 2)
    route_pair = routing.get_primary_and_alternative_route(
        start,
        dest,
        custom_G=routing.G,
        street_penalty_factor=street_penalty_factor,
    )
    main_street = route_pair["main_street"]

    # Option 1: normal
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
    )

    if option1["status"] != "success":
        return jsonify(option1)

    # Option 2: penalized main street (alternative route)
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
        )
    else:
        option2 = {**option1, "routing_mode": "normal", "penalized_street": None}

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
