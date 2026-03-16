from flask import Flask, request, jsonify, render_template
import os, time, json, math
import numpy as np
from jinja2 import FileSystemLoader
from datetime import datetime, timedelta
import pytz
import routing
from scenarios import SCENARIOS
from locations import LOCATIONS
from transformation_experiment_recent import StateAdapter
from algorithms_final import (
    FiniteHorizonMDP,
    HeuristicLookahead,
    MDP_Difference,
)

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
STATIC_DIR = os.path.join(BASE_DIR, "static")
MANUAL_FILE = os.path.join(BASE_DIR, "manual_spots_new.json")
DISTANCE_CACHE_FILE = os.path.join(BASE_DIR, "distance_cache.json")

if not os.path.exists(STATIC_DIR):
    os.makedirs(STATIC_DIR, exist_ok=True)

app = Flask(__name__, static_folder=STATIC_DIR, template_folder=BASE_DIR)
app.jinja_loader = FileSystemLoader(BASE_DIR)
routing.initialize_graph()


############ NEW CHANGE #################
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


############ NEW CHANGE #################
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
# DISTANCE_CACHE = load_json(DISTANCE_CACHE_FILE, {})
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


@app.route("/solve", methods=["POST"])
def solve():
    DISTANCE_CACHE.clear()
    data = request.json
    start = parse_coord(data.get("start"))
    dest = parse_coord(data.get("dest"))
    ref = parse_coord(data.get("ref"))
    exit_multiplier = float(data.get("exit_multiplier", 1.0))

    ############# NEW CHANGE ##########
    # tz = pytz.timezone("Asia/Almaty")
    # now = datetime.now(tz)
    ############# NEW CHANGE ##########
    # STAY_DURATION = {
    #     "Short Stay (10-40m)": 25,
    #     "Mid Stay (1-2h)": 90,
    #     "Evening Stay (2-4h)": 180,
    #     "Long Stay - Day (6-10h)": 480,
    #     "Long Stay - Night (8-12h)": 600,
    #     "Taxi (5-10m)": 7,
    #     "Delivery (1-4m)": 3,
    # }
    ############# NEW CHANGE ##########
    if not MANUAL_SPOTS:
        return jsonify({"status": "error", "message": "No spots defined."})

    # 1. Setup Augmented Routing
    G_aug = routing.augment_graph_with_spots(MANUAL_SPOTS)

    # 2. Resolve Scenario & Parameters
    scenario_key = data.get("scenario", "Short Stay (10-40m)")
    scenario_cfg = SCENARIOS.get(scenario_key, SCENARIOS["Short Stay (10-40m)"])
    params = scenario_cfg["params"].copy()
    traffic_mult_drive = params.get("traffic_multiplier_drive", 1.4)
    traffic_mult_exit = params.get("traffic_multiplier_exit", 1.4)
    ############# NEW CHANGE ##########
    # traffic_mult = traffic_multiplier_for_now(now)
    ############# NEW CHANGE ##########

    if scenario_key == "Custom":
        custom = data.get("custom_params", {})
        params = {
            "lambda_w": custom.get("lambda_w", 1.5),
            "lambda_e": custom.get("lambda_e", 1.0),
            "lambda_tr": custom.get("lambda_tr", 1.0),
            "lambda_turns_arr": custom.get("lambda_turns_arr", 1.0),
            "lambda_turns_exit": custom.get("lambda_turns_exit", 1.0),
            "lambda_cross": custom.get("lambda_cross", 1.0),
            "k": custom.get("k", 5),
        }
    # stay_minutes = STAY_DURATION.get(scenario_key, 30)
    # exit_time = now + timedelta(minutes=stay_minutes)
    # traffic_mult_drive = traffic_multiplier_for_time(now)
    # traffic_mult_exit = traffic_multiplier_for_time(exit_time)

    # 3. Cached Augmented Router
    # def drive_fn(u, v):
    #     ########## NEW CHANGE ##########
    #     is_exit_leg = (v[0] == "node" and v[1] == "ref")
    #     mult = traffic_mult_exit if is_exit_leg else traffic_mult_drive
    #     ########## NEW CHANGE ##########
    #     u_id = (
    #         f"spot_{u[1]['id']}" if u[0] == "spot" else f"coord_{start[0]}_{start[1]}"
    #     )
    #     v_id = f"spot_{v[1]['id']}" if v[0] == "spot" else f"coord_{ref[0]}_{ref[1]}"
    #     cache_key = f"{u_id}_to_{v_id}_{mult}" #changed from traffic_mult
    #     if cache_key in DISTANCE_CACHE:
    #         return float(DISTANCE_CACHE[cache_key])
    #     u_t = f"spot_{u[1]['id']}" if u[0] == "spot" else start
    #     v_t = f"spot_{v[1]['id']}" if v[0] == "spot" else ref
    #     res = routing.get_route(u_t, v_t, custom_G=G_aug)
    #     if not res["path"]:
    #         return jsonify(
    #             {
    #                 "status": "error",
    #                 "message": f"Route failed from {curr_origin_id} to {spot_node_id}",
    #             }
    #         )
    #     final_time = res["travel_time"] * mult # changed from traffic_mult
    #     # routes = routing.get_k_best_routes(u_t, v_t, custom_G=G_aug)
    #     # if not routes:
    #     #     return float("inf")
    #     # best_route = routes[0]  # already sorted
    #     # final_time = best_route["total_time"] * traffic_mult
    #     # routes = routing.get_k_best_routes(...)
    #     # for idx, r in enumerate(routes):
    #     #     segments.append({
    #     #         "coords": r["path"],
    #     #         "type": "drive_option",
    #     #         "rank": idx+1
    #     #     })
    #     DISTANCE_CACHE[cache_key] = final_time
    #     return final_time
    def drive_fn(u, v):
        # u, v are tuples like ("node","start") or ("node","ref") or ("spot", spot_dict)

        is_exit_leg = v[0] == "node" and v[1] == "ref"
        mult = traffic_mult_exit if is_exit_leg else traffic_mult_drive

        # ids for caching
        # u_id = f"spot_{u[1]['id']}" if u[0] == "spot" else f"node_{u[1]}"
        # v_id = f"spot_{v[1]['id']}" if v[0] == "spot" else f"node_{v[1]}"

        # cache_key = f"{u_id}_to_{v_id}_mult_{mult:.2f}"
        # if cache_key in DISTANCE_CACHE:
        #     # return float(DISTANCE_CACHE[cache_key])
        #     return DISTANCE_CACHE[cache_key]
        def node_key(x):
            if x[0] == "spot":
                return f"spot_{x[1]['id']}"
            if x[0] == "node" and x[1] == "start":
                return f"start_{start[0]:.6f}_{start[1]:.6f}"
            if x[0] == "node" and x[1] == "ref":
                return f"ref_{ref[0]:.6f}_{ref[1]:.6f}"
            return f"{x[0]}_{x[1]}"

        u_id = node_key(u)
        v_id = node_key(v)

        cache_key = f"{u_id}_to_{v_id}_mult_{mult}"

        if cache_key in DISTANCE_CACHE:
            return DISTANCE_CACHE[cache_key]

        # real routing endpoints
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

        res = routing.get_route(u_t, v_t, custom_G=G_aug)
        # if not res["path"]:
        #     return float(
        #         "inf"
        #     )  # IMPORTANT: drive_fn should return a number, not jsonify
        if not res["path"]:
            return {
                "travel_time": float("inf"),
                "turn_penalty": float("inf"),
            }

        # final_time = res["travel_time"] * mult
        result = {
            "travel_time": res["travel_time"] * mult,
            "turn_penalty": res["turn_penalty"] * mult,
        }
        final_time = result["travel_time"]
        DISTANCE_CACHE[cache_key] = result
        return result

    adapter = StateAdapter(
        start, dest, ref, MANUAL_SPOTS, traffic_multiplier=traffic_mult_drive
    )
    state = adapter.get_state()
    state["drive_fn"] = drive_fn
    state["exit_multiplier"] = exit_multiplier
    state["G_aug"] = G_aug
    # Choose Solver
    algo_choice = data.get("algo", "MDP_Difference")
    solver_class = (
        MDP_Difference
        if algo_choice == "MDP_Difference"
        else (FiniteHorizonMDP if algo_choice == "MDP" else HeuristicLookahead)
    )
    solver = solver_class(state)
    chain, metrics, _ = solver.solve(**params)
    # chains, metrics_list, _ = solver.solve(**params)

    if not chain:
        return jsonify({"status": "error", "message": "Pathfinding failed."})

    # 4. Persistence & UI Breakdown
    save_json(DISTANCE_CACHE_FILE, DISTANCE_CACHE)

    segments = []
    details = []
    curr_origin_id = start

    for i, idx in enumerate(chain):
        spot = state["spots"][idx]
        spot_node_id = f"spot_{spot['id']}"

        # Physical drive for this leg
        res = routing.get_route(curr_origin_id, spot_node_id, custom_G=G_aug)
        leg_drive_time = res["travel_time"] * traffic_mult_drive  #### changed

        # Search penalty from the PREVIOUS failed attempt
        phi_prev = state["spots"][chain[i - 1]]["phi_exit_seconds"] if i > 0 else 0

        # Transition = Drive + Phi_prev
        total_arrival_time = leg_drive_time + phi_prev

        segments.append(
            {"coords": res["path"], "type": "drive_transition", "label": spot["street"]}
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

    # Add final exit segments
    last_spot_id = f"spot_{state['spots'][chain[-1]]['id']}"
    res_ex = routing.get_route(last_spot_id, ref, custom_G=G_aug)
    segments.append({"coords": res_ex["path"], "type": "drive_exit"})
    segments.append(
        {"coords": [state["spots"][chain[-1]]["coords"], dest], "type": "walk"}
    )
    ######### CHANGED ##################
    # if not chains:
    #     return jsonify({"status": "error", "message": "Pathfinding failed."})

    # all_solutions = []

    # for solution_idx, chain in enumerate(chains):
    #     segments = []
    #     details = []
    #     curr_origin_id = start

    #     for i, idx in enumerate(chain):
    #         spot = state["spots"][idx]
    #         spot_node_id = f"spot_{spot['id']}"

    #         res = routing.get_route(curr_origin_id, spot_node_id, custom_G=G_aug)
    #         leg_drive_time = res["travel_time"] * traffic_mult

    #         phi_prev = state["spots"][chain[i - 1]]["phi_exit_seconds"] if i > 0 else 0
    #         total_arrival_time = leg_drive_time + phi_prev

    #         segments.append(
    #             {
    #                 "coords": res["path"],
    #                 "type": "drive_transition",
    #                 "label": spot["street"],
    #                 "solution": solution_idx,
    #             }
    #         )

    #         details.append(
    #             {
    #                 "order": i + 1,
    #                 "street": spot.get("street", "Spot"),
    #                 "coords": spot["coords"],
    #                 "p": spot.get("p_i", 0.8),
    #                 "phi": spot.get("phi_exit_seconds", 60),
    #                 "drive_leg": leg_drive_time,
    #                 "phi_prev": phi_prev,
    #                 "arrival_time": total_arrival_time,
    #                 "walk_b": state["walk_fn"](spot["coords"]),
    #                 "exit_ref": state["drive_fn"](("spot", spot), ("node", "ref")),
    #             }
    #         )

    #         curr_origin_id = spot_node_id

    #     # Final exit
    #     last_spot_id = f"spot_{state['spots'][chain[-1]]['id']}"
    #     res_ex = routing.get_route(last_spot_id, ref, custom_G=G_aug)

    #     segments.append(
    #         {"coords": res_ex["path"], "type": "drive_exit", "solution": solution_idx}
    #     )

    #     segments.append(
    #         {
    #             "coords": [state["spots"][chain[-1]]["coords"], dest],
    #             "type": "walk",
    #             "solution": solution_idx,
    #         }
    #     )

    # all_solutions.append(
    #     {
    #         "chain": chain,
    #         "metrics": metrics_list[solution_idx],
    #         "segments": segments,
    #         "details": details,
    #     }
    # )
    # all_solutions.append(
    #     {
    #         "id": solution_idx,
    #         "label": f"Option {solution_idx + 1}",
    #         "chain": chain,
    #         "metrics": metrics_list[solution_idx],
    #         "segments": segments,
    #         "details": details,
    #     }
    # )
    ######### CHANGED ##################
    return jsonify(
        {
            "status": "success",
            "metrics": list(metrics)
            + [1.0 - np.prod([1 - state["spots"][idx]["p_i"] for idx in chain])],
            "details": details,
            "segments": segments,
        }
    )
    # return jsonify({"status": "success", "solutions": all_solutions})


if __name__ == "__main__":
    app.run(debug=True, port=5001)
