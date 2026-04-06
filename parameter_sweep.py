import os
import json
import itertools
import pandas as pd
import numpy as np

import routing
from transformation import StateAdapter
from algorithms import MDP_Difference, FiniteHorizonMDP, HeuristicLookahead
from utils import calculate_metrics  # or your updated metrics function

# =========================================================
# CONFIG
# =========================================================

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MANUAL_FILE = os.path.join(BASE_DIR, "manual_spots_new.json")
OUTPUT_DIR = os.path.join(BASE_DIR, "sweep_outputs")
os.makedirs(OUTPUT_DIR, exist_ok=True)

ALGO_CHOICE = "MDP_Difference"  # or "MDP", or "Heuristic"
EXIT_MULTIPLIER = 1.0

# Choose ONE scenario logic at a time by defining parameter ranges here
PARAM_GRID = {
    "lambda_w": [5.0, 10.0, 20.0],
    "lambda_e": [8.0, 15.0, 25.0],
    "lambda_tr": [1.0, 5.0, 9.0, 15],
    "lambda_turns_arr": [0.0, 2.0, 5.0],
    "lambda_turns_exit": [5.0, 8.0, 15],
    "lambda_cross": [1.0, 2.0, 4.0],
    "k": [7],  # keep fixed first
}

# =========================================================
# LOCATIONS
# =========================================================

N = [51.133696570341655, 71.43163913937695]
S0 = [51.11026057362518, 71.42052168109727]
E = [51.12521564839819, 71.46731192666734]
W0 = [51.126580141666295, 71.40707493653436]

RITZ_CARLTON = [51.124765685067246, 71.43210200919789]
ABU_DHABI_1 = [51.12143227387381, 71.42878515268714]
ABU_DHABI_2 = [51.12172304520112, 71.42652738082138]
MEGA = [51.08672776951535, 71.41118884135531]
KHAN_SHATYR = [51.132599389420065, 71.40672935111475]
VOKZAL = [51.19034801714996, 71.4070625470186]

DEST = RITZ_CARLTON
S = MEGA
W = KHAN_SHATYR

LOCATIONS = {
    # "Set 1: N-N": {"start": N, "dest": DEST, "ref": N},
    # "Set 2: N-E": {"start": N, "dest": DEST, "ref": E},
    # "Set 3: N-S": {"start": N, "dest": DEST, "ref": S},
    # "Set 4: N-W": {"start": N, "dest": DEST, "ref": W},
    # "Set 5: E-E": {"start": E, "dest": DEST, "ref": E},
    # "Set 6: E-S": {"start": E, "dest": DEST, "ref": S},
    # "Set 7: E-W": {"start": E, "dest": DEST, "ref": W},
    # "Set 8: E-N": {"start": E, "dest": DEST, "ref": N},
    # "Set 9: S-S": {"start": S, "dest": DEST, "ref": S},
    "Set 10: S-W": {"start": S, "dest": DEST, "ref": W},
    # "Set 11: S-N": {"start": S, "dest": DEST, "ref": N},
    # "Set 12: S-E": {"start": S, "dest": DEST, "ref": E},
    # "Set 13: W-W": {"start": W, "dest": DEST, "ref": W},
    "Set 14: W-N": {"start": W, "dest": DEST, "ref": N},
    #     "Set 15: W-E": {"start": W, "dest": DEST, "ref": E},
    #     "Set 16: W-S": {"start": W, "dest": DEST, "ref": S},
}

# =========================================================
# HELPERS
# =========================================================


def load_json(path, default):
    if os.path.exists(path):
        try:
            with open(path, "r") as f:
                return json.load(f)
        except Exception:
            return default
    return default


def restrict_spots_by_walk_time(spots, dest, limit=30):
    from geopy.distance import geodesic

    ranked = sorted(spots, key=lambda s: geodesic(s["coords"], dest).meters / 1.3)
    return ranked[:limit]


def node_key(x, start, ref):
    if x[0] == "spot":
        return f"spot_{x[1]['id']}"
    if x[0] == "node" and x[1] == "start":
        return f"start_{start[0]:.6f}_{start[1]:.6f}"
    if x[0] == "node" and x[1] == "ref":
        return f"ref_{ref[0]:.6f}_{ref[1]:.6f}"
    return f"{x[0]}_{x[1]}"


def build_drive_fn(start, ref, G_used, traffic_mult_drive=1.0, traffic_mult_exit=1.0):
    distance_cache = {}

    def drive_fn(u, v):
        is_exit_leg = v[0] == "node" and v[1] == "ref"
        mult = traffic_mult_exit if is_exit_leg else traffic_mult_drive

        u_id = node_key(u, start, ref)
        v_id = node_key(v, start, ref)
        cache_key = f"{u_id}_to_{v_id}_mult_{mult}"

        if cache_key in distance_cache:
            return distance_cache[cache_key]

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

        res = routing.get_route(
            u_t, v_t, custom_G=G_used, twogis_display=False
        )

        if not res["nodes"]:
            result = {
                "travel_time": float("inf"),
                "turn_penalty": float("inf"),
            }
            distance_cache[cache_key] = result
            return result

        result = {
            "travel_time": res["travel_time"] * mult,
            "turn_penalty": res["turn_penalty"] * mult,
        }
        distance_cache[cache_key] = result
        return result

    return drive_fn


def choose_solver(algo_choice, state):
    if algo_choice == "MDP_Difference":
        return MDP_Difference(state)
    if algo_choice == "MDP":
        return FiniteHorizonMDP(state)
    return HeuristicLookahead(state)


def safe_name(name: str) -> str:
    return name.replace(":", "").replace(" ", "_").replace("/", "_").replace("-", "_")


def metrics_to_dict(metrics):
    # If your updated utils now returns dict, just return metrics
    if isinstance(metrics, dict):
        return metrics

    # Old tuple fallback
    return {
        "expected_total_sec": metrics[0],
        "expected_drive_sec": metrics[1],
        "expected_walk_sec": metrics[2],
        "expected_exit_sec": metrics[3],
        "expected_cross_sec": metrics[4],
        "success_probability": metrics[5],
    }


def chain_to_ids(chain, spots):
    return [spots[idx]["id"] for idx in chain]


# =========================================================
# MAIN SWEEP
# =========================================================


def main():
    routing.initialize_graph()

    manual_spots = load_json(MANUAL_FILE, [])
    if not manual_spots:
        raise ValueError("No spots found in manual_spots_new.json")

    # Build parameter combinations
    keys = list(PARAM_GRID.keys())
    values = [PARAM_GRID[k] for k in keys]
    param_sets = [dict(zip(keys, combo)) for combo in itertools.product(*values)]

    print(f"Total parameter sets: {len(param_sets)}")
    print(f"Total location sets: {len(LOCATIONS)}")

    for loc_name, loc_data in LOCATIONS.items():
        start = tuple(loc_data["start"])
        dest = tuple(loc_data["dest"])
        ref = tuple(loc_data["ref"])

        print(f"\nRunning {loc_name}")

        G_aug = routing.augment_graph_with_spots(manual_spots)
        restricted_spots = restrict_spots_by_walk_time(manual_spots, dest, limit=30)

        adapter = StateAdapter(
            start=start,
            dest=dest,
            ref=ref,
            spots=restricted_spots,
            traffic_multiplier=1.0,
        )

        state = adapter.get_state()
        state["G_aug"] = G_aug
        state["exit_multiplier"] = EXIT_MULTIPLIER
        state["drive_fn"] = build_drive_fn(
            start=start,
            ref=ref,
            G_used=G_aug,
            traffic_mult_drive=1.0,
            traffic_mult_exit=1.0,
        )

        rows = []

        for idx, params in enumerate(param_sets, start=1):
            solver = choose_solver(ALGO_CHOICE, state)
            chain, metrics, _ = solver.solve(**params)

            metrics_dict = metrics_to_dict(metrics)

            row = {
                "location_set": loc_name,
                "start": start,
                "dest": dest,
                "ref": ref,
                "algo": ALGO_CHOICE,
                **params,
                **metrics_dict,
                "chain_indices": chain,
                "chain_spot_ids": chain_to_ids(chain, state["spots"]) if chain else [],
                "chain_length": len(chain),
            }

            rows.append(row)

            if idx % 20 == 0 or idx == len(param_sets):
                print(f"  done {idx}/{len(param_sets)}")

        df = pd.DataFrame(rows)
        df = df.sort_values(
            by=["expected_total_sec", "success_probability"], ascending=[True, False]
        )

        out_path = os.path.join(OUTPUT_DIR, f"{safe_name(loc_name)}.csv")
        df.to_csv(out_path, index=False)
        print(f"Saved: {out_path}")

    print("\nAll 16 tables created.")


if __name__ == "__main__":
    main()
