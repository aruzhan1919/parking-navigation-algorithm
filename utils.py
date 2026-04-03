# import numpy as np


# def calculate_metrics(chain, state):
#     """
#     Calculates the TRUE Expected Time (in seconds).

#     UPDATED LOGIC:
#     - Success Case: Arrival + Walk + Exit (No search penalty 'phi')
#     - Failure Case: Arrival + Search Penalty 'phi' (Time spent looking/failing)
#     """
#     if not chain:
#         return (999999.0, 0, 0, 0, 0)

#     spots = state["spots"]  # парковки
#     drive_fn = state["drive_fn"]  # drive time
#     walk_fn = state["walk_fn"]  # walk time to dest from parking

#     exit_mult = state.get("exit_multiplier", 1.0)

#     total_expected_time = 0.0
#     cumulative_fail_prob = 1.0
#     current_timeline_time = 0.0
#     prev_node = ("node", "start")

#     for i, idx in enumerate(chain):
#         spot = spots[idx]
#         p_i = spot.get("p_i", 0.8)
#         # phi = spot.get("phi_exit_seconds", 60)
#         phi = 0.0

#         # 1. Drive to Spot
#         drive_time = drive_fn(prev_node, ("spot", spot))["travel_time"]  # NEW
#         arrival_time = current_timeline_time + drive_time

#         # 2. Terminal Actions
#         walk_time = walk_fn(spot["coords"])

#         # 3. Exit Drive
#         raw_exit_drive = drive_fn(
#             ("spot", spot), ("node", "ref")
#         )[
#             "travel_time"
#         ]  # how long it will take to leave the parking spot and drive to the reference point C
#         drive_exit_ref = raw_exit_drive * exit_mult

#         # 4. Success Case: NO PHI. We found a spot!
#         time_if_success = arrival_time + walk_time + drive_exit_ref

#         # Add to expected value
#         total_expected_time += cumulative_fail_prob * p_i * time_if_success

#         # 5. Failure Case: INCLUDES PHI. We wasted time searching.
#         time_if_fail = arrival_time + phi
#         current_timeline_time = time_if_fail
#         cumulative_fail_prob *= 1.0 - p_i

#         prev_node = ("spot", spot)

#     # Penalty for failing the entire chain (Physical penalty estimate)
#     total_expected_time += cumulative_fail_prob * (current_timeline_time + 1800)

#     final_fail_prob = 1.0
#     for idx in chain:
#         final_fail_prob *= 1.0 - spots[idx].get("p_i", 0.8)
#     confidence = 1.0 - final_fail_prob

#     return (total_expected_time, 0, 0, 0, confidence)
def calculate_metrics(chain, state):
    """
    Returns decomposed Expected Time in seconds.
    Each factor weighted by probability of reaching that spot.
    """
    if not chain:
        return (999999.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    spots = state["spots"]
    drive_fn = state["drive_fn"]
    walk_fn = state["walk_fn"]
    exit_mult = state.get("exit_multiplier", 1.0)

    expected_drive = 0.0
    expected_walk = 0.0
    expected_exit = 0.0
    expected_cross = 0.0

    reach_prob = 1.0  # probability of still searching when arriving at spot i
    prev_node = ("node", "start")

    for idx in chain:
        spot = spots[idx]
        p_i = spot.get("p_i", 0.8)

        drive_time = drive_fn(prev_node, ("spot", spot))["travel_time"]
        walk_time = walk_fn(spot["coords"])
        exit_time = drive_fn(("spot", spot), ("node", "ref"))["travel_time"] * exit_mult
        cross_penalty = spot.get("cross_penalty", 0.0)

        # drive is always paid when you reach this spot
        expected_drive += reach_prob * drive_time

        # walk, exit, cross only paid if spot is free
        expected_walk += reach_prob * p_i * walk_time
        expected_exit += reach_prob * p_i * exit_time
        expected_cross += reach_prob * p_i * cross_penalty

        reach_prob *= 1.0 - p_i
        prev_node = ("spot", spot)

    # penalty if entire chain fails
    expected_drive += reach_prob * 1800.0

    expected_total = expected_drive + expected_walk + expected_exit + expected_cross
    success_prob = 1.0 - reach_prob

    return (
        expected_total,  # [0] total expected time in seconds
        expected_drive,  # [1] expected driving time
        expected_walk,  # [2] expected walking time
        expected_exit,  # [3] expected exit time
        expected_cross,  # [4] expected crossing penalty
        success_prob,  # [5] probability of finding a spot
    )


# def calculate_metrics(chain, state):
#     if not chain:
#         return {
#             "expected_total_sec": 999999.0,
#             "expected_drive_sec": 0.0,
#             "expected_walk_sec": 0.0,
#             "expected_exit_sec": 0.0,
#             "expected_cross_sec": 0.0,
#             "expected_arrival_turn_sec": 0.0,
#             "expected_exit_turn_sec": 0.0,
#             "success_probability": 0.0,
#         }

#     spots = state["spots"]
#     drive_fn = state["drive_fn"]
#     walk_fn = state["walk_fn"]
#     exit_mult = state.get("exit_multiplier", 1.0)

#     expected_drive = 0.0
#     expected_walk = 0.0
#     expected_exit = 0.0
#     expected_cross = 0.0
#     expected_arrival_turn = 0.0
#     expected_exit_turn = 0.0

#     reach_prob = 1.0
#     prev_node = ("node", "start")

#     for idx in chain:
#         spot = spots[idx]
#         p_i = spot.get("p_i", 0.8)

#         arrival = drive_fn(prev_node, ("spot", spot))
#         exit_r = drive_fn(("spot", spot), ("node", "ref"))

#         drive_time = arrival["travel_time"]
#         arrival_turn = arrival["turn_penalty"]
#         walk_time = walk_fn(spot["coords"])
#         exit_time = exit_r["travel_time"] * exit_mult
#         exit_turn = exit_r["turn_penalty"]
#         cross_penalty = spot.get("cross_penalty", 0.0)

#         expected_drive += reach_prob * drive_time
#         expected_arrival_turn += reach_prob * arrival_turn

#         expected_walk += reach_prob * p_i * walk_time
#         expected_exit += reach_prob * p_i * exit_time
#         expected_exit_turn += reach_prob * p_i * exit_turn
#         expected_cross += reach_prob * p_i * cross_penalty

#         reach_prob *= 1.0 - p_i
#         prev_node = ("spot", spot)

#     expected_drive += reach_prob * 1800.0

#     expected_total = (
#         expected_drive
#         + expected_walk
#         + expected_exit
#         + expected_cross
#         + expected_arrival_turn
#         + expected_exit_turn
#     )

#     return {
#         "expected_total_sec": expected_total,
#         "expected_drive_sec": expected_drive,
#         "expected_walk_sec": expected_walk,
#         "expected_exit_sec": expected_exit,
#         "expected_cross_sec": expected_cross,
#         "expected_arrival_turn_sec": expected_arrival_turn,
#         "expected_exit_turn_sec": expected_exit_turn,
#         "success_probability": 1.0 - reach_prob,
#     }


"""
The logic per spot:
```
reach_prob * drive_time          ← you drove there regardless (free or occupied)
reach_prob * p_i * walk_time     ← only if spot was free
reach_prob * p_i * exit_time     ← only if spot was free
reach_prob * p_i * cross_penalty ← only if spot was free

reach_prob updates: *= (1 - p_i) ← next spot only reached if this one failed
"""
