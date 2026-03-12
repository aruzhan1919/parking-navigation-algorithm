import numpy as np


def calculate_metrics(chain, state):
    """
    Calculates the TRUE expected time (in seconds) using the SAME drive_fn as optimization.

    - Success: Arrival + Walk + Exit
    - Failure after spot: Arrival + phi (search penalty)
    - Total failure (all spots): Arrival at last + exit drive + safety buffer
    """
    if not chain:
        return (999999.0, 0, 0, 0, 0.0)

    spots = state["spots"]
    drive_fn = state["drive_fn"]  # ← this is the important one: from StateAdapter
    walk_fn = state["walk_fn"]
    exit_mult = state.get("exit_multiplier", 1.0)

    total_expected_time = 0.0
    cumulative_fail_prob = 1.0
    current_timeline_time = 0.0
    prev_node = ("node", "start")

    for i, idx in enumerate(chain):
        spot = spots[idx]
        p_i = spot.get("p_i", 0.8)
        phi = spot.get("phi_exit_seconds", 60)

        # 1. Drive to this spot (using real drive_fn with turns + geometry + traffic)
        drive_time = drive_fn(prev_node, ("spot", spot))
        arrival_time = current_timeline_time + drive_time

        # 2. Walk time from spot to destination
        walk_time = walk_fn(spot["coords"])

        # 3. Exit drive: spot → ref (with multiplier)
        exit_drive_raw = drive_fn(("spot", spot), ("node", "ref"))
        drive_exit_ref = exit_drive_raw * exit_mult

        # Success case: we found parking here
        time_if_success = arrival_time + walk_time + drive_exit_ref

        # Contribute to expected value
        total_expected_time += cumulative_fail_prob * p_i * time_if_success

        # Failure case: pay search penalty, continue to next
        time_if_fail = arrival_time + phi
        current_timeline_time = time_if_fail
        cumulative_fail_prob *= 1.0 - p_i

        prev_node = ("spot", spot)

    # If ALL spots failed → still need to leave the area
    last_spot = spots[chain[-1]]
    final_exit_drive = drive_fn(("spot", last_spot), ("node", "ref")) * exit_mult
    failure_buffer = 600.0  # e.g. 10 min extra frustration/safety

    total_expected_time += cumulative_fail_prob * (
        current_timeline_time + final_exit_drive + failure_buffer
    )

    # Confidence = probability we found at least one spot
    final_fail_prob = cumulative_fail_prob  # after last multiplication
    confidence = 1.0 - final_fail_prob

    # For compatibility with your old return format
    return (
        round(total_expected_time, 1),
        0,  # placeholder (old values)
        0,
        0,
        round(confidence * 100, 1),  # % confidence
    )
