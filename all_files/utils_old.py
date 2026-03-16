import numpy as np


def calculate_metrics(chain, state):
    """
    Calculates the TRUE Expected Time (in seconds).

    UPDATED LOGIC:
    - Success Case: Arrival + Walk + Exit (No search penalty 'phi')
    - Failure Case: Arrival + Search Penalty 'phi' (Time spent looking/failing)
    """
    if not chain:
        return (999999.0, 0, 0, 0, 0)

    spots = state["spots"]  # парковки
    drive_fn = state["drive_fn"]  # drive time
    walk_fn = state["walk_fn"]  # walk time to dest from parking

    exit_mult = state.get("exit_multiplier", 1.0)

    total_expected_time = 0.0
    cumulative_fail_prob = 1.0
    current_timeline_time = 0.0
    prev_node = ("node", "start")

    for i, idx in enumerate(chain):
        spot = spots[idx]
        p_i = spot.get("p_i", 0.8)
        phi = spot.get("phi_exit_seconds", 60)

        # 1. Drive to Spot
        drive_time = drive_fn(prev_node, ("spot", spot))
        arrival_time = current_timeline_time + drive_time

        # 2. Terminal Actions
        walk_time = walk_fn(spot["coords"])

        # 3. Exit Drive
        raw_exit_drive = drive_fn(
            ("spot", spot), ("node", "ref")
        )  # how long it will take to leave the parking spot and drive to the reference point C
        drive_exit_ref = raw_exit_drive * exit_mult

        # 4. Success Case: NO PHI. We found a spot!
        time_if_success = arrival_time + walk_time + drive_exit_ref

        # Add to expected value
        total_expected_time += cumulative_fail_prob * p_i * time_if_success

        # 5. Failure Case: INCLUDES PHI. We wasted time searching.
        time_if_fail = arrival_time + phi
        current_timeline_time = time_if_fail
        cumulative_fail_prob *= 1.0 - p_i

        prev_node = ("spot", spot)

    # Penalty for failing the entire chain (Physical penalty estimate)
    total_expected_time += cumulative_fail_prob * (current_timeline_time + 1800)

    final_fail_prob = 1.0
    for idx in chain:
        final_fail_prob *= 1.0 - spots[idx].get("p_i", 0.8)
    confidence = 1.0 - final_fail_prob

    return (total_expected_time, 0, 0, 0, confidence)
