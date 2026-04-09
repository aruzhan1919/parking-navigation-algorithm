"""
utils.py
========
Utility functions for metrics calculation.
"""


def calculate_metrics(chain, state):
    """
    Calculate decomposed expected time metrics for a parking chain.

    Uses probabilistic weighting: each cost component is multiplied by
    the probability of reaching that spot in the chain.

    Args:
        chain: list of spot indices representing the parking chain
        state: dict containing spots, drive_fn, walk_fn, exit_multiplier

    Returns:
        tuple of 6 floats:
            [0] expected_total: total expected time (seconds)
            [1] expected_drive: expected driving time
            [2] expected_walk: expected walking time
            [3] expected_exit: expected exit time
            [4] expected_cross: expected crossing penalty
            [5] success_prob: probability of finding at least one free spot
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

        # Drive time is always paid when reaching this spot
        drive_time = drive_fn(prev_node, ("spot", spot))["travel_time"]
        expected_drive += reach_prob * drive_time

        # Walk, exit, and crossing penalties only paid if spot is free
        walk_time = walk_fn(spot["coords"])
        exit_time = drive_fn(("spot", spot), ("node", "ref"))["travel_time"] * exit_mult
        cross_penalty = spot.get("cross_penalty", 0.0)

        expected_walk += reach_prob * p_i * walk_time
        expected_exit += reach_prob * p_i * exit_time
        expected_cross += reach_prob * p_i * cross_penalty

        # Update probability for next iteration
        reach_prob *= 1.0 - p_i
        prev_node = ("spot", spot)

    # Penalty if entire chain fails (30 min search elsewhere)
    expected_drive += reach_prob * 1800.0

    expected_total = expected_drive + expected_walk + expected_exit + expected_cross
    success_prob = 1.0 - reach_prob

    return (
        expected_total,
        expected_drive,
        expected_walk,
        expected_exit,
        expected_cross,
        success_prob,
    )
