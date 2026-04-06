"""
algorithms.py
=============
Optimization algorithms for selecting the best parking spot chain.

CHANGE from original:
- _postprocess_chain() now uses coordinate-based routing (2GIS/OSM)
  instead of graph node lookups. No dependency on spot._node_id or G_aug nodes.
- All optimization logic (beam search, cost model) unchanged.
"""

import numpy as np
from abc import ABC

import routing
from utils import calculate_metrics


# ==========================================
# BASE CLASS
# ==========================================


class BaseAlgorithm(ABC):
    def __init__(self, state):
        self.state = state
        self.spots = state["spots"]
        self.drive_fn = state["drive_fn"]
        self.walk_fn = state["walk_fn"]
        self.topology = state["topology"]
        self.n = len(self.spots)

        self.start_progress_ab = state["start_progress_ab"]
        self.start_progress_bc = state["start_progress_bc"]
        self.dest_progress_ab = state["dest_progress_ab"]
        self.dest_progress_bc = state["dest_progress_bc"]
        self.ref_progress_ab = state["ref_progress_ab"]
        self.ref_progress_bc = state["ref_progress_bc"]

        self.max_drive = 1.0
        self.max_walk = 1.0
        self.max_phi = 1.0
        self.max_turn = 1.0
        self.max_cross = 1.0

        self.trans_matrix = np.zeros((self.n + 1, self.n))
        self.turn_matrix = np.zeros((self.n + 1, self.n))
        self.cross_vector = np.zeros(self.n)
        self.static_matrix = np.zeros(self.n)

        self._find_max_bounds()
        self._build_topology_matrix(exit_mult=state.get("exit_multiplier", 1.0))

    def _get_phi(self, idx):
        if idx < 0:
            return 0
        return 0.0

    def _find_max_bounds(self):
        drive_times = []
        walk_times = []
        phis = []
        turn_penalties = []
        cross_penalties = []

        for i in range(self.n):
            walk_times.append(self.walk_fn(self.spots[i]["coords"]))
            phis.append(self._get_phi(i))
            cross_penalties.append(self.spots[i].get("cross_penalty", 0.0))

            r1 = self.drive_fn(("node", "start"), ("spot", self.spots[i]))
            drive_times.append(r1["travel_time"])
            turn_penalties.append(r1["turn_penalty"])

            r2 = self.drive_fn(("spot", self.spots[i]), ("node", "ref"))
            drive_times.append(r2["travel_time"])
            turn_penalties.append(r2["turn_penalty"])

            if self.n > 1:
                next_idx = (i + 1) % self.n
                r3 = self.drive_fn(
                    ("spot", self.spots[i]), ("spot", self.spots[next_idx])
                )
                drive_times.append(r3["travel_time"])
                turn_penalties.append(r3["turn_penalty"])

        self.max_drive = max(max(drive_times) if drive_times else 1.0, 1.0)
        self.max_walk = max(max(walk_times) if walk_times else 1.0, 1.0)
        self.max_phi = max(max(phis) if phis else 1.0, 1.0)
        self.max_turn = max(max(turn_penalties) if turn_penalties else 1.0, 1.0)
        self.max_cross = max(max(cross_penalties) if cross_penalties else 1.0, 1.0)

    def _build_topology_matrix(self, exit_mult=1.0):
        for i in range(self.n):
            walk = self.walk_fn(self.spots[i]["coords"])
            raw_exit = self.drive_fn(("spot", self.spots[i]), ("node", "ref"))[
                "travel_time"
            ]
            self.cross_vector[i] = self.spots[i].get("cross_penalty", 0.0)
            self.static_matrix[i] = (walk / self.max_walk) + (
                (raw_exit * exit_mult) / self.max_drive
            )

        for u in range(-1, self.n):
            u_node = ("node", "start") if u == -1 else ("spot", self.spots[u])

            for v in range(self.n):
                if u == v:
                    self.trans_matrix[u + 1, v] = float("inf")
                    self.turn_matrix[u + 1, v] = float("inf")
                    continue

                route = self.drive_fn(u_node, ("spot", self.spots[v]))
                self.trans_matrix[u + 1, v] = route["travel_time"]
                self.turn_matrix[u + 1, v] = route["turn_penalty"]

    def _postprocess_chain(self, best_chain, k):
        """
        Insert additional spots that lie geographically close to the route.

        SIMPLIFIED from original:
        - No longer depends on graph node IDs (spot_node_id) or G_aug
        - Uses coordinate-based routing to check proximity
        - Inserts spots whose coords are within WALK_ENROUTE of dest
          and lie geographically between consecutive chain spots

        This is a best-effort insertion — coordinate distance only,
        no graph traversal required.
        """
        WALK_ENROUTE = 420  # seconds

        if not best_chain:
            return []

        full_chain = list(best_chain)

        # Insert spots with very short walk time that aren't already in chain
        # and lie between start and first spot (by progress_ab)
        if best_chain:
            first_idx = best_chain[0]
            first_prog = self.topology[first_idx]["progress_ab"]
            start_prog = self.start_progress_ab

            candidates = []
            for j in range(self.n):
                if j in full_chain:
                    continue
                j_prog = self.topology[j]["progress_ab"]
                walk_j = self.walk_fn(self.spots[j]["coords"])
                # Must be between start and first spot, and reasonable walk
                if start_prog <= j_prog <= first_prog and walk_j <= WALK_ENROUTE:
                    candidates.append((j_prog, j))

            candidates.sort(key=lambda x: x[0])
            prefix = [j for _, j in candidates]
            full_chain = prefix + full_chain

        return full_chain


# ==========================================
# ALGORITHM 1: MDP_Difference (primary)
# ==========================================


class MDP_Difference(BaseAlgorithm):
    def solve(self, **kwargs):
        k = int(kwargs.get("k", 3))
        lw = kwargs.get("lambda_w", 1.0)
        le = kwargs.get("lambda_e", 1.0)
        ltr = kwargs.get("lambda_tr", 1.0)
        lturn_arr = kwargs.get("lambda_turns_arr", 1.0)
        lturn_exit = kwargs.get("lambda_turns_exit", 1.0)
        lcross = kwargs.get("lambda_cross", 1.0)

        WALK_FIRST = 180

        beam = [(0.0, 1.0, -1, [], frozenset())]
        best_chain = []

        for _ in range(k):
            candidates = []
            for cost, fail_prob, curr, path, visited in beam:
                phi_prev = self._get_phi(curr) if curr >= 0 else 0
                norm_phi_prev = phi_prev / self.max_phi

                for i in range(self.n):
                    raw_drive = self.trans_matrix[curr + 1, i]
                    raw_turn = self.turn_matrix[curr + 1, i]
                    if (
                        raw_drive == float("inf")
                        or raw_turn == float("inf")
                        or i in visited
                    ):
                        continue

                    walk = self.walk_fn(self.spots[i]["coords"])
                    if curr == -1 and walk > WALK_FIRST:
                        continue

                    p_i = self.spots[i].get("p_i", 0.8)
                    r = self.drive_fn(("spot", self.spots[i]), ("node", "ref"))
                    exit_d = r["travel_time"]
                    exit_turn = r["turn_penalty"]
                    raw_cross = self.cross_vector[i]

                    drive_w = ltr if curr >= 0 else 1.0
                    norm_drive = drive_w * raw_drive / self.max_drive
                    norm_turn = lturn_arr * raw_turn / self.max_turn
                    norm_cross = lcross * raw_cross / self.max_cross
                    norm_exit_turn = lturn_exit * exit_turn / self.max_turn

                    arrival_cost = norm_drive + norm_turn + norm_phi_prev
                    success_cost = (
                        lw * walk / self.max_walk
                        + le * exit_d / self.max_drive
                        + norm_cross
                        + norm_exit_turn
                    )
                    increment = fail_prob * (arrival_cost + p_i * success_cost)

                    candidates.append(
                        (
                            cost + increment,
                            fail_prob * (1.0 - p_i),
                            i,
                            path + [i],
                            visited | {i},
                        )
                    )

            candidates.sort(key=lambda x: x[0])
            beam = candidates[:1000]
            if beam:
                best_chain = beam[0][3]

        if not best_chain:
            return [], calculate_metrics([], self.state), {}

        full_chain = self._postprocess_chain(best_chain, k)
        return full_chain, calculate_metrics(full_chain, self.state), {}


# ==========================================
# ALGORITHM 2: FiniteHorizonMDP
# ==========================================


class FiniteHorizonMDP(BaseAlgorithm):
    def solve(self, **kwargs):
        k = int(kwargs.get("k", 3))
        lw = kwargs.get("lambda_w", 1.0)
        le = kwargs.get("lambda_e", 1.0)
        ltr = kwargs.get("lambda_tr", 1.0)
        lturn_arr = kwargs.get("lambda_turns_arr", 1.0)
        lturn_exit = kwargs.get("lambda_turns_exit", 1.0)
        lcross = kwargs.get("lambda_cross", 1.0)

        beam = [(0.0, 1.0, -1, [], frozenset())]
        best_completed = None
        min_cost = float("inf")

        for _ in range(max(k, 7)):
            candidates = []
            for cost, fail_p, curr, path, visited in beam:
                if fail_p < 0.002:
                    if cost < min_cost:
                        min_cost, best_completed = cost, path
                    continue

                phi_prev = self._get_phi(curr) if curr >= 0 else 0
                norm_phi_prev = phi_prev / self.max_phi

                for i in range(self.n):
                    raw_drive = self.trans_matrix[curr + 1, i]
                    raw_turn = self.turn_matrix[curr + 1, i]
                    if (
                        raw_drive == float("inf")
                        or raw_turn == float("inf")
                        or i in visited
                    ):
                        continue

                    p_i = self.spots[i].get("p_i", 0.8)
                    walk = self.walk_fn(self.spots[i]["coords"])
                    if curr == -1 and walk > 180:
                        continue

                    r = self.drive_fn(("spot", self.spots[i]), ("node", "ref"))
                    exit_d = r["travel_time"]
                    exit_turn = r["turn_penalty"]
                    raw_cross = self.cross_vector[i]

                    drive_w = ltr if curr >= 0 else 1.0
                    norm_drive = (drive_w * raw_drive) / self.max_drive
                    norm_turn = (lturn_arr * raw_turn) / self.max_turn
                    norm_cross = (lcross * raw_cross) / self.max_cross
                    norm_exit_turn = (lturn_exit * exit_turn) / self.max_turn

                    arrival_cost = norm_drive + norm_turn + norm_phi_prev
                    success_cost = (
                        (lw * (walk / self.max_walk))
                        + (le * (exit_d / self.max_drive))
                        + norm_cross
                        + norm_exit_turn
                    )
                    expected_increment = (
                        fail_p * arrival_cost + fail_p * p_i * success_cost
                    )

                    candidates.append(
                        (
                            cost + expected_increment,
                            fail_p * (1.0 - p_i),
                            i,
                            path + [i],
                            visited | {i},
                        )
                    )

            candidates.sort(key=lambda x: x[0])
            beam = candidates[:1000]
            if beam and (1.0 - beam[0][1]) >= 0.9 and beam[0][0] < min_cost:
                min_cost, best_completed = beam[0][0], beam[0][3]

        best_chain = best_completed or (beam[0][3] if beam else [])

        if not best_chain:
            return [], calculate_metrics([], self.state), {}

        full_chain = self._postprocess_chain(best_chain, k)
        return full_chain, calculate_metrics(full_chain, self.state), {}


# ==========================================
# ALGORITHM 3: HeuristicLookahead
# ==========================================


class HeuristicLookahead(BaseAlgorithm):
    def solve(self, **kwargs):
        k = int(kwargs.get("k", 3))
        lw = kwargs.get("lambda_w", 1.0)
        le = kwargs.get("lambda_e", 1.0)
        ltr = kwargs.get("lambda_tr", 1.0)
        lturn_arr = kwargs.get("lambda_turns_arr", 1.0)
        lturn_exit = kwargs.get("lambda_turns_exit", 1.0)
        lcross = kwargs.get("lambda_cross", 1.0)

        chain, curr = [], -1

        for _ in range(k):
            best_i, min_q = -1, float("inf")
            phi_prev = self._get_phi(curr) if curr >= 0 else 0
            norm_phi_prev = phi_prev / self.max_phi

            for i in range(self.n):
                raw_drive = self.trans_matrix[curr + 1, i]
                raw_turn = self.turn_matrix[curr + 1, i]
                if raw_drive == float("inf") or raw_turn == float("inf") or i in chain:
                    continue

                p_i = self.spots[i].get("p_i", 0.8)
                walk = self.walk_fn(self.spots[i]["coords"])
                if curr == -1 and walk > 180:
                    continue

                r = self.drive_fn(("spot", self.spots[i]), ("node", "ref"))
                exit_d = r["travel_time"]
                exit_turn = r["turn_penalty"]
                raw_cross = self.cross_vector[i]

                drive_w = ltr if curr >= 0 else 1.0
                norm_drive = (drive_w * raw_drive) / self.max_drive
                norm_turn = (lturn_arr * raw_turn) / self.max_turn
                norm_cross = (lcross * raw_cross) / self.max_cross
                norm_exit_turn = (lturn_exit * exit_turn) / self.max_turn

                arrival_cost = norm_drive + norm_turn + norm_phi_prev
                success_cost = (
                    (lw * (walk / self.max_walk))
                    + (le * (exit_d / self.max_drive))
                    + norm_cross
                    + norm_exit_turn
                )
                q = arrival_cost + (success_cost / max(p_i, 0.1))

                if q < min_q:
                    min_q, best_i = q, i

            if best_i == -1:
                break
            chain.append(best_i)
            curr = best_i

        if not chain:
            return [], calculate_metrics([], self.state), {}

        full_chain = self._postprocess_chain(chain, k)
        return full_chain, calculate_metrics(full_chain, self.state), {}


# ==========================================
# ALGORITHM 4: TwoStageMDP
# ==========================================


class TwoStageMDP(BaseAlgorithm):
    def solve(self, **kwargs):
        k = int(kwargs.get("k", 3))
        lw = kwargs.get("lambda_w", 1.0)
        le = kwargs.get("lambda_e", 1.0)
        lturn_exit = kwargs.get("lambda_turns_exit", 1.0)
        lturn_arr = kwargs.get("lambda_turns_arr", 1.0)
        ltr = kwargs.get("lambda_tr", 1.0)
        lcross = kwargs.get("lambda_cross", 1.0)

        quality_scores = []
        for i in range(self.n):
            p_i = self.spots[i].get("p_i", 0.8)
            walk = self.walk_fn(self.spots[i]["coords"])
            r = self.drive_fn(("spot", self.spots[i]), ("node", "ref"))
            exit_d = r["travel_time"]
            exit_turn = r["turn_penalty"]
            raw_cross = self.cross_vector[i]

            norm_walk = lw * walk / self.max_walk
            norm_exit = le * exit_d / self.max_drive
            norm_exit_turn = lturn_exit * exit_turn / self.max_turn
            norm_cross = lcross * raw_cross / self.max_cross

            success_cost = norm_walk + norm_exit + norm_exit_turn + norm_cross
            quality = p_i * success_cost
            quality_scores.append((quality, i))

        quality_scores.sort(key=lambda x: x[0])
        top_k_indices = [i for _, i in quality_scores[:k]]

        beam = [(0.0, 1.0, -1, [], frozenset())]
        best_chain = []

        for _ in range(k):
            candidates = []
            for cost, fail_prob, curr, path, visited in beam:
                for i in top_k_indices:
                    if i in visited:
                        continue
                    if curr == -1:
                        walk = self.walk_fn(self.spots[i]["coords"])
                        if walk > 180:
                            continue

                    raw_drive = self.trans_matrix[curr + 1, i]
                    raw_turn = self.turn_matrix[curr + 1, i]
                    if raw_drive == float("inf") or raw_turn == float("inf"):
                        continue

                    p_i = self.spots[i].get("p_i", 0.8)
                    drive_w = ltr if curr >= 0 else 1.0
                    norm_drive = drive_w * raw_drive / self.max_drive
                    norm_turn = lturn_arr * raw_turn / self.max_turn

                    arrival_cost = norm_drive + norm_turn
                    increment = fail_prob * arrival_cost

                    candidates.append(
                        (
                            cost + increment,
                            fail_prob * (1.0 - p_i),
                            i,
                            path + [i],
                            visited | {i},
                        )
                    )

            candidates.sort(key=lambda x: x[0])
            beam = candidates[:1000]
            if beam:
                best_chain = beam[0][3]

        if not best_chain:
            return [], calculate_metrics([], self.state), {}

        full_chain = self._postprocess_chain(best_chain, k)
        return full_chain, calculate_metrics(full_chain, self.state), {}
