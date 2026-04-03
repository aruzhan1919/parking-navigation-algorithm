"""
algorithms.py
=============
Optimization algorithms for selecting the best parking spot chain.

Three algorithms, all inheriting from BaseAlgorithm:

1. MDP_Difference (primary)
   - Beam search (beam_size=1000) with probabilistic cost model
   - Tracks fail_prob: cumulative probability all previous spots were occupied
   - Separates arrival_cost and success_cost, weighted by fail_prob and p_i
   - Post-processes chain to insert free spots along the route

2. FiniteHorizonMDP (alternative)
   - Same beam search but stops expanding states when fail_prob < 0.002
   - Used as baseline when p_i values are very low

3. HeuristicLookahead (greedy baseline)
   - Greedy one-step-at-a-time selection, O(k × n)
   - No beam, no probabilities — picks locally best spot at each step
   - Divides success_cost by p_i to penalize unreliable spots
   - Fast but may produce suboptimal chains vs MDP_Difference

All algorithms share the same post-processing logic:
- Insert spots that lie along the route before/between chosen spots
- Final chain is ordered by actual route position

Cost function (all components normalized to [0,1] via M_* constants):
    arrival_cost  = norm_drive + norm_turn + norm_phi_prev
    success_cost  = λ_w*norm_walk + λ_e*norm_exit + norm_cross + norm_exit_turn
    expected_cost = fail_prob * arrival_cost + fail_prob * p_i * success_cost
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
        """
        Initialize shared data structures and precompute normalization bounds
        and transition matrices used by all algorithms.

        Args:
            state: dict built by StateAdapter.get_state() and extended by app.py with:
                - spots: list of spot dicts
                - drive_fn: callable(u, v) -> {travel_time, turn_penalty}
                - walk_fn: callable(spot_coords) -> seconds
                - topology: list of {progress_ab, progress_bc, side} per spot
                - G_aug: augmented road graph with spot nodes
                - exit_multiplier: float — scales exit route cost
                - start/dest/ref _progress_ab/_progress_bc: float projection values
        """
        self.state = state
        self.spots = state["spots"]
        self.drive_fn = state["drive_fn"]
        self.walk_fn = state["walk_fn"]
        self.topology = state["topology"]
        self.n = len(self.spots)

        # Dual-axis progress values for topology ordering
        self.start_progress_ab = state["start_progress_ab"]
        self.start_progress_bc = state["start_progress_bc"]
        self.dest_progress_ab = state["dest_progress_ab"]
        self.dest_progress_bc = state["dest_progress_bc"]
        self.ref_progress_ab = state["ref_progress_ab"]
        self.ref_progress_bc = state["ref_progress_bc"]

        # Normalization constants — computed dynamically per session
        self.max_drive = 1.0
        self.max_walk = 1.0
        self.max_phi = 1.0
        self.max_turn = 1.0
        self.max_cross = 1.0

        # Precomputed matrices
        # trans_matrix[u+1, v] = travel_time from u to spot v (u=-1 means start)
        # turn_matrix[u+1, v]  = turn_penalty from u to spot v
        # cross_vector[i]      = cross_penalty for spot i
        # static_matrix[i]     = normalized walk + exit cost for spot i
        self.trans_matrix = np.zeros((self.n + 1, self.n))
        self.turn_matrix = np.zeros((self.n + 1, self.n))
        self.cross_vector = np.zeros(self.n)
        self.static_matrix = np.zeros(self.n)

        # Order matters: bounds must be computed before matrix
        self._find_max_bounds()
        self._build_topology_matrix(exit_mult=state.get("exit_multiplier", 1.0))

    def _get_phi(self, idx):
        """
        Return phi_exit_seconds (search overhead penalty) for spot at idx.
        Currently returns 0 for all spots — reserved for future use.
        """
        if idx < 0:
            return 0
        return 0.0

    def _find_max_bounds(self):
        """
        Dynamically compute normalization constants M_drive, M_walk, M_turn,
        M_cross, M_phi from actual route data for this session.

        Samples O(3n) routes instead of O(n²) for efficiency:
            - start → spot_i
            - spot_i → ref
            - spot_i → spot_(i+1)  (for better M_drive estimate)

        Guards against zero division by clamping all maxima to 1.0 minimum.
        """
        drive_times = []
        walk_times = []
        phis = []
        turn_penalties = []
        cross_penalties = []

        for i in range(self.n):
            walk_times.append(self.walk_fn(self.spots[i]["coords"]))
            phis.append(self._get_phi(i))
            cross_penalties.append(self.spots[i].get("cross_penalty", 0.0))

            # start → spot_i
            r1 = self.drive_fn(("node", "start"), ("spot", self.spots[i]))
            drive_times.append(r1["travel_time"])
            turn_penalties.append(r1["turn_penalty"])

            # spot_i → ref
            r2 = self.drive_fn(("spot", self.spots[i]), ("node", "ref"))
            drive_times.append(r2["travel_time"])
            turn_penalties.append(r2["turn_penalty"])

            # spot_i → spot_(i+1) — sample inter-spot transitions
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
        """
        Fill trans_matrix, turn_matrix, cross_vector, and static_matrix.

        static_matrix[i] = normalized (walk + exit) — does not depend on arrival path.
        trans_matrix[u+1, v] = travel_time from u to spot v.
        turn_matrix[u+1, v]  = turn_penalty from u to spot v.
        Diagonal (u==v) is set to inf — a spot cannot follow itself.

        Uses dual-axis progress (AB and BC) to determine active ordering axis.
        """
        # Static costs per spot
        for i in range(self.n):
            walk = self.walk_fn(self.spots[i]["coords"])
            raw_exit = self.drive_fn(("spot", self.spots[i]), ("node", "ref"))[
                "travel_time"
            ]
            self.cross_vector[i] = self.spots[i].get("cross_penalty", 0.0)
            self.static_matrix[i] = (walk / self.max_walk) + (
                (raw_exit * exit_mult) / self.max_drive
            )

        # Transition costs between all pairs
        for u in range(-1, self.n):
            u_node = ("node", "start") if u == -1 else ("spot", self.spots[u])

            u_ab = (
                self.start_progress_ab if u == -1 else self.topology[u]["progress_ab"]
            )
            u_bc = (
                self.start_progress_bc if u == -1 else self.topology[u]["progress_bc"]
            )

            if u_ab <= self.dest_progress_ab:
                active_axis = "progress_ab"
            else:
                active_axis = "progress_bc"

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
        Insert additional spots that lie along the route into the chain.

        Three passes:
        1. Spots BEFORE the first chosen spot (walk_time <= 420s only)
        2. Spots BETWEEN consecutive chosen spots
        3. Append last chosen spot if not already included

        Args:
            best_chain: list of spot indices chosen by the algorithm
            k: max chain depth (unused for trimming currently)

        Returns:
            full_chain: list of spot indices ordered by route position
        """
        WALK_ENROUTE = 420

        def spot_node(idx):
            return f"spot_{self.spots[idx]['id']}"

        if not best_chain:
            return []

        full_chain = []
        used = set()

        # Pass 1: spots before first chosen spot
        first = best_chain[0]
        route_sf = routing.get_route(
            self.state["grid"].nodes["start"],
            spot_node(first),
            custom_G=self.state["G_aug"],
        )
        route_nodes = route_sf["nodes"]
        if spot_node(first) not in route_nodes:
            route_nodes.append(spot_node(first))
        idx_first = route_nodes.index(spot_node(first))

        before_spots = [
            (route_nodes.index(spot_node(j)), j)
            for j in range(self.n)
            if j != first
            and spot_node(j) in route_nodes
            and route_nodes.index(spot_node(j)) < idx_first
            and self.walk_fn(self.spots[j]["coords"]) <= WALK_ENROUTE
        ]
        for _, j in sorted(before_spots):
            full_chain.append(j)
            used.add(j)

        # Pass 2: spots between consecutive chosen spots
        chain = list(full_chain)
        for idx in best_chain:
            if idx not in used:
                chain.append(idx)

        for i in range(len(chain) - 1):
            a, b = chain[i], chain[i + 1]
            if a not in used:
                full_chain.append(a)
                used.add(a)

            route = routing.get_route(
                spot_node(a), spot_node(b), custom_G=self.state["G_aug"]
            )
            route_nodes = route["nodes"]
            route_edges = {(u, v) for u, v in zip(route_nodes[:-1], route_nodes[1:])}
            route_edges |= {(v, u) for u, v in zip(route_nodes[:-1], route_nodes[1:])}

            between = []
            for j in range(self.n):
                if j in used:
                    continue
                jn = spot_node(j)
                if jn in route_nodes:
                    between.append((route_nodes.index(jn), j))
                    continue
                u, v, _ = self.spots[j]["_edge"]
                if (u, v) in route_edges:
                    ref = u if u in route_nodes else v if v in route_nodes else None
                    if ref is not None:
                        between.append((route_nodes.index(ref), j))

            for _, j in sorted(between):
                full_chain.append(j)
                used.add(j)

        # Pass 3: append last chosen spot
        last = chain[-1]
        if last not in used:
            full_chain.append(last)
            used.add(last)

        return full_chain


# ==========================================
# ALGORITHM 1: MDP_Difference (primary)
# ==========================================


class MDP_Difference(BaseAlgorithm):
    def solve(self, **kwargs):
        """
        Build optimal parking chain using beam search with probabilistic cost model.

        Cost model:
            expected_increment = fail_prob * arrival_cost + fail_prob * p_i * success_cost
            arrival_cost = norm_drive + norm_turn + norm_phi_prev
            success_cost = λ_w*norm_walk + λ_e*norm_exit + norm_cross + norm_exit_turn

        Beam search keeps top 1000 candidates at each step.
        First spot must be within 180 seconds walking distance.

        Args (via kwargs):
            k:                 int   — chain depth
            lambda_w:          float — weight for walk time to dest
            lambda_e:          float — weight for exit time to ref
            lambda_tr:         float — weight for drive time between spots
            lambda_turns_arr:  float — weight for turn penalties on arrival
            lambda_turns_exit: float — weight for turn penalties on exit
            lambda_cross:      float — weight for pedestrian crossing penalty

        Returns:
            (full_chain, metrics, {})
        """
        # k = int(kwargs.get("k", 3))
        # lw = kwargs.get("lambda_w", 1.0)
        # le = kwargs.get("lambda_e", 1.0)
        # ltr = kwargs.get("lambda_tr", 1.0)
        # lturn_arr = kwargs.get("lambda_turns_arr", 1.0)
        # lturn_exit = kwargs.get("lambda_turns_exit", 1.0)
        # lcross = kwargs.get("lambda_cross", 1.0)

        # beam = [(0.0, 1.0, -1, [], frozenset())]
        # best_chain = []

        # for step in range(k):
        #     candidates = []
        #     for cost_so_far, fail_prob, curr, path, visited in beam:
        #         phi_prev = self._get_phi(curr) if curr >= 0 else 0
        #         norm_phi_prev = phi_prev / self.max_phi

        #         for i in range(self.n):
        #             raw_drive = self.trans_matrix[curr + 1, i]
        #             raw_turn = self.turn_matrix[curr + 1, i]
        #             if (
        #                 raw_drive == float("inf")
        #                 or raw_turn == float("inf")
        #                 or i in visited
        #             ):
        #                 continue

        #             p_i = self.spots[i].get("p_i", 0.8)
        #             walk = self.walk_fn(self.spots[i]["coords"])
        #             if curr == -1 and walk > 180:
        #                 continue

        #             r = self.drive_fn(("spot", self.spots[i]), ("node", "ref"))
        #             exit_d = r["travel_time"]
        #             exit_turn = r["turn_penalty"]
        #             raw_cross = self.cross_vector[i]

        #             drive_w = ltr if curr >= 0 else 1.0
        #             norm_drive = (drive_w * raw_drive) / self.max_drive
        #             norm_turn = (lturn_arr * raw_turn) / self.max_turn
        #             norm_cross = (lcross * raw_cross) / self.max_cross
        #             norm_exit_turn = (lturn_exit * exit_turn) / self.max_turn

        #             arrival_cost = norm_drive + norm_turn + norm_phi_prev
        #             success_cost = (
        #                 (lw * (walk / self.max_walk))
        #                 + (le * (exit_d / self.max_drive))
        #                 + norm_cross
        #                 + norm_exit_turn
        #             )
        #             expected_increment = (
        #                 fail_prob * arrival_cost + fail_prob * p_i * success_cost
        #             )

        #             candidates.append(
        #                 (
        #                     cost_so_far + expected_increment,
        #                     fail_prob * (1.0 - p_i),
        #                     i,
        #                     path + [i],
        #                     visited | {i},
        #                 )
        #             )

        #     candidates.sort(key=lambda x: x[0])
        #     beam = candidates[:1000]
        #     if beam:
        #         best_chain = beam[0][3]

        # if not best_chain:
        #     return [], calculate_metrics([], self.state), {}

        # # Post-process: insert free spots along the route
        # def spot_node(idx):
        #     return f"spot_{self.spots[idx]['id']}"

        # full_chain = []
        # used = set()

        # first = best_chain[0]
        # first_node = spot_node(first)
        # route_sf = routing.get_route(
        #     self.state["grid"].nodes["start"],
        #     first_node,
        #     custom_G=self.state["G_aug"],
        # )
        # route_nodes = route_sf["nodes"]
        # if first_node not in route_nodes:
        #     route_nodes.append(first_node)
        # idx_first = route_nodes.index(first_node)

        # before_spots = []
        # for j in range(self.n):
        #     if j == first:
        #         continue
        #     j_node = spot_node(j)
        #     if j_node in route_nodes:
        #         idx_j = route_nodes.index(j_node)
        #         if idx_j < idx_first and self.walk_fn(self.spots[j]["coords"]) <= 300:
        #             before_spots.append((idx_j, j))

        # before_spots.sort(key=lambda x: x[0])
        # for _, j in before_spots:
        #     full_chain.append(j)
        #     used.add(j)

        # full_chain_base = full_chain.copy()
        # for idx in best_chain:
        #     if idx not in full_chain_base:
        #         full_chain_base.append(idx)

        # for i in range(len(full_chain_base) - 1):
        #     a, b = full_chain_base[i], full_chain_base[i + 1]
        #     if a not in used:
        #         full_chain.append(a)
        #         used.add(a)

        #     route = routing.get_route(
        #         spot_node(a),
        #         spot_node(b),
        #         custom_G=self.state["G_aug"],
        #     )
        #     route_nodes = route["nodes"]
        #     route_edges = set()
        #     for u, v in zip(route_nodes[:-1], route_nodes[1:]):
        #         route_edges.add((u, v))
        #         route_edges.add((v, u))

        #     between_spots = []
        #     for j in range(self.n):
        #         if j in used:
        #             continue
        #         j_node = spot_node(j)
        #         if j_node in route_nodes:
        #             between_spots.append((route_nodes.index(j_node), j))
        #             continue
        #         u, v, _ = self.spots[j]["_edge"]
        #         if (u, v) in route_edges:
        #             if u in route_nodes:
        #                 between_spots.append((route_nodes.index(u), j))
        #             elif v in route_nodes:
        #                 between_spots.append((route_nodes.index(v), j))

        #     between_spots.sort(key=lambda x: x[0])
        #     for _, j in between_spots:
        #         full_chain.append(j)
        #         used.add(j)

        # last = best_chain[-1]
        # if last not in used:
        #     full_chain.append(last)

        # return full_chain, calculate_metrics(full_chain, self.state), {}

        k = int(kwargs.get("k", 3))
        lw = kwargs.get("lambda_w", 1.0)
        le = kwargs.get("lambda_e", 1.0)
        ltr = kwargs.get("lambda_tr", 1.0)
        lturn_arr = kwargs.get("lambda_turns_arr", 1.0)
        lturn_exit = kwargs.get("lambda_turns_exit", 1.0)
        lcross = kwargs.get("lambda_cross", 1.0)

        WALK_FIRST = 180  # sec — max walk to first spot from start
        WALK_ENROUTE = 420  # sec — max walk to enroute spots

        # ── Beam search ───────────────────────────────────────────────────────
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

        # ── Postprocessing: insert free spots along the route ─────────────────
        # def spot_node(idx):
        #     return f"spot_{self.spots[idx]['id']}"

        # full_chain = []
        # used = set()

        # # 1. Spots physically before the first chosen spot on start→first route
        # first = best_chain[0]
        # route_sf = routing.get_route(
        #     self.state["grid"].nodes["start"],
        #     spot_node(first),
        #     custom_G=self.state["G_aug"],
        # )
        # route_nodes = route_sf["nodes"]
        # if spot_node(first) not in route_nodes:
        #     route_nodes.append(spot_node(first))
        # idx_first = route_nodes.index(spot_node(first))

        # before_spots = [
        #     (route_nodes.index(spot_node(j)), j)
        #     for j in range(self.n)
        #     if j != first
        #     and spot_node(j) in route_nodes
        #     and route_nodes.index(spot_node(j)) < idx_first
        #     and self.walk_fn(self.spots[j]["coords"]) <= WALK_ENROUTE
        # ]
        # for _, j in sorted(before_spots):
        #     full_chain.append(j)
        #     used.add(j)

        # # 2. Walk each a→b segment, inserting spots that lie on the path
        # chain = list(full_chain)
        # for idx in best_chain:
        #     if idx not in used:
        #         chain.append(idx)

        # for i in range(len(chain) - 1):
        #     a, b = chain[i], chain[i + 1]
        #     if a not in used:
        #         full_chain.append(a)
        #         used.add(a)

        #     route = routing.get_route(
        #         spot_node(a), spot_node(b), custom_G=self.state["G_aug"]
        #     )
        #     route_nodes = route["nodes"]
        #     route_edges = {(u, v) for u, v in zip(route_nodes[:-1], route_nodes[1:])}
        #     route_edges |= {(v, u) for u, v in zip(route_nodes[:-1], route_nodes[1:])}

        #     between = []
        #     for j in range(self.n):
        #         if j in used:
        #             continue
        #         jn = spot_node(j)
        #         if jn in route_nodes:
        #             between.append((route_nodes.index(jn), j))
        #             continue
        #         u, v, _ = self.spots[j]["_edge"]
        #         if (u, v) in route_edges:
        #             ref = u if u in route_nodes else v if v in route_nodes else None
        #             if ref is not None:
        #                 between.append((route_nodes.index(ref), j))

        #     for _, j in sorted(between):
        #         full_chain.append(j)
        #         used.add(j)

        # # 3. Commit the final node
        # last = chain[-1]
        # if last not in used:
        #     full_chain.append(last)
        #     used.add(last)
        full_chain = self._postprocess_chain(best_chain, k)
        return full_chain, calculate_metrics(full_chain, self.state), {}


# class MDP_Difference(BaseAlgorithm):
#     DEBUG_DATA = []

#     def solve(self, **kwargs):
#         """
#         Build optimal parking chain using beam search with probabilistic cost model.

#         Cost model:
#             expected_increment = fail_prob * arrival_cost + fail_prob * p_i * success_cost
#             arrival_cost = norm_drive + norm_turn + norm_phi_prev
#             success_cost = λ_w*norm_walk + λ_e*norm_exit + norm_cross + norm_exit_turn

#         Beam search keeps top 1000 candidates at each step.
#         First spot must be within 180 seconds walking distance.

#         Returns:
#             (full_chain, metrics, {})
#         """
#         self.DEBUG_DATA = []

#         k = int(kwargs.get("k", 3))
#         lw = kwargs.get("lambda_w", 1.0)
#         le = kwargs.get("lambda_e", 1.0)
#         ltr = kwargs.get("lambda_tr", 1.0)
#         lturn_arr = kwargs.get("lambda_turns_arr", 1.0)
#         lturn_exit = kwargs.get("lambda_turns_exit", 1.0)
#         lcross = kwargs.get("lambda_cross", 1.0)

#         beam = [(0.0, 1.0, -1, [], frozenset())]
#         best_chain = []

#         for step in range(k):
#             candidates = []

#             for cost_so_far, fail_prob, curr, path, visited in beam:
#                 phi_prev = self._get_phi(curr) if curr >= 0 else 0
#                 norm_phi_prev = phi_prev / self.max_phi

#                 for i in range(self.n):
#                     raw_drive = self.trans_matrix[curr + 1, i]
#                     raw_turn = self.turn_matrix[curr + 1, i]

#                     if (
#                         raw_drive == float("inf")
#                         or raw_turn == float("inf")
#                         or i in visited
#                     ):
#                         continue

#                     p_i = self.spots[i].get("p_i", 0.8)
#                     walk = self.walk_fn(self.spots[i]["coords"])

#                     if curr == -1 and walk > 180:
#                         continue

#                     r = self.drive_fn(("spot", self.spots[i]), ("node", "ref"))
#                     exit_d = r["travel_time"]
#                     exit_turn = r["turn_penalty"]
#                     raw_cross = self.cross_vector[i]

#                     drive_w = ltr if curr >= 0 else 1.0

#                     # pure normalized, without lambdas
#                     pure_norm_drive = raw_drive / self.max_drive
#                     pure_norm_turn = raw_turn / self.max_turn
#                     pure_norm_walk = walk / self.max_walk
#                     pure_norm_exit = exit_d / self.max_drive
#                     pure_norm_cross = raw_cross / self.max_cross
#                     pure_norm_exit_turn = exit_turn / self.max_turn

#                     # weighted terms, actually used in optimization
#                     norm_drive = drive_w * pure_norm_drive
#                     norm_turn = lturn_arr * pure_norm_turn
#                     norm_walk = lw * pure_norm_walk
#                     norm_exit = le * pure_norm_exit
#                     norm_cross = lcross * pure_norm_cross
#                     norm_exit_turn = lturn_exit * pure_norm_exit_turn

#                     self.DEBUG_DATA.append(
#                         {
#                             "step": step,
#                             "from_idx": curr,
#                             "to_idx": i,
#                             "raw_drive": raw_drive,
#                             "raw_turn": raw_turn,
#                             "walk": walk,
#                             "exit_drive": exit_d,
#                             "exit_turn": exit_turn,
#                             "cross": raw_cross,
#                             "p_i": p_i,
#                             "pure_norm_drive": pure_norm_drive,
#                             "pure_norm_turn": pure_norm_turn,
#                             "pure_norm_walk": pure_norm_walk,
#                             "pure_norm_exit": pure_norm_exit,
#                             "pure_norm_cross": pure_norm_cross,
#                             "pure_norm_exit_turn": pure_norm_exit_turn,
#                             "norm_drive": norm_drive,
#                             "norm_turn": norm_turn,
#                             "norm_walk": norm_walk,
#                             "norm_exit": norm_exit,
#                             "norm_cross": norm_cross,
#                             "norm_exit_turn": norm_exit_turn,
#                             "fail_prob": fail_prob,
#                             "drive_weight": drive_w,
#                             "lambda_w": lw,
#                             "lambda_e": le,
#                             "lambda_tr": ltr,
#                             "lambda_turns_arr": lturn_arr,
#                             "lambda_turns_exit": lturn_exit,
#                             "lambda_cross": lcross,
#                         }
#                     )

#                     arrival_cost = norm_drive + norm_turn + norm_phi_prev
#                     success_cost = norm_walk + norm_exit + norm_cross + norm_exit_turn

#                     expected_increment = (
#                         fail_prob * arrival_cost + fail_prob * p_i * success_cost
#                     )

#                     candidates.append(
#                         (
#                             cost_so_far + expected_increment,
#                             fail_prob * (1.0 - p_i),
#                             i,
#                             path + [i],
#                             visited | {i},
#                         )
#                     )

#             candidates.sort(key=lambda x: x[0])
#             beam = candidates[:1000]

#             if beam:
#                 best_chain = beam[0][3]

#         if not best_chain:
#             return [], calculate_metrics([], self.state), {}

#         def spot_node(idx):
#             return f"spot_{self.spots[idx]['id']}"

#         full_chain = []
#         used = set()

#         first = best_chain[0]
#         first_node = spot_node(first)
#         route_sf = routing.get_route(
#             self.state["grid"].nodes["start"],
#             first_node,
#             custom_G=self.state["G_aug"],
#         )
#         route_nodes = route_sf["nodes"]

#         if first_node not in route_nodes:
#             route_nodes.append(first_node)

#         idx_first = route_nodes.index(first_node)

#         before_spots = []
#         for j in range(self.n):
#             if j == first:
#                 continue

#             j_node = spot_node(j)
#             if j_node in route_nodes:
#                 idx_j = route_nodes.index(j_node)
#                 if idx_j < idx_first and self.walk_fn(self.spots[j]["coords"]) <= 300:
#                     before_spots.append((idx_j, j))

#         before_spots.sort(key=lambda x: x[0])
#         for _, j in before_spots:
#             full_chain.append(j)
#             used.add(j)

#         full_chain_base = full_chain.copy()
#         for idx in best_chain:
#             if idx not in full_chain_base:
#                 full_chain_base.append(idx)

#         for i in range(len(full_chain_base) - 1):
#             a, b = full_chain_base[i], full_chain_base[i + 1]

#             if a not in used:
#                 full_chain.append(a)
#                 used.add(a)

#             route = routing.get_route(
#                 spot_node(a),
#                 spot_node(b),
#                 custom_G=self.state["G_aug"],
#             )
#             route_nodes = route["nodes"]

#             route_edges = set()
#             for u, v in zip(route_nodes[:-1], route_nodes[1:]):
#                 route_edges.add((u, v))
#                 route_edges.add((v, u))

#             between_spots = []
#             for j in range(self.n):
#                 if j in used:
#                     continue

#                 j_node = spot_node(j)

#                 if j_node in route_nodes:
#                     between_spots.append((route_nodes.index(j_node), j))
#                     continue

#                 u, v, _ = self.spots[j]["_edge"]
#                 if (u, v) in route_edges:
#                     if u in route_nodes:
#                         between_spots.append((route_nodes.index(u), j))
#                     elif v in route_nodes:
#                         between_spots.append((route_nodes.index(v), j))

#             between_spots.sort(key=lambda x: x[0])
#             for _, j in between_spots:
#                 full_chain.append(j)
#                 used.add(j)

#         last = best_chain[-1]
#         if last not in used:
#             full_chain.append(last)

#         return full_chain, calculate_metrics(full_chain, self.state), {}


# ==========================================
# ALGORITHM 2: FiniteHorizonMDP (alternative)
# ==========================================


class FiniteHorizonMDP(BaseAlgorithm):
    def solve(self, **kwargs):
        """
        Beam search with early stopping when fail_prob < 0.002.

        Stops expanding a beam state when cumulative failure probability
        drops below 0.2% — the chain is already reliable enough.
        Same cost formula and post-processing as MDP_Difference.

        Args: same as MDP_Difference.
        Returns: (full_chain, metrics, {})
        """
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

        # Post-processing (identical to MDP_Difference)
        # def spot_node(idx):
        #     return f"spot_{self.spots[idx]['id']}"

        # full_chain = []
        # used = set()

        # first = best_chain[0]
        # first_node = spot_node(first)
        # route_sf = routing.get_route(
        #     self.state["grid"].nodes["start"],
        #     first_node,
        #     custom_G=self.state["G_aug"],
        # )
        # route_nodes = route_sf["nodes"]
        # if first_node not in route_nodes:
        #     route_nodes.append(first_node)
        # idx_first = route_nodes.index(first_node)

        # before_spots = []
        # for j in range(self.n):
        #     if j == first:
        #         continue
        #     j_node = spot_node(j)
        #     if j_node in route_nodes:
        #         idx_j = route_nodes.index(j_node)
        #         if idx_j < idx_first and self.walk_fn(self.spots[j]["coords"]) <= 300:
        #             before_spots.append((idx_j, j))

        # before_spots.sort(key=lambda x: x[0])
        # for _, j in before_spots:
        #     full_chain.append(j)
        #     used.add(j)

        # chain = full_chain.copy()
        # for idx in best_chain:
        #     if idx not in chain:
        #         chain.append(idx)

        # for i in range(len(chain) - 1):
        #     a, b = chain[i], chain[i + 1]
        #     if a not in used:
        #         full_chain.append(a)
        #         used.add(a)

        #     route = routing.get_route(
        #         spot_node(a),
        #         spot_node(b),
        #         custom_G=self.state["G_aug"],
        #     )
        #     route_nodes = route["nodes"]
        #     route_edges = set()
        #     for u, v in zip(route_nodes[:-1], route_nodes[1:]):
        #         route_edges.add((u, v))
        #         route_edges.add((v, u))

        #     between_spots = []
        #     for j in range(self.n):
        #         if j in used:
        #             continue
        #         j_node = spot_node(j)
        #         if j_node in route_nodes:
        #             between_spots.append((route_nodes.index(j_node), j))
        #             continue
        #         u, v, _ = self.spots[j]["_edge"]
        #         if (u, v) in route_edges:
        #             if u in route_nodes:
        #                 between_spots.append((route_nodes.index(u), j))
        #             elif v in route_nodes:
        #                 between_spots.append((route_nodes.index(v), j))

        #     between_spots.sort(key=lambda x: x[0])
        #     for _, j in between_spots:
        #         full_chain.append(j)
        #         used.add(j)

        # last = best_chain[-1]
        # if last not in used:
        #     full_chain.append(last)
        full_chain = self._postprocess_chain(best_chain, k)
        return full_chain, calculate_metrics(full_chain, self.state), {}


# ==========================================
# ALGORITHM 3: HeuristicLookahead (greedy)
# ==========================================


class HeuristicLookahead(BaseAlgorithm):
    def solve(self, **kwargs):
        """
        Greedy one-step-at-a-time spot selection. O(k × n) complexity.

        At each step picks the locally best spot using:
            q = arrival_cost + success_cost / max(p_i, 0.1)

        Dividing by p_i penalizes unreliable spots — spots with low
        probability of being free are ranked lower even if close.

        No beam, no global optimization — fast but may miss better chains
        that require a slightly worse first step.

        Args: same as MDP_Difference.
        Returns: (full_chain, metrics, {})
        """
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

                # Greedy heuristic: penalize unreliable spots by dividing by p_i
                q = arrival_cost + (success_cost / max(p_i, 0.1))

                if q < min_q:
                    min_q, best_i = q, i

            if best_i == -1:
                break
            chain.append(best_i)
            curr = best_i

        if not chain:
            return [], calculate_metrics([], self.state), {}

        # Post-processing (identical to MDP_Difference)
        # def spot_node(idx):
        #     return f"spot_{self.spots[idx]['id']}"

        # full_chain = []
        # used = set()

        # first = chain[0]
        # first_node = spot_node(first)
        # route_sf = routing.get_route(
        #     self.state["grid"].nodes["start"],
        #     first_node,
        #     custom_G=self.state["G_aug"],
        # )
        # route_nodes = route_sf["nodes"]
        # if first_node not in route_nodes:
        #     route_nodes.append(first_node)
        # idx_first = route_nodes.index(first_node)

        # before_spots = []
        # for j in range(self.n):
        #     if j == first:
        #         continue
        #     j_node = spot_node(j)
        #     if j_node in route_nodes:
        #         idx_j = route_nodes.index(j_node)
        #         if idx_j < idx_first and self.walk_fn(self.spots[j]["coords"]) <= 300:
        #             before_spots.append((idx_j, j))

        # before_spots.sort(key=lambda x: x[0])
        # for _, j in before_spots:
        #     full_chain.append(j)
        #     used.add(j)

        # full_chain_base = full_chain.copy()
        # for idx in chain:
        #     if idx not in full_chain_base:
        #         full_chain_base.append(idx)

        # for i in range(len(full_chain_base) - 1):
        #     a, b = full_chain_base[i], full_chain_base[i + 1]
        #     if a not in used:
        #         full_chain.append(a)
        #         used.add(a)

        #     route = routing.get_route(
        #         spot_node(a),
        #         spot_node(b),
        #         custom_G=self.state["G_aug"],
        #     )
        #     route_nodes = route["nodes"]
        #     route_edges = set()
        #     for u, v in zip(route_nodes[:-1], route_nodes[1:]):
        #         route_edges.add((u, v))
        #         route_edges.add((v, u))

        #     between_spots = []
        #     for j in range(self.n):
        #         if j in used:
        #             continue
        #         j_node = spot_node(j)
        #         if j_node in route_nodes:
        #             between_spots.append((route_nodes.index(j_node), j))
        #             continue
        #         u, v, _ = self.spots[j]["_edge"]
        #         if (u, v) in route_edges:
        #             if u in route_nodes:
        #                 between_spots.append((route_nodes.index(u), j))
        #             elif v in route_nodes:
        #                 between_spots.append((route_nodes.index(v), j))

        #     between_spots.sort(key=lambda x: x[0])
        #     for _, j in between_spots:
        #         full_chain.append(j)
        #         used.add(j)

        # last = chain[-1]
        # if last not in used:
        #     full_chain.append(last)

        # full_chain = full_chain[:k]
        full_chain = self._postprocess_chain(chain, k)
        return full_chain, calculate_metrics(full_chain, self.state), {}


class TwoStageMDP(BaseAlgorithm):
    def solve(self, **kwargs):
        """
        Two-stage parking chain optimization.

        Stage 1: Score all spots by quality (p_i × success_cost),
                 select top k spots.
        Stage 2: Beam search over selected spots using arrival cost only
                 (drive + turn penalties), weighted by fail_prob.

        Returns:
            (full_chain, metrics, {})
        """
        k = int(kwargs.get("k", 3))
        lw = kwargs.get("lambda_w", 1.0)
        le = kwargs.get("lambda_e", 1.0)
        lturn_exit = kwargs.get("lambda_turns_exit", 1.0)
        lturn_arr = kwargs.get("lambda_turns_arr", 1.0)
        ltr = kwargs.get("lambda_tr", 1.0)
        lcross = kwargs.get("lambda_cross", 1.0)

        # ── Stage 1: select top k spots by quality ────────────────────────────
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

        # sort ascending — lower cost is better quality
        quality_scores.sort(key=lambda x: x[0])
        top_k_indices = [i for _, i in quality_scores[:k]]

        # ── Stage 2: beam search over top k spots using arrival cost only ─────
        beam = [(0.0, 1.0, -1, [], frozenset())]
        best_chain = []

        for _ in range(k):
            candidates = []
            for cost, fail_prob, curr, path, visited in beam:
                for i in top_k_indices:
                    if i in visited:
                        continue

                    # filter first spot by walk distance
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
