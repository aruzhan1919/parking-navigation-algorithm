import numpy as np
from abc import ABC
from utils import calculate_metrics


class BaseAlgorithm(ABC):
    def __init__(self, state):
        self.state, self.spots = state, state["spots"]
        self.drive_fn, self.walk_fn = state["drive_fn"], state["walk_fn"]
        self.topology = state["topology"]
        self.n = len(self.spots)
        ######## CHANGE############
        self.dest_progress = state["dest_progress"]
        self.ref_progress = state["ref_progress"]
        # Initialize dynamic normalization bounds
        self.max_drive = 1.0
        self.max_walk = 1.0
        self.max_phi = 1.0

        # Matrices
        self.trans_matrix = np.zeros(
            (self.n + 1, self.n)
        )  # n+1 rows, n cols (cost of driving from one spot to other)
        self.static_matrix = np.zeros(
            self.n
        )  # 1 row with n vals (How bad is this spot if it succeeds?)

        # 1. First, find the maximums to establish denominators
        self._find_max_bounds()

        # 2. Build the topology using those bounds
        self._build_topology_matrix(exit_mult=state.get("exit_multiplier", 1.0))

    def _get_phi(self, idx):
        if idx < 0:
            return 0
        # return self.spots[idx].get("phi_exit_seconds", 60)
        return 0.0

    def _find_max_bounds(self):  # finds max drive, walk, exit times
        """
        Dynamically calculates M_drive, M_walk, and M_phi
        based on the actual input coordinates and spots.
        """
        drive_times = []
        walk_times = []
        phis = []

        for i in range(self.n):
            # Walking components
            walk_times.append(self.walk_fn(self.spots[i]["coords"]))

            # Search components
            phis.append(self._get_phi(i))

            # Driving components (Start -> Spot and Spot -> Ref)
            drive_times.append(
                self.drive_fn(("node", "start"), ("spot", self.spots[i]))
            )
            drive_times.append(self.drive_fn(("spot", self.spots[i]), ("node", "ref")))

            # Sample Spot -> Spot transitions for a better M_drive estimate
            # (We don't do all n^2 to stay efficient, but we take enough samples)
            if self.n > 1:
                next_idx = (i + 1) % self.n
                drive_times.append(
                    self.drive_fn(
                        ("spot", self.spots[i]), ("spot", self.spots[next_idx])
                    )
                )

        self.max_drive = max(drive_times) if drive_times else 1.0
        self.max_walk = max(walk_times) if walk_times else 1.0
        self.max_phi = max(phis) if phis else 1.0

        # Guard against zero division
        if self.max_drive == 0:
            self.max_drive = 1.0
        if self.max_walk == 0:
            self.max_walk = 1.0
        if self.max_phi == 0:
            self.max_phi = 1.0

    def _build_topology_matrix(self, exit_mult=1.0):
        # 1. Static Cost: Quality metrics normalized by dynamic maximums
        for i in range(self.n):
            walk = self.walk_fn(self.spots[i]["coords"])
            raw_exit = self.drive_fn(("spot", self.spots[i]), ("node", "ref"))
            # Static Success Cost = Normalized Walk + Normalized Exit
            self.static_matrix[i] = (walk / self.max_walk) + (
                (raw_exit * exit_mult) / self.max_drive
            )  # how bad is this spot compared to all other spots

        # 2. Transition Matrix: Real road times
        for u in range(-1, self.n):
            u_prog = (
                self.state["start_progress"]
                if u == -1
                else self.topology[u]["progress"]
            )
            u_node = ("node", "start") if u == -1 else ("spot", self.spots[u])
            ######### CHANGE ############
            # if u == -1:
            #     u_prog_ab = self.state["start_progress_ab"]
            #     u_prog_bc = self.state["start_progress_bc"]
            #     u_node = ("node", "start")
            # else:
            #     u_prog_ab = self.topology[u]["progress_ab"]
            #     u_prog_bc = self.topology[u]["progress_bc"]
            #     u_node = ("spot", self.spots[u])

            for v in range(self.n):
                if u == v:
                    self.trans_matrix[u + 1, v] = float("inf")
                    continue

                v_prog = self.topology[v]["progress"]
                # v_prog_ab = self.topology[v]["progress_ab"]
                # v_prog_bc = self.topology[v]["progress_bc"]
                # Zero-tolerance backtracking
                ############ CHANGE ###############
                # if v_prog < u_prog:
                #     self.trans_matrix[u + 1, v] = float("inf")
                #     continue
                # if v_prog < u_prog and (self.dest_progress <= self.ref_progress):
                #     self.trans_matrix[u + 1, v] = float("inf")
                #     continue
                # ---- Smoothed Prohibition ----
                # Only prohibit if spot is backwards in BOTH directions
                # if (v_prog_ab < u_prog_ab) and (v_prog_bc < u_prog_bc):
                #     self.trans_matrix[u + 1, v] = float("inf")
                #     continue
                # если мы ещё не прошли dest
                # if u != -1:
                #     if u_prog <= self.dest_progress:
                #         if v_prog < u_prog:
                #             self.trans_matrix[u + 1, v] = float("inf")
                #             continue
                # if u != -1:
                #     if v_prog < u_prog:
                #         self.trans_matrix[u + 1, v] = float("inf")
                #         continue
                # same_edge = u != -1 and self.spots[u].get("_edge") == self.spots[v].get(
                #     "_edge"
                # )

                # if v_prog < u_prog and not same_edge:
                #     self.trans_matrix[u + 1, v] = float("inf")
                #     continue
                # same_edge = u != -1 and self.spots[u].get("_edge") == self.spots[v].get(
                #     "_edge"
                # )

                # b1 = self.spots[u].get("_bearing", 0) if u != -1 else 0
                # b2 = self.spots[v].get("_bearing", 0)

                # angle = abs((b2 - b1 + 180) % 360 - 180)

                # if v_prog < u_prog and not (same_edge or angle < 45):
                #     self.trans_matrix[u + 1, v] = float("inf")
                #     continue

                self.trans_matrix[u + 1, v] = self.drive_fn(
                    u_node, ("spot", self.spots[v])
                )


# Given many possible parking spots, each with probability
# of success and costs, find an ordered chain of at most k spots
# that minimizes the expected total time of the trip.


class MDP_Difference(BaseAlgorithm):
    def solve(self, **kwargs):
        k = int(kwargs.get("k", 3))
        beam_width = kwargs.get("beam_width", 200)

        # Each element: (chain, visited_set)
        beam = [([], set())]
        best_chain = []
        best_value = float("inf")

        for depth in range(k):
            candidates = []

            for chain, visited in beam:
                for i in range(self.n):
                    if i in visited:
                        continue

                    new_chain = chain + [i]
                    new_visited = visited | {i}

                    # 🔹 TRUE expected time evaluation
                    expected_time, *_ = calculate_metrics(new_chain, self.state)

                    candidates.append((expected_time, new_chain, new_visited))

            if not candidates:
                break

            # Sort by real expected time
            candidates.sort(key=lambda x: x[0])

            # Keep best beam_width
            beam = [(chain, visited) for _, chain, visited in candidates[:beam_width]]

            # Track best complete chain seen so far
            if candidates[0][0] < best_value:
                best_value = candidates[0][0]
                best_chain = candidates[0][1]

        # Final evaluation
        metrics = calculate_metrics(best_chain, self.state)

        return best_chain, metrics, {}

class FiniteHorizonMDP(BaseAlgorithm):
    def solve(self, **kwargs):
        k, lw, le, ltr = (
            int(kwargs.get("k", 3)),
            kwargs.get("lambda_w", 1.0),
            kwargs.get("lambda_e", 1.0),
            kwargs.get("lambda_tr", 1.0),
        )
        beam = [(0.0, 1.0, -1, [], frozenset())]
        # first_idx = min(
        #     range(self.n), key=lambda i: self.walk_fn(self.spots[i]["coords"])
        # )

        # beam = [(0.0, 1.0, first_idx, [first_idx], {first_idx})]
        best_completed = None
        min_cost = float("inf")

        for step in range(max(k, 7)):
            candidates = []
            for cost, fail_p, curr, path, visited in beam:
                if fail_p < 0.002:
                    if cost < min_cost:
                        min_cost, best_completed = cost, path
                    continue
                phi_prev = self._get_phi(curr) if curr >= 0 else 0
                for i in range(self.n):
                    raw_drive = self.trans_matrix[curr + 1, i]
                    if raw_drive == float("inf") or i in visited:
                        continue
                    p_i, walk = (
                        self.spots[i].get("p_i", 0.8),
                        self.walk_fn(self.spots[i]["coords"]),
                    )
                    exit_d, phi_i = (
                        self.drive_fn(("spot", self.spots[i]), ("node", "ref")),
                        self._get_phi(i),
                    )
                    drive_w = ltr if curr >= 0 else 1.0
                    n_drive = (drive_w * raw_drive) / self.max_drive
                    n_phi_p = phi_prev / self.max_phi

                    c_success = (
                        (n_drive + n_phi_p)
                        + (lw * walk / self.max_walk)
                        + (le * exit_d / self.max_drive)
                    )
                    c_fail = (n_drive + n_phi_p) + (phi_i / self.max_phi)

                    candidates.append(
                        (
                            cost + fail_p * (p_i * c_success + (1.0 - p_i) * c_fail),
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
        return (
            (best_completed or beam[0][3]),
            calculate_metrics(best_completed or beam[0][3], self.state),
            {},
        )


class HeuristicLookahead(BaseAlgorithm):
    def solve(self, **kwargs):
        k, lw, le, ltr = (
            int(kwargs.get("k", 3)),
            kwargs.get("lambda_w", 1.0),
            kwargs.get("lambda_e", 1.0),
            kwargs.get("lambda_tr", 1.0),
        )
        chain, curr = [], -1
        for _ in range(k):
            best_i, min_q = -1, float("inf")
            phi_prev = self._get_phi(curr) if curr >= 0 else 0
            for i in range(self.n):
                raw_drive = self.trans_matrix[curr + 1, i]
                if raw_drive == float("inf") or i in chain:
                    continue
                p_i = self.spots[i].get("p_i", 0.8)
                walk, exit_d = (
                    self.walk_fn(self.spots[i]["coords"]),
                    self.drive_fn(("spot", self.spots[i]), ("node", "ref")),
                )
                drive_w = ltr if curr >= 0 else 1.0
                n_step = ((drive_w * raw_drive) / self.max_drive) + (
                    phi_prev / self.max_phi
                )
                n_quality = (
                    (lw * walk / self.max_walk)
                    + (le * exit_d / self.max_drive)
                    + (self._get_phi(i) / self.max_phi)
                )
                q = n_step + (n_quality / max(p_i, 0.1))
                if q < min_q:
                    min_q, best_i = q, i
            if best_i == -1:
                break
            chain.append(best_i)
            curr = best_i
        return chain, calculate_metrics(chain, self.state), {}
