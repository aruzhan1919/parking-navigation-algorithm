import numpy as np
from abc import ABC
from utils import calculate_metrics


class BaseAlgorithm(ABC):
    def __init__(self, state):

        self.state = state
        self.spots = state["spots"]
        self.drive_fn = state["drive_fn"]
        self.walk_fn = state["walk_fn"]
        self.topology = state["topology"]

        self.n = len(self.spots)

        # -------- Dual-axis progress --------
        self.start_progress_ab = state["start_progress_ab"]
        self.start_progress_bc = state["start_progress_bc"]

        self.dest_progress_ab = state["dest_progress_ab"]
        self.dest_progress_bc = state["dest_progress_bc"]

        self.ref_progress_ab = state["ref_progress_ab"]
        self.ref_progress_bc = state["ref_progress_bc"]

        # -------- Normalization --------
        self.max_drive = 1.0
        self.max_walk = 1.0
        self.max_phi = 1.0
        self.max_turn = 1.0  # NEW
        self.max_cross = 1.0  # NEW

        self.trans_matrix = np.zeros((self.n + 1, self.n))
        self.turn_matrix = np.zeros((self.n + 1, self.n))  # NEW
        self.cross_vector = np.zeros(self.n)  # NEW
        self.static_matrix = np.zeros(self.n)

        self._find_max_bounds()
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
        turn_penalties = []
        cross_penalties = []

        for i in range(self.n):
            # Walking components
            walk_times.append(self.walk_fn(self.spots[i]["coords"]))

            # Search components
            phis.append(self._get_phi(i))

            cross_penalties.append(self.spots[i].get("cross_penalty", 0.0))  # NEW

            # Driving components (Start -> Spot and Spot -> Ref)
            # drive_times.append(
            #     self.drive_fn(("node", "start"), ("spot", self.spots[i]))
            # )
            # Start -> Spot
            r1 = self.drive_fn(("node", "start"), ("spot", self.spots[i]))
            drive_times.append(r1["travel_time"])
            turn_penalties.append(r1["turn_penalty"])  # NEW

            # drive_times.append(self.drive_fn(("spot", self.spots[i]), ("node", "ref")))
            # Spot -> Ref
            r2 = self.drive_fn(("spot", self.spots[i]), ("node", "ref"))
            drive_times.append(r2["travel_time"])
            turn_penalties.append(r2["turn_penalty"])  # NEW

            # Sample Spot -> Spot transitions for a better M_drive estimate
            # (We don't do all n^2 to stay efficient, but we take enough samples)
            if self.n > 1:
                next_idx = (i + 1) % self.n
                # drive_times.append(
                #     self.drive_fn(
                #         ("spot", self.spots[i]), ("spot", self.spots[next_idx])
                #     )
                # )
                r3 = self.drive_fn(
                    ("spot", self.spots[i]), ("spot", self.spots[next_idx])
                )
                drive_times.append(r3["travel_time"])
                turn_penalties.append(r3["turn_penalty"])  # NEW

        self.max_drive = max(drive_times) if drive_times else 1.0
        self.max_walk = max(walk_times) if walk_times else 1.0
        self.max_phi = max(phis) if phis else 1.0
        self.max_turn = max(turn_penalties) if turn_penalties else 1.0  # NEW
        self.max_cross = max(cross_penalties) if cross_penalties else 1.0  # NEW

        # Guard against zero division
        if self.max_drive == 0:
            self.max_drive = 1.0
        if self.max_walk == 0:
            self.max_walk = 1.0
        if self.max_phi == 0:
            self.max_phi = 1.0
        if self.max_turn == 0:
            self.max_turn = 1.0  # NEW
        if self.max_cross == 0:
            self.max_cross = 1.0  # NEW

    def _build_topology_matrix(self, exit_mult=1.0):

        # -------- Static costs --------
        for i in range(self.n):
            walk = self.walk_fn(self.spots[i]["coords"])
            raw_exit = self.drive_fn(("spot", self.spots[i]), ("node", "ref"))[
                "travel_time"
            ]  # NEW

            self.cross_vector[i] = self.spots[i].get("cross_penalty", 0.0)  # NEW

            self.static_matrix[i] = (walk / self.max_walk) + (
                (raw_exit * exit_mult) / self.max_drive
            )

        # -------- Transition matrix --------
        for u in range(-1, self.n):
            # Determine node
            u_node = ("node", "start") if u == -1 else ("spot", self.spots[u])

            # ----- Get dual progress -----
            if u == -1:
                u_ab = self.start_progress_ab
                u_bc = self.start_progress_bc
            else:
                u_ab = self.topology[u]["progress_ab"]
                u_bc = self.topology[u]["progress_bc"]

            # ----- Phase selection -----
            if u_ab <= self.dest_progress_ab:
                active_axis = "progress_ab"
                u_prog = u_ab
            else:
                active_axis = "progress_bc"
                u_prog = u_bc

            for v in range(self.n):
                if u == v:
                    self.trans_matrix[u + 1, v] = float("inf")
                    self.turn_matrix[u + 1, v] = float("inf")  # NEW
                    continue

                v_prog = self.topology[v][active_axis]

                # # ----- Direction enforcement -----
                # if u != -1 and v_prog < u_prog:
                #     same_edge = self.spots[u].get("_edge") == self.spots[v].get("_edge")

                #     b1 = self.spots[u].get("_bearing", 0)
                #     b2 = self.spots[v].get("_bearing", 0)
                #     angle = abs((b2 - b1 + 180) % 360 - 180)

                #     # Allow small turns or same edge continuation
                #     if not (same_edge or angle < 45):
                #         self.trans_matrix[u + 1, v] = float("inf")
                #         continue

                # ----- Normal drive cost -----
                # self.trans_matrix[u + 1, v] = self.drive_fn(
                #     u_node, ("spot", self.spots[v])
                # )
                route = self.drive_fn(u_node, ("spot", self.spots[v]))
                self.trans_matrix[u + 1, v] = route["travel_time"]
                self.turn_matrix[u + 1, v] = route["turn_penalty"]  # NEW


##NEW ALGORITHM#############

# class HeuristicLookahead(BaseAlgorithm):
#     def solve(self, **kwargs):
#         k = int(kwargs.get("k", 3))
#         beam_width = 1000
#         top_k_return = 5

#         beam = [(0.0, 0.0, 1.0, -1, [], frozenset())]

#         # -------- OPTIMIZATION --------
#         for _ in range(k):
#             candidates = []

#             for score, cum_drive, fail_prob, last_idx, path, visited in beam:
#                 for i in range(self.n):
#                     if i in visited:
#                         continue

#                     raw_drive = self.trans_matrix[last_idx + 1, i]
#                     if raw_drive == float("inf"):
#                         continue

#                     p_i = self.spots[i].get("p_i", 0.5)

#                     new_cum_drive = cum_drive + raw_drive
#                     walk = self.walk_fn(self.spots[i]["coords"])
#                     exit_d = self.drive_fn(("spot", self.spots[i]), ("node", "ref"))

#                     success_time = new_cum_drive + walk + exit_d

#                     new_score = score + fail_prob * p_i * success_time
#                     new_fail_prob = fail_prob * (1 - p_i)

#                     candidates.append(
#                         (
#                             new_score,
#                             new_cum_drive,
#                             new_fail_prob,
#                             i,
#                             path + [i],
#                             visited | {i},
#                         )
#                     )

#             if not candidates:
#                 break

#             candidates.sort(key=lambda x: x[0])
#             beam = candidates[:beam_width]

#         if not beam:
#             return [], None, {}

#         # 🔥 TAKE TOP 5 CHAINS
#         top_beams = beam[:top_k_return]
#         best_chains = [b[4] for b in top_beams]

#         import routing

#         def spot_node(idx):
#             return f"spot_{self.spots[idx]['id']}"

#         def build_full_chain(best_chain):
#             full_chain = []
#             used = set()

#             if not best_chain:
#                 return []

#             # ---------- 1️⃣ Spots before first ----------
#             first = best_chain[0]
#             first_node = spot_node(first)

#             route_sf = routing.get_route(
#                 self.state["grid"].nodes["start"],
#                 first_node,
#                 custom_G=self.state["G_aug"],
#             )

#             route_nodes = route_sf["nodes"]

#             if first_node not in route_nodes:
#                 route_nodes.append(first_node)

#             idx_first = route_nodes.index(first_node)

#             before_spots = []

#             for j in range(self.n):
#                 if j == first:
#                     continue

#                 j_node = spot_node(j)

#                 if j_node in route_nodes:
#                     idx_j = route_nodes.index(j_node)

#                     if idx_j < idx_first:
#                         walk_time = self.walk_fn(self.spots[j]["coords"])
#                         if walk_time <= 300:
#                             before_spots.append((idx_j, j))

#             before_spots.sort(key=lambda x: x[0])

#             for _, j in before_spots:
#                 full_chain.append(j)
#                 used.add(j)

#             # ---------- 2️⃣ Main chain ----------
#             chain = full_chain.copy()

#             for idx in best_chain:
#                 if idx not in chain:
#                     chain.append(idx)

#             for i in range(len(chain) - 1):
#                 a = chain[i]
#                 b = chain[i + 1]

#                 if a not in used:
#                     full_chain.append(a)
#                     used.add(a)

#                 route = routing.get_route(
#                     spot_node(a),
#                     spot_node(b),
#                     custom_G=self.state["G_aug"],
#                 )

#                 route_nodes = route["nodes"]

#                 route_edges = set()
#                 for u, v in zip(route_nodes[:-1], route_nodes[1:]):
#                     route_edges.add((u, v))
#                     route_edges.add((v, u))

#                 between_spots = []

#                 for j in range(self.n):
#                     if j in used:
#                         continue

#                     j_node = spot_node(j)

#                     if j_node in route_nodes:
#                         idx_j = route_nodes.index(j_node)
#                         between_spots.append((idx_j, j))
#                         continue

#                     u, v, _ = self.spots[j]["_edge"]
#                     if (u, v) in route_edges:
#                         if u in route_nodes:
#                             idx_j = route_nodes.index(u)
#                         elif v in route_nodes:
#                             idx_j = route_nodes.index(v)
#                         else:
#                             continue

#                         between_spots.append((idx_j, j))

#                 between_spots.sort(key=lambda x: x[0])

#                 for _, j in between_spots:
#                     full_chain.append(j)
#                     used.add(j)

#             # ---------- 3️⃣ Add last ----------
#             last = best_chain[-1]
#             if last not in used:
#                 full_chain.append(last)

#             return full_chain

#         # 🔥 BUILD ALL FULL CHAINS
#         full_chains = []
#         metrics_list = []

#         for chain in best_chains:
#             fc = build_full_chain(chain)
#             full_chains.append(fc)
#             metrics_list.append(calculate_metrics(fc, self.state))

#         return full_chains, metrics_list, {}


class HeuristicLookahead(BaseAlgorithm):
    def solve(self, **kwargs):
        k = int(kwargs.get("k", 3))
        beam_width = 1000

        beam = [(0.0, 0.0, 1.0, -1, [], frozenset())]

        # -------- OPTIMIZATION --------
        for _ in range(k):
            candidates = []

            for score, cum_drive, fail_prob, last_idx, path, visited in beam:
                for i in range(self.n):
                    if i in visited:
                        continue

                    raw_drive = self.trans_matrix[last_idx + 1, i]
                    if raw_drive == float("inf"):
                        continue

                    p_i = self.spots[i].get("p_i", 0.5)

                    new_cum_drive = cum_drive + raw_drive
                    walk = self.walk_fn(self.spots[i]["coords"])
                    exit_d = self.drive_fn(("spot", self.spots[i]), ("node", "ref"))

                    success_time = new_cum_drive + walk + exit_d

                    new_score = score + fail_prob * p_i * success_time
                    new_fail_prob = fail_prob * (1 - p_i)

                    candidates.append(
                        (
                            new_score,
                            new_cum_drive,
                            new_fail_prob,
                            i,
                            path + [i],
                            visited | {i},
                        )
                    )

            if not candidates:
                break

            candidates.sort(key=lambda x: x[0])
            beam = candidates[:beam_width]

        best_chain = beam[0][4] if beam else []

        # -------- POST-PROCESS --------
        import routing

        def spot_node(idx):
            return f"spot_{self.spots[idx]['id']}"

        full_chain = []
        used = set()

        # # 1️⃣ Add spots BEFORE first chosen
        # first = best_chain[0]
        # edge_first = self.spots[first].get("_edge")
        # prog_first = self.topology[first]["progress_ab"]

        # drive_first = self.drive_fn(("node", "start"), ("spot", self.spots[first]))

        # before_spots = []

        # for j in range(self.n):
        #     if j == first:
        #         continue

        #     if self.spots[j].get("_edge") == edge_first:
        #         prog_j = self.topology[j]["progress_ab"]

        #         if prog_j < prog_first:
        #             drive_j = self.drive_fn(("node", "start"), ("spot", self.spots[j]))

        #             if drive_j <= drive_first:
        #                 before_spots.append(j)

        # before_spots.sort(key=lambda j: self.topology[j]["progress_ab"])

        # for j in before_spots:
        #     full_chain.append(j)
        #     used.add(j)
        # 1️⃣ Add spots BEFORE first chosen (path-based, no drive calls)
        if not best_chain:
            return [], calculate_metrics([], self.state), {}
        first = best_chain[0]
        first_node = spot_node(first)

        route_sf = routing.get_route(
            self.state["grid"].nodes["start"],
            first_node,
            custom_G=self.state["G_aug"],
        )

        route_nodes = route_sf["nodes"]

        if first_node not in route_nodes:
            route_nodes.append(first_node)

        idx_first = route_nodes.index(first_node)

        before_spots = []

        for j in range(self.n):
            if j == first:
                continue

            j_node = spot_node(j)

            if j_node in route_nodes:
                idx_j = route_nodes.index(j_node)

                # must lie before first on path
                if idx_j < idx_first:
                    walk_time = self.walk_fn(self.spots[j]["coords"])

                    # walk ≤ 7 minutes (420 sec)
                    if walk_time <= 300:
                        before_spots.append((idx_j, j))

        # sort by real order along route
        before_spots.sort(key=lambda x: x[0])

        for _, j in before_spots:
            full_chain.append(j)
            used.add(j)

        # 2️⃣ Add chosen spots + between spots
        # for i in range(len(best_chain) - 1):
        #     a = best_chain[i]
        #     b = best_chain[i + 1]

        #     if a not in used:
        #         full_chain.append(a)
        #         used.add(a)

        #     route = routing.get_route(
        #         spot_node(a),
        #         spot_node(b),
        #         custom_G=self.state["G_aug"],
        #     )

        # route_nodes = route["nodes"]

        # between_spots = [
        #     j
        #     for j in range(self.n)
        #     if j not in used and spot_node(j) in route_nodes
        # ]
        # route_nodes = route["nodes"]

        # between_spots.sort(key=lambda j: route_nodes.index(spot_node(j)))

        # for j in between_spots:
        #     full_chain.append(j)
        #     used.add(j)
        # route_nodes = route["nodes"]

        # route_edges = set()
        # for u, v in zip(route_nodes[:-1], route_nodes[1:]):
        #     route_edges.add((u, v))
        #     route_edges.add((v, u))

        # between_spots = [
        #     j
        #     for j in range(self.n)
        #     if j not in used
        #     and (
        #         spot_node(j) in route_nodes
        #         or self.spots[j].get("_edge") in route_edges
        #     )
        # ]

        # def route_position(j):
        #     if spot_node(j) in route_nodes:
        #         return route_nodes.index(spot_node(j))
        #     u, v = self.spots[j]["_edge"]
        #     if u in route_nodes:
        #         return route_nodes.index(u)
        #     if v in route_nodes:
        #         return route_nodes.index(v)
        #     return float("inf")

        # between_spots.sort(key=route_position)

        # for j in between_spots:
        #     full_chain.append(j)
        #     used.add(j)
        # for i in range(len(best_chain) - 1):
        #     a = best_chain[i]
        #     b = best_chain[i + 1]

        #     if a not in used:
        #         full_chain.append(a)
        #         used.add(a)

        #     route = routing.get_route(
        #         spot_node(a),
        #         spot_node(b),
        #         custom_G=self.state["G_aug"],
        #     )

        #     route_nodes = route["nodes"]

        #     # collect spots that lie on this path
        #     between_spots = []

        #     for j in range(self.n):
        #         if j in used:
        #             continue

        #         j_node = spot_node(j)

        #         if j_node in route_nodes:
        #             idx_j = route_nodes.index(j_node)
        #             between_spots.append((idx_j, j))

        #     # sort by real order along route
        #     between_spots.sort(key=lambda x: x[0])

        #     for _, j in between_spots:
        #         full_chain.append(j)
        #         used.add(j)

        ############### LAST WORKED PART ########################
        chain = full_chain.copy()

        for idx in best_chain:
            if idx not in chain:
                chain.append(idx)

        for i in range(len(chain) - 1):
            a = chain[i]
            b = chain[i + 1]

            if a not in used:
                full_chain.append(a)
                used.add(a)

            route = routing.get_route(
                spot_node(a),
                spot_node(b),
                custom_G=self.state["G_aug"],
            )

            route_nodes = route["nodes"]

            # collect all edges of this path
            route_edges = set()
            for u, v in zip(route_nodes[:-1], route_nodes[1:]):
                route_edges.add((u, v))
                route_edges.add((v, u))

            between_spots = []

            for j in range(self.n):
                if j in used:
                    continue

                j_node = spot_node(j)

                # check if spot node is directly on path
                if j_node in route_nodes:
                    idx_j = route_nodes.index(j_node)
                    between_spots.append((idx_j, j))
                    continue

                # check if spot lies on one of the path edges
                u, v, _ = self.spots[j]["_edge"]
                if (u, v) in route_edges:
                    # approximate position by first occurrence of u or v
                    if u in route_nodes:
                        idx_j = route_nodes.index(u)
                    elif v in route_nodes:
                        idx_j = route_nodes.index(v)
                    else:
                        continue

                    between_spots.append((idx_j, j))

            # sort by order along path
            between_spots.sort(key=lambda x: x[0])

            for _, j in between_spots:
                full_chain.append(j)
                used.add(j)

        # 3️⃣ Add last chosen
        if best_chain:
            last = best_chain[-1]
            if last not in used:
                full_chain.append(last)

        return full_chain, calculate_metrics(full_chain, self.state), {}
        # 2️⃣ Build base chain (before + best_chain)

        # chain = full_chain.copy()

        # for idx in best_chain:
        #     if idx not in chain:
        #         chain.append(idx)

        # used = set(chain)

        # # 3️⃣ Dynamically insert between spots

        # i = 0
        # while i < len(chain) - 1:
        #     a = chain[i]
        #     b = chain[i + 1]

        #     route = routing.get_route(
        #         spot_node(a),
        #         spot_node(b),
        #         custom_G=self.state["G_aug"],
        #     )

        #     route_nodes = route["nodes"]

        #     candidates = []

        #     for j in range(self.n):
        #         if j in used:
        #             continue

        #         j_node = spot_node(j)

        #         if j_node in route_nodes:
        #             idx_j = route_nodes.index(j_node)
        #             candidates.append((idx_j, j))

        #     # sort by order along route
        #     candidates.sort(key=lambda x: x[0])

        #     if candidates:
        #         for offset, (_, j) in enumerate(candidates):
        #             chain.insert(i + 1 + offset, j)
        #             used.add(j)
        #     else:
        #         i += 1

        # return chain, calculate_metrics(chain, self.state), {}


class MDP_Difference(BaseAlgorithm):
    def solve(self, **kwargs):
        k, lw, le, ltr, lturn_arr, lturn_exit, lcross = (
            int(kwargs.get("k", 3)),
            kwargs.get("lambda_w", 1.0),
            kwargs.get("lambda_e", 1.0),
            kwargs.get("lambda_tr", 1.0),
            kwargs.get("lambda_turns_arr", 1.0),
            kwargs.get("lambda_turns_exit", 1.0),  # NEW
            kwargs.get("lambda_cross", 1.0),
        )
        beam = [(0.0, 1.0, -1, [], frozenset())]
        ############ CHANGE ###############
        # first_idx = min(
        #     range(self.n), key=lambda i: self.walk_fn(self.spots[i]["coords"])
        # )

        # beam = [(0.0, 1.0, first_idx, [first_idx], {first_idx})]
        best_chain = []  # (cost_so_far, fail_probability, current_spot, path_taken, visited_spots)

        for _ in range(k):
            candidates = []
            for cost_so_far, fail_prob, curr, path, visited in beam:
                # Search penalty of failed spot normalized by M_phi
                phi_prev = self._get_phi(curr) if curr >= 0 else 0
                norm_phi_prev = phi_prev / self.max_phi

                for i in range(self.n):
                    raw_drive = self.trans_matrix[
                        curr + 1, i
                    ]  # driving time from current position to spot  (curr +1 means current position in our matrix)
                    raw_turn = self.turn_matrix[curr + 1, i]  # NEW
                    # if raw_drive == float("inf") or i in visited:
                    #     continue
                    if (
                        raw_drive == float("inf")
                        or raw_turn == float("inf")
                        or i in visited
                    ):
                        continue

                    p_i = self.spots[i].get("p_i", 0.8)
                    walk = self.walk_fn(self.spots[i]["coords"])
                    # 🔹 First choice constraint: must be within 3 min walking
                    if curr == -1:
                        if walk > 180:
                            continue
                    r = self.drive_fn(("spot", self.spots[i]), ("node", "ref"))
                    exit_d = r["travel_time"]  # NEW
                    exit_turn = r["turn_penalty"]
                    raw_cross = self.cross_vector[i]  # NEW

                    # Transition: Phi_prev + Drive(u, v)
                    drive_w = ltr if curr >= 0 else 1.0
                    norm_drive = (drive_w * raw_drive) / self.max_drive
                    norm_turn = (lturn_arr * raw_turn) / self.max_turn  # NEW
                    norm_cross = (lcross * raw_cross) / self.max_cross  # NEW
                    norm_exit_turn = (lturn_exit * exit_turn) / self.max_turn
                    # norm_step = (
                    #     norm_drive + norm_turn + norm_phi_prev  # NEW
                    # )  # показывает нормализованную стоимость перехода к следующей парковке

                    # # Quality: Normalized by respective M values
                    # norm_quality = (lw * (walk / self.max_walk)) + (
                    #     le * (exit_d / self.max_drive)
                    # )  # нормализованная + взвешенная
                    # q = (
                    #     norm_step + norm_quality
                    # )  # это стоимость добавить эту парковку как следующий шаг поиска
                    arrival_cost = norm_drive + norm_turn + norm_phi_prev
                    success_cost = (lw * (walk / self.max_walk)) + (
                        le * (exit_d / self.max_drive) + norm_cross + norm_exit_turn
                    )

                    expected_increment = (
                        fail_prob * arrival_cost + fail_prob * p_i * success_cost
                    )

                    candidates.append(
                        (
                            # cost_so_far + fail_prob * p_i * q,
                            cost_so_far + expected_increment,
                            fail_prob * (1.0 - p_i),
                            i,
                            path + [i],
                            visited | {i},
                        )
                    )
                    ########## CHANGE ################
                    # c_fail = (norm_drive + norm_phi_prev) + (phi_prev / self.max_phi)

                    # candidates.append(
                    #     (
                    #         cost_so_far + fail_prob * (p_i * q + (1.0 - p_i) * c_fail),
                    #         fail_prob * (1.0 - p_i),
                    #         i,
                    #         path + [i],
                    #         visited | {i},
                    #     )
                    # )
                    ########### CHANGE ################

            candidates.sort(key=lambda x: x[0])
            # beam = candidates[:1000]
            beam = candidates[:1000]
            if beam:
                best_chain = beam[0][3]
            import routing

            def spot_node(idx):
                return f"spot_{self.spots[idx]['id']}"

            full_chain = []
            used = set()

            # 1️⃣ Add spots BEFORE first chosen (path-based, no drive calls)
            if not best_chain:
                return [], calculate_metrics([], self.state), {}
            first = best_chain[0]
            first_node = spot_node(first)

            route_sf = routing.get_route(
                self.state["grid"].nodes["start"],
                first_node,
                custom_G=self.state["G_aug"],
            )

            route_nodes = route_sf["nodes"]

            if first_node not in route_nodes:
                route_nodes.append(first_node)

            idx_first = route_nodes.index(first_node)

            before_spots = []

            for j in range(self.n):
                if j == first:
                    continue

                j_node = spot_node(j)

                if j_node in route_nodes:
                    idx_j = route_nodes.index(j_node)

                    # must lie before first on path
                    if idx_j < idx_first:
                        walk_time = self.walk_fn(self.spots[j]["coords"])

                        # walk ≤ 7 minutes (420 sec)
                        if walk_time <= 300:
                            before_spots.append((idx_j, j))

            # sort by real order along route
            before_spots.sort(key=lambda x: x[0])

            for _, j in before_spots:
                full_chain.append(j)
                used.add(j)

            ############### LAST WORKED PART ########################
            chain = full_chain.copy()

            for idx in best_chain:
                if idx not in chain:
                    chain.append(idx)

            for i in range(len(chain) - 1):
                a = chain[i]
                b = chain[i + 1]

                if a not in used:
                    full_chain.append(a)
                    used.add(a)

                route = routing.get_route(
                    spot_node(a),
                    spot_node(b),
                    custom_G=self.state["G_aug"],
                )

                route_nodes = route["nodes"]

                # collect all edges of this path
                route_edges = set()
                for u, v in zip(route_nodes[:-1], route_nodes[1:]):
                    route_edges.add((u, v))
                    route_edges.add((v, u))

                between_spots = []

                for j in range(self.n):
                    if j in used:
                        continue

                    j_node = spot_node(j)

                    # check if spot node is directly on path
                    if j_node in route_nodes:
                        idx_j = route_nodes.index(j_node)
                        between_spots.append((idx_j, j))
                        continue

                    # check if spot lies on one of the path edges
                    u, v, _ = self.spots[j]["_edge"]
                    if (u, v) in route_edges:
                        # approximate position by first occurrence of u or v
                        if u in route_nodes:
                            idx_j = route_nodes.index(u)
                        elif v in route_nodes:
                            idx_j = route_nodes.index(v)
                        else:
                            continue

                        between_spots.append((idx_j, j))

                # sort by order along path
                between_spots.sort(key=lambda x: x[0])

                for _, j in between_spots:
                    full_chain.append(j)
                    used.add(j)

            # 3️⃣ Add last chosen
            if best_chain:
                last = best_chain[-1]
                if last not in used:
                    full_chain.append(last)

        return full_chain, calculate_metrics(full_chain, self.state), {}
        # return best_chain, calculate_metrics(best_chain, self.state), {}


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


# class HeuristicLookahead(BaseAlgorithm):
#     def solve(self, **kwargs):
#         k, lw, le, ltr = (
#             int(kwargs.get("k", 3)),
#             kwargs.get("lambda_w", 1.0),
#             kwargs.get("lambda_e", 1.0),
#             kwargs.get("lambda_tr", 1.0),
#         )
#         chain, curr = [], -1
#         for _ in range(k):
#             best_i, min_q = -1, float("inf")
#             phi_prev = self._get_phi(curr) if curr >= 0 else 0
#             for i in range(self.n):
#                 raw_drive = self.trans_matrix[curr + 1, i]
#                 if raw_drive == float("inf") or i in chain:
#                     continue
#                 p_i = self.spots[i].get("p_i", 0.8)
#                 walk, exit_d = (
#                     self.walk_fn(self.spots[i]["coords"]),
#                     self.drive_fn(("spot", self.spots[i]), ("node", "ref")),
#                 )
#                 drive_w = ltr if curr >= 0 else 1.0
#                 n_step = ((drive_w * raw_drive) / self.max_drive) + (
#                     phi_prev / self.max_phi
#                 )
#                 n_quality = (
#                     (lw * walk / self.max_walk)
#                     + (le * exit_d / self.max_drive)
#                     + (self._get_phi(i) / self.max_phi)
#                 )
#                 q = n_step + (n_quality / max(p_i, 0.1))
#                 if q < min_q:
#                     min_q, best_i = q, i
#             if best_i == -1:
#                 break
#             chain.append(best_i)
#             curr = best_i
#         return chain, calculate_metrics(chain, self.state), {}
