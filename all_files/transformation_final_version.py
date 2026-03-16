import json  # Read/write structured data (spots, configs, results)
import os
from geopy.distance import geodesic  # Compute real-world distances between coordinates
import math
import routing  # Project routing logic (paths, travel times, graph access)


class StateAdapter:
    def __init__(
        self, start, dest, ref, spots, traffic_multiplier=1.4, distance_cache=None
    ):
        self.start, self.dest, self.ref = (
            tuple(start),
            tuple(dest),
            tuple(ref),
        )  # coordinates saved in tuples to avoid changes
        self.spots = spots
        self.traffic_multiplier = traffic_multiplier
        # This cache now stores BASE travel time (without traffic multiplier)
        self.distance_cache = (
            distance_cache if distance_cache is not None else {}
        )  # distance cashe is given as dict
        # Key = pair of coordinates; Value = base travel time

        # Astana Longitude Scaling (51.1N)
        self.lng_scale = math.cos(math.radians(self.start[0]))
        self.axis_b = self.ref  # for the direction of movement

        self.spot_topology = []
        self.node_cache = {
            "start": self._map_to_avenue(self.start),  # 0,0
            "ref": self._map_to_avenue(self.ref),  # |AB|, 0
            "dest": self._map_to_avenue(self.dest),
        }

        for s in self.spots:
            progress, side_val = self._map_to_avenue(s["coords"])  # for point P
            s["_cached_topo"] = (
                progress,
                side_val,
            )  # cached topo for spot (creates new key and value)
            self.spot_topology.append(
                {"progress": progress, "side": "LEFT" if side_val > 0 else "RIGHT"}
            )

        self.start_progress = self.node_cache["start"][0]
        print(
            f"[Topology] Mapped {len(self.spots)} spots. Cache loaded with {len(self.distance_cache)} routes."
        )

    def _map_to_avenue(self, point):
        # yA, xA = self.start[0], self.start[1] * self.lng_scale  # point of start
        ######### CHANGE ############
        yA, xA = self.dest[0], self.dest[1] * self.lng_scale
        yB, xB = self.axis_b[0], self.axis_b[1] * self.lng_scale  # point of ref
        yP, xP = point[0], point[1] * self.lng_scale  # point of parking

        side_val = (xP - xA) * (yB - yA) - (yP - yA) * (
            xB - xA
        )  # determinant of AP, AB (to the LEFT or RIGHT from the road to AB)
        dx_ab, dy_ab = xB - xA, yB - yA  # A->B
        dx_ap, dy_ap = xP - xA, yP - yA  # A->P
        mag_ab = math.sqrt(dx_ab**2 + dy_ab**2)  # magnitutde of AB |AB|
        if mag_ab == 0:
            return 0, 0
        progress = (dx_ap * dx_ab + dy_ap * dy_ab) / mag_ab  # AP*AB/|AB|
        return progress, side_val

    ############ CHANGE #################
    ############ CHANGE #################

    def _bearing(self, lat1, lon1, lat2, lon2):
        dLon = math.radians(lon2 - lon1)
        lat1 = math.radians(lat1)
        lat2 = math.radians(lat2)

        y = math.sin(dLon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(
            lat2
        ) * math.cos(dLon)

        brng = math.degrees(math.atan2(y, x))
        return (brng + 360) % 360

    ############ CHANGE #################

    ############ CHANGE #################
    def _drive_fn(self, u_ent, v_ent):

        # --- TURN PENALTIES (seconds) ---
        COST_U_TURN = 300.0
        COST_LEFT = 45.0
        COST_RIGHT = 10.0
        COST_STRAIGHT = 0.0

        # --- EXTRACT COORDS ---
        if u_ent[0] == "node":
            u_coords = self.start if u_ent[1] == "start" else self.ref
            prog_u, side_u_val = self.node_cache.get(u_ent[1], (0, 0))
        else:
            u_coords = u_ent[1]["coords"]
            prog_u, side_u_val = u_ent[1].get("_cached_topo", (0, 0))

        if v_ent[0] == "node":
            v_coords = self.ref if v_ent[1] == "ref" else self.start
            prog_v, side_v_val = self.node_cache.get(v_ent[1], (0, 0))
        else:
            v_coords = v_ent[1]["coords"]
            prog_v, side_v_val = v_ent[1].get("_cached_topo", (0, 0))

        cache_key = str((tuple(u_coords), tuple(v_coords)))

        # ======================================================
        # 1️⃣ GET OR COMPUTE BASE TRAVEL TIME (NO TURNS)
        # ======================================================
        route_data = None

        if cache_key in self.distance_cache:
            base_time = self.distance_cache[cache_key]
        else:
            route_data = routing.get_route(u_coords, v_coords)

            if route_data["travel_time"] == float("inf"):
                dist = geodesic(u_coords, v_coords).meters
                base_time = (dist / 8.33) + 30.0
            else:
                base_time = route_data["travel_time"]

            base_time = max(1.0, base_time)
            self.distance_cache[cache_key] = base_time

        # ======================================================
        # 2️⃣ TURN PENALTY (NOT CACHED)
        # ======================================================
        turn_penalty = 0.0

        if route_data is None:
            route_data = routing.get_route(u_coords, v_coords)

        if route_data["travel_time"] != float("inf"):
            nodes = route_data.get("nodes", [])

            for i in range(1, len(nodes) - 1):
                n1, n2, n3 = nodes[i - 1], nodes[i], nodes[i + 1]

                lat1 = routing.G.nodes[n1]["y"]
                lon1 = routing.G.nodes[n1]["x"]
                lat2 = routing.G.nodes[n2]["y"]
                lon2 = routing.G.nodes[n2]["x"]
                lat3 = routing.G.nodes[n3]["y"]
                lon3 = routing.G.nodes[n3]["x"]

                b1 = self._bearing(lat1, lon1, lat2, lon2)
                b2 = self._bearing(lat2, lon2, lat3, lon3)

                angle = (b2 - b1 + 180) % 360 - 180

                if abs(angle) < 20:
                    turn_penalty += COST_STRAIGHT
                elif abs(angle) > 150:
                    turn_penalty += COST_U_TURN
                elif angle > 0:
                    turn_penalty += COST_RIGHT
                else:
                    turn_penalty += COST_LEFT

        # ======================================================
        # 3️⃣ APPLY TRAFFIC MULTIPLIER
        # ======================================================
        final_duration = (base_time + turn_penalty) * self.traffic_multiplier

        # ======================================================
        # 4️⃣ GEOMETRIC PENALTIES
        # ======================================================
        side_u = "LEFT" if side_u_val > 0 else "RIGHT"
        side_v = "LEFT" if side_v_val > 0 else "RIGHT"

        geometric_penalty = 0.0

        if side_u == side_v and prog_v < prog_u - 0.00001:
            geometric_penalty = 1800.0
        elif side_u != side_v:
            geometric_penalty = 1200.0

        return final_duration + geometric_penalty

    def _walk_fn(self, spot_coords):
        return (
            geodesic(spot_coords, self.dest).meters / 1.3
        )  # 1.3 m/s = average human walking speed

    def get_state(self):
        return {
            "spots": self.spots,
            "start_node": "start",
            "dest_coords": self.dest,
            "destination_C": self.ref,
            "drive_fn": self._drive_fn,
            "walk_fn": self._walk_fn,
            "grid": type("Mock", (), {"nodes": {"start": self.start, "ref": self.ref}}),
            "topology": self.spot_topology,
            "start_progress": self.start_progress,
            "ref_progress": self.node_cache["ref"][0],
            "dest_progress": self.node_cache["dest"][0],
        }
