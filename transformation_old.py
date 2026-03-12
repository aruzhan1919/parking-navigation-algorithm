import json
import os
from geopy.distance import geodesic
import math
import routing


class StateAdapter:
    def __init__(
        self, start, dest, ref, spots, traffic_multiplier=1.4, distance_cache=None
    ):
        self.start, self.dest, self.ref = tuple(start), tuple(dest), tuple(ref)
        self.spots = spots
        self.traffic_multiplier = traffic_multiplier
        # This cache now stores BASE travel time (without traffic multiplier)
        self.distance_cache = distance_cache if distance_cache is not None else {}

        # Astana Longitude Scaling (51.1N)
        self.lng_scale = math.cos(math.radians(self.start[0]))
        self.axis_b = self.ref

        self.spot_topology = []
        self.node_cache = {
            "start": self._map_to_avenue(self.start),
            "ref": self._map_to_avenue(self.ref),
            "dest": self._map_to_avenue(self.dest),
        }

        for s in self.spots:
            progress, side_val = self._map_to_avenue(s["coords"])
            s["_cached_topo"] = (progress, side_val)
            self.spot_topology.append(
                {"progress": progress, "side": "LEFT" if side_val > 0 else "RIGHT"}
            )

        self.start_progress = self.node_cache["start"][0]
        print(
            f"[Topology] Mapped {len(self.spots)} spots. Cache loaded with {len(self.distance_cache)} routes."
        )

    def _map_to_avenue(self, point):
        yA, xA = self.start[0], self.start[1] * self.lng_scale
        yB, xB = self.axis_b[0], self.axis_b[1] * self.lng_scale
        yP, xP = point[0], point[1] * self.lng_scale

        side_val = (xP - xA) * (yB - yA) - (yP - yA) * (xB - xA)
        dx_ab, dy_ab = xB - xA, yB - yA
        dx_ap, dy_ap = xP - xA, yP - yA
        mag_ab = math.sqrt(dx_ab**2 + dy_ab**2)
        if mag_ab == 0:
            return 0, 0
        progress = (dx_ap * dx_ab + dy_ap * dy_ab) / mag_ab
        return progress, side_val

    def _drive_fn(self, u_ent, v_ent):
        """
        Calculates drive time with Geometric Traffic Constraints.
        """
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

        # --- CACHE KEY ---
        cache_key = str((tuple(u_coords), tuple(v_coords)))

        base_time = 0
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

        final_duration = base_time * self.traffic_multiplier

        # --- ASYMMETRY ENFORCER ---
        side_u = "LEFT" if side_u_val > 0 else "RIGHT"
        side_v = "LEFT" if side_v_val > 0 else "RIGHT"

        penalty = 0.0
        # If moving backward on the same side (Backtracking)
        if side_u == side_v and prog_v < prog_u - 0.00001:
            penalty = 1800.0  # 30 min penalty
        # If switching sides (Crossing the avenue)
        elif side_u != side_v:
            penalty = 1200.0  # 20 min penalty

        return final_duration + penalty

    def _walk_fn(self, spot_coords):
        return geodesic(spot_coords, self.dest).meters / 1.3

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
        }
