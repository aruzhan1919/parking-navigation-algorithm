import json  # Read/write structured data (spots, configs, results)
import os
from geopy.distance import geodesic  # Compute real-world distances between coordinates
import math
import routing  # Project routing logic (paths, travel times, graph access)


class StateAdapter:
    def __init__(
        self,
        start,
        dest,
        ref,
        spots,
        traffic_multiplier=1.4,
        distance_cache=None,
    ):
        if routing.G is None:
            routing.initialize_graph()

        self.start, self.dest, self.ref = (
            tuple(start),
            tuple(dest),
            tuple(ref),
        )

        self.spots = spots
        self.traffic_multiplier = traffic_multiplier
        self.distance_cache = distance_cache if distance_cache is not None else {}

        self.lng_scale = math.cos(math.radians(self.start[0]))

        # Axis definitions
        self.axis_b = self.dest  # Start → Dest
        self.axis_c = self.ref  # Dest → Ref

        self.spot_topology = []

        # ---------- FIX 1: unpack 3 values ----------
        start_ab, start_bc, _ = self._map_to_avenue(self.start)
        dest_ab, dest_bc, _ = self._map_to_avenue(self.dest)
        ref_ab, ref_bc, _ = self._map_to_avenue(self.ref)

        self.node_cache = {
            "start": {"progress_ab": start_ab, "progress_bc": start_bc},
            "dest": {"progress_ab": dest_ab, "progress_bc": dest_bc},
            "ref": {"progress_ab": ref_ab, "progress_bc": ref_bc},
        }

        # ---------- FIX 2: store both projections per spot ----------
        for s in self.spots:
            progress_ab, progress_bc, side_val = self._map_to_avenue(s["coords"])

            s["_cached_topo"] = {
                "progress_ab": progress_ab,
                "progress_bc": progress_bc,
                "side": side_val,
            }

            self.spot_topology.append(
                {
                    "progress_ab": progress_ab,
                    "progress_bc": progress_bc,
                    "side": "LEFT" if side_val > 0 else "RIGHT",
                }
            )

            # crossings = routing.count_crossings(
            #     s["coords"],
            #     dest,
            # )
            crossings = routing.count_crossings(
                s["coords"],
                self.dest,
                routing.G,
                ignore_edge=s.get("_edge"),
            )

            s["crossings"] = crossings
            s["cross_penalty"] = crossings * 15

        print(
            f"[Topology] Mapped {len(self.spots)} spots. Cache loaded with {len(self.distance_cache)} routes."
        )

    # -----------------------------------------------------

    def _map_to_avenue(self, point):
        yA, xA = self.start[0], self.start[1] * self.lng_scale
        yB, xB = self.axis_b[0], self.axis_b[1] * self.lng_scale
        yP, xP = point[0], point[1] * self.lng_scale

        # ---------- Start → Dest projection ----------
        side_val = (xP - xA) * (yB - yA) - (yP - yA) * (xB - xA)

        dx_ab, dy_ab = xB - xA, yB - yA
        dx_ap, dy_ap = xP - xA, yP - yA
        mag_ab = math.sqrt(dx_ab**2 + dy_ab**2)

        progress_ab = 0
        if mag_ab > 0:
            progress_ab = (dx_ap * dx_ab + dy_ap * dy_ab) / mag_ab

        # ---------- Dest → Ref projection ----------
        yC, xC = self.axis_c[0], self.axis_c[1] * self.lng_scale

        dx_bc, dy_bc = xC - xB, yC - yB
        dx_bp, dy_bp = xP - xB, yP - yB
        mag_bc = math.sqrt(dx_bc**2 + dy_bc**2)

        progress_bc = 0
        if mag_bc > 0:
            progress_bc = (dx_bp * dx_bc + dy_bp * dy_bc) / mag_bc

        return progress_ab, progress_bc, side_val

    # -----------------------------------------------------

    def _walk_fn(self, spot_coords):
        return geodesic(spot_coords, self.dest).meters / 1.3

    # -----------------------------------------------------

    def get_state(self):
        return {
            "spots": self.spots,
            "start_node": "start",
            "dest_coords": self.dest,
            "destination_C": self.ref,
            "walk_fn": self._walk_fn,
            "grid": type("Mock", (), {"nodes": {"start": self.start, "ref": self.ref}}),
            "topology": self.spot_topology,
            # ---------- FIX 3: expose both axes ----------
            "start_progress_ab": self.node_cache["start"]["progress_ab"],
            "dest_progress_ab": self.node_cache["dest"]["progress_ab"],
            "ref_progress_ab": self.node_cache["ref"]["progress_ab"],
            "start_progress_bc": self.node_cache["start"]["progress_bc"],
            "dest_progress_bc": self.node_cache["dest"]["progress_bc"],
            "ref_progress_bc": self.node_cache["ref"]["progress_bc"],
        }
