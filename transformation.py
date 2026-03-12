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
        yA, xA = self.start[0], self.start[1] * self.lng_scale  # point of start
        ######### CHANGE ############
        # yA, xA = self.dest[0], self.dest[1] * self.lng_scale
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
            "walk_fn": self._walk_fn,
            "grid": type("Mock", (), {"nodes": {"start": self.start, "ref": self.ref}}),
            "topology": self.spot_topology,
            "start_progress": self.start_progress,
            "ref_progress": self.node_cache["ref"][0],
            "dest_progress": self.node_cache["dest"][0],
        }