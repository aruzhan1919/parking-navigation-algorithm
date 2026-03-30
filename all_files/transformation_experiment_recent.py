"""
transformation.py
=================
Transforms raw input (start, dest, ref, spots) into a structured state dict
ready for the optimization algorithms.

Key responsibilities:
1. Project all spots and key points onto two axes (Start→Dest, Dest→Ref)
   to get progress values used for topology ordering
2. Precompute pedestrian crossing counts for each spot
3. Expose walk_fn (geodesic distance / 1.3 m/s)
4. Build and return state dict consumed by BaseAlgorithm

NOT YET IMPLEMENTED (planned):
- Walk graph routing (real pedestrian paths instead of straight-line distance)
- Distance cache integration
"""

import math

from geopy.distance import geodesic

import routing


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
        """
        Initialize adapter and precompute all spot topology data.

        Args:
            start: (lat, lon) — current driver position
            dest:  (lat, lon) — destination (walk target after parking)
            ref:   (lat, lon) — next route point (exit convenience reference)
            spots: list of spot dicts (must already have _edge from augment_graph_with_spots)
            traffic_multiplier: float — reserved, not used in walk_fn
            distance_cache: dict — precomputed routes cache (passed from app.py)

        Side effects on each spot dict:
            - crossings: int — number of streets to cross on foot to dest
            - cross_penalty: int — crossings * 15 seconds
            - _cached_topo: dict — progress_ab, progress_bc, side values
        """
        if routing.G is None:
            routing.initialize_graph()

        self.start = tuple(start)
        self.dest = tuple(dest)
        self.ref = tuple(ref)

        self.spots = spots
        self.traffic_multiplier = traffic_multiplier
        self.distance_cache = distance_cache if distance_cache is not None else {}

        # Scale longitude differences by cos(lat) for approximate Euclidean projection
        self.lng_scale = math.cos(math.radians(self.start[0]))

        # Two axes for dual-axis topology projection
        self.axis_b = self.dest  # axis AB: Start → Dest
        self.axis_c = self.ref  # axis BC: Dest → Ref

        # Precompute progress values for start, dest, ref
        start_ab, start_bc, _ = self._map_to_avenue(self.start)
        dest_ab, dest_bc, _ = self._map_to_avenue(self.dest)
        ref_ab, ref_bc, _ = self._map_to_avenue(self.ref)

        self.node_cache = {
            "start": {"progress_ab": start_ab, "progress_bc": start_bc},
            "dest": {"progress_ab": dest_ab, "progress_bc": dest_bc},
            "ref": {"progress_ab": ref_ab, "progress_bc": ref_bc},
        }

        # Precompute topology and crossings for each spot
        self.spot_topology = []

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

            # Count streets the pedestrian must cross walking from spot to dest
            crossings = routing.count_crossings(
                s["coords"],
                self.dest,
                routing.G,
                ignore_edge=s.get("_edge"),
            )
            s["crossings"] = crossings
            s["cross_penalty"] = crossings * 15  # 15 seconds per crossing

        print(
            f"[Topology] Mapped {len(self.spots)} spots. Cache loaded with {len(self.distance_cache)} routes."
        )

    def _map_to_avenue(self, point):
        """
        Project a point onto two axes:
          - AB axis: Start → Dest (progress_ab)
          - BC axis: Dest → Ref  (progress_bc)

        Uses approximate 2D Euclidean projection with longitude scaling.

        Args:
            point: (lat, lon)

        Returns:
            (progress_ab, progress_bc, side_val)
            - progress_ab: signed scalar projection onto Start→Dest axis
            - progress_bc: signed scalar projection onto Dest→Ref axis
            - side_val: cross product sign — positive = left of AB axis
        """
        yA, xA = self.start[0], self.start[1] * self.lng_scale
        yB, xB = self.axis_b[0], self.axis_b[1] * self.lng_scale
        yP, xP = point[0], point[1] * self.lng_scale

        # Cross product for side determination (left vs right of AB)
        side_val = (xP - xA) * (yB - yA) - (yP - yA) * (xB - xA)

        # Project point onto AB axis
        dx_ab, dy_ab = xB - xA, yB - yA
        dx_ap, dy_ap = xP - xA, yP - yA
        mag_ab = math.sqrt(dx_ab**2 + dy_ab**2)
        progress_ab = (dx_ap * dx_ab + dy_ap * dy_ab) / mag_ab if mag_ab > 0 else 0

        # Project point onto BC axis (Dest → Ref)
        yC, xC = self.axis_c[0], self.axis_c[1] * self.lng_scale
        dx_bc, dy_bc = xC - xB, yC - yB
        dx_bp, dy_bp = xP - xB, yP - yB
        mag_bc = math.sqrt(dx_bc**2 + dy_bc**2)
        progress_bc = (dx_bp * dx_bc + dy_bp * dy_bc) / mag_bc if mag_bc > 0 else 0

        return progress_ab, progress_bc, side_val

    def _walk_fn(self, spot_coords):
        """
        Estimate walking time from spot to destination.

        Uses geodesic (real-world spherical) distance divided by 1.3 m/s
        (average pedestrian speed ~4.7 km/h).

        NOTE: Straight-line approximation.
        Real pedestrian routing via walk graph is planned but not yet implemented.

        Args:
            spot_coords: (lat, lon)

        Returns:
            float: estimated walk time in seconds
        """
        return geodesic(spot_coords, self.dest).meters / 1.3

    def get_state(self):
        """
        Build and return the state dict consumed by BaseAlgorithm.

        Returns:
            dict with keys:
                spots:              list of spot dicts
                start_node:         "start" (string key for grid.nodes)
                dest_coords:        (lat, lon) of destination
                destination_C:      (lat, lon) of ref point
                walk_fn:            callable(spot_coords) -> seconds
                grid:               mock object with .nodes["start"] and .nodes["ref"]
                topology:           list of {progress_ab, progress_bc, side} per spot
                start/dest/ref _progress_ab/_progress_bc: float projection values

        Note: drive_fn, G_aug, and exit_multiplier are added by app.py after this call.
        """
        return {
            "spots": self.spots,
            "start_node": "start",
            "dest_coords": self.dest,
            "destination_C": self.ref,
            "walk_fn": self._walk_fn,
            "grid": type("Mock", (), {"nodes": {"start": self.start, "ref": self.ref}}),
            "topology": self.spot_topology,
            "start_progress_ab": self.node_cache["start"]["progress_ab"],
            "dest_progress_ab": self.node_cache["dest"]["progress_ab"],
            "ref_progress_ab": self.node_cache["ref"]["progress_ab"],
            "start_progress_bc": self.node_cache["start"]["progress_bc"],
            "dest_progress_bc": self.node_cache["dest"]["progress_bc"],
            "ref_progress_bc": self.node_cache["ref"]["progress_bc"],
        }
