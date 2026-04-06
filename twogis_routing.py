# """
# 2gis_routing.py
# ===============
# 2GIS Routing API integration.

# Provides get_route_2gis() which calls the 2GIS Routing API
# and returns the same dict format as routing.get_route():
#     {
#         path: [[lat, lon], ...],
#         travel_time: float (seconds),
#         turn_penalty: 0.0,  # 2GIS handles turns internally
#         nodes: [],          # no graph nodes, coordinate-based
#     }

# Falls back to None on failure so the caller can use OSM routing instead.

# Usage in app.py:
#     from 2gis_routing import get_route_2gis
#     route = get_route_2gis(origin, dest) or routing.get_route(origin, dest)
# """

# import re
# import requests

# TWOGIS_API_KEY = "51e15411-93a6-468f-8dae-ee1e9c95384d"
# TWOGIS_ROUTING_URL = "https://routing.api.2gis.com/carrouting/6.0.0/global"


# def _parse_wkt_linestring(wkt: str):
#     """
#     Parse WKT LINESTRING into list of [lat, lon] pairs.
#     Input:  "LINESTRING(lon1 lat1, lon2 lat2, ...)"
#     Output: [[lat1, lon1], [lat2, lon2], ...]
#     """
#     coords = []
#     # Extract the coordinate string inside LINESTRING(...)
#     match = re.search(r"LINESTRING\s*\((.+)\)", wkt, re.IGNORECASE)
#     if not match:
#         return coords
#     for pair in match.group(1).split(","):
#         parts = pair.strip().split()
#         if len(parts) >= 2:
#             lon, lat = float(parts[0]), float(parts[1])
#             coords.append([lat, lon])
#     return coords


# def get_route_2gis(origin_coords, dest_coords, route_type="jam"):
#     """
#     Get a car route from 2GIS Routing API.

#     Args:
#         origin_coords: (lat, lon) tuple
#         dest_coords:   (lat, lon) tuple
#         route_type:    "jam" (real-time traffic, default) or "shortest"

#     Returns:
#         dict with keys: path, travel_time, turn_penalty, nodes
#         or None if the API call fails
#     """
#     try:
#         payload = {
#             "locale": "ru",
#             "type": route_type,
#             "points": [
#                 {"type": "walking", "x": origin_coords[1], "y": origin_coords[0]},
#                 {"type": "walking", "x": dest_coords[1], "y": dest_coords[0]},
#             ],
#         }

#         response = requests.post(
#             TWOGIS_ROUTING_URL,
#             params={"key": TWOGIS_API_KEY},
#             json=payload,
#             timeout=8,
#         )

#         if response.status_code != 200:
#             print(f"[2GIS] HTTP {response.status_code}: {response.text[:200]}")
#             return None

#         data = response.json()

#         # The response contains a list of route objects
#         routes = data if isinstance(data, list) else data.get("result", [])
#         if not routes:
#             print("[2GIS] No routes returned")
#             return None

#         route = routes[0]

#         total_duration = float(route.get("total_duration", 0))
#         total_distance = float(route.get("total_distance", 0))

#         # Extract geometry from maneuvers
#         path_coords = []
#         maneuvers = route.get("maneuvers", [])

#         for maneuver in maneuvers:
#             outcoming = maneuver.get("outcoming_path", {})
#             geometry_list = outcoming.get("geometry", [])
#             for geom in geometry_list:
#                 selection = geom.get("selection", "")
#                 if selection:
#                     coords = _parse_wkt_linestring(selection)
#                     if path_coords and coords:
#                         # Avoid duplicating the junction point
#                         path_coords.extend(coords[1:])
#                     else:
#                         path_coords.extend(coords)

#         if not path_coords:
#             print("[2GIS] Empty geometry in response")
#             return None

#         return {
#             "path": path_coords,
#             "travel_time": total_duration,
#             "turn_penalty": 0.0,
#             "nodes": [],
#             "distance": total_distance,
#             "source": "2gis",
#         }

#     except requests.exceptions.Timeout:
#         print("[2GIS] Request timed out")
#         return None
#     except Exception as e:
#         print(f"[2GIS] Error: {e}")
#         return None


# def get_routes_2gis_both(origin_coords, dest_coords):
#     """
#     Get primary (jam) and alternative (shortest) routes from 2GIS.
#     Returns (primary, alternative) — either can be None on failure.
#     """
#     primary = get_route_2gis(origin_coords, dest_coords, route_type="jam")
#     alternative = get_route_2gis(origin_coords, dest_coords, route_type="shortest")
#     return primary, alternative
# """
# twogis_routing.py
# =================
# 2GIS Routing API integration.

# Returns a unified route dict:
#     {
#         "path": [[lat, lon], ...],
#         "travel_time": float,
#         "turn_penalty": 0.0,
#         "nodes": [],
#         "distance": float,
#         "source": "2gis",
#     }
# """
############# WORKING PART #######################
# import os
# import re
# from typing import List, Optional, Tuple, Dict, Any

# import requests

# TWOGIS_API_KEY = os.getenv("TWOGIS_API_KEY", "51e15411-93a6-468f-8dae-ee1e9c95384d")
# TWOGIS_ROUTING_URL = "https://routing.api.2gis.com/carrouting/6.0.0/global"


# def _parse_wkt_linestring(wkt: str) -> List[List[float]]:
#     """
#     Parse WKT LINESTRING into [[lat, lon], ...].

#     Accepts:
#         LINESTRING(lon lat, lon lat, ...)
#         LINESTRING(lon lat z, lon lat z, ...)
#     """
#     coords: List[List[float]] = []

#     if not isinstance(wkt, str):
#         return coords

#     match = re.search(r"LINESTRING\s*\((.+)\)", wkt, re.IGNORECASE)
#     if not match:
#         return coords

#     for pair in match.group(1).split(","):
#         parts = pair.strip().split()
#         if len(parts) >= 2:
#             lon = float(parts[0])
#             lat = float(parts[1])
#             coords.append([lat, lon])

#     return coords


# def _dedupe_path(coords: List[List[float]]) -> List[List[float]]:
#     if not coords:
#         return coords

#     out = [coords[0]]
#     for p in coords[1:]:
#         if p != out[-1]:
#             out.append(p)
#     return out


# def get_route_2gis(
#     origin_coords: Tuple[float, float],
#     dest_coords: Tuple[float, float],
#     route_type: str = "jam",
#     timeout: int = 8,
# ) -> Optional[Dict[str, Any]]:
#     """
#     Get a car route from 2GIS.

#     Args:
#         origin_coords: (lat, lon)
#         dest_coords: (lat, lon)
#         route_type: "jam" or "shortest"

#     Returns:
#         route dict or None on failure
#     """
#     try:
#         payload = {
#             "locale": "ru",
#             "type": route_type,
#             "points": [
#                 {"type": "walking", "x": origin_coords[1], "y": origin_coords[0]},
#                 {"type": "walking", "x": dest_coords[1], "y": dest_coords[0]},
#             ],
#         }

#         response = requests.post(
#             TWOGIS_ROUTING_URL,
#             params={"key": TWOGIS_API_KEY},
#             json=payload,
#             timeout=timeout,
#         )

#         if response.status_code != 200:
#             print(f"[2GIS] HTTP {response.status_code}: {response.text[:200]}")
#             return None

#         data = response.json()

#         routes = data if isinstance(data, list) else data.get("result", [])
#         if not routes:
#             print("[2GIS] No routes returned")
#             return None

#         route = routes[0]

#         total_duration = float(route.get("total_duration", 0.0))
#         total_distance = float(route.get("total_distance", 0.0))

#         path_coords: List[List[float]] = []
#         maneuvers = route.get("maneuvers", [])

#         for maneuver in maneuvers:
#             outcoming = maneuver.get("outcoming_path", {})
#             geometry_list = outcoming.get("geometry", [])

#             for geom in geometry_list:
#                 selection = geom.get("selection", "")
#                 if not selection:
#                     continue

#                 coords = _parse_wkt_linestring(selection)
#                 if not coords:
#                     continue

#                 if path_coords:
#                     path_coords.extend(
#                         coords[1:] if coords[0] == path_coords[-1] else coords
#                     )
#                 else:
#                     path_coords.extend(coords)

#         path_coords = _dedupe_path(path_coords)

#         if len(path_coords) < 2:
#             print("[2GIS] Empty or too-short geometry in response")
#             return None

#         return {
#             "path": path_coords,
#             "travel_time": total_duration,
#             "turn_penalty": 0.0,
#             "nodes": [],
#             "distance": total_distance,
#             "source": "2gis",
#         }

#     except requests.exceptions.Timeout:
#         print("[2GIS] Request timed out")
#         return None
#     except Exception as e:
#         print(f"[2GIS] Error: {e}")
#         return None


# def get_routes_2gis_both(
#     origin_coords: Tuple[float, float],
#     dest_coords: Tuple[float, float],
# ):
#     primary = get_route_2gis(origin_coords, dest_coords, route_type="jam")
#     alternative = get_route_2gis(origin_coords, dest_coords, route_type="shortest")
#     return primary, alternative
########################## WORKING PART ##########################

# """
# twogis_routing.py
# =================
# 2GIS Routing API wrapper.

# Returns same dict format as routing.get_route():
#     {
#         "path": [[lat, lon], ...],
#         "travel_time": float (seconds),
#         "turn_penalty": float,
#         "nodes": [],
#         "source": "2gis"
#     }

# Returns None on any failure so caller can fallback to OSM.
# """

# import re
# import requests

# TWOGIS_API_KEY = "51e15411-93a6-468f-8dae-ee1e9c95384d"
# TWOGIS_ROUTING_URL = "https://routing.api.2gis.com/carrouting/6.0.0/global"


# def _parse_wkt_linestring(wkt: str):
#     """
#     Parse WKT LINESTRING into [[lat, lon], ...].
#     Input:  "LINESTRING(lon1 lat1, lon2 lat2, ...)"
#     Output: [[lat1, lon1], [lat2, lon2], ...]
#     """
#     coords = []
#     match = re.search(r"LINESTRING\s*\((.+)\)", wkt, re.IGNORECASE)
#     if not match:
#         return coords
#     for pair in match.group(1).split(","):
#         parts = pair.strip().split()
#         if len(parts) >= 2:
#             try:
#                 lon, lat = float(parts[0]), float(parts[1])
#                 coords.append([lat, lon])
#             except ValueError:
#                 continue
#     return coords


# def get_route_2gis(origin_coords, dest_coords, route_type="jam"):
#     """
#     Get a car route from 2GIS Routing API.

#     Args:
#         origin_coords: (lat, lon)
#         dest_coords:   (lat, lon)
#         route_type:    "jam" (real-time traffic) or "shortest"

#     Returns:
#         dict with path, travel_time, turn_penalty, nodes
#         or None on failure
#     """
#     try:
#         payload = {
#             "locale": "ru",
#             "type": route_type,
#             "points": [
#                 {
#                     "type": "walking",
#                     "x": float(origin_coords[1]),
#                     "y": float(origin_coords[0]),
#                 },
#                 {
#                     "type": "walking",
#                     "x": float(dest_coords[1]),
#                     "y": float(dest_coords[0]),
#                 },
#             ],
#         }

#         response = requests.post(
#             TWOGIS_ROUTING_URL,
#             params={"key": TWOGIS_API_KEY},
#             json=payload,
#             timeout=8,
#         )

#         if response.status_code != 200:
#             print(f"[2GIS] HTTP {response.status_code}: {response.text[:200]}")
#             return None

#         data = response.json()

#         # Response is a list of route objects
#         routes = data if isinstance(data, list) else data.get("result", [])
#         if not routes:
#             print("[2GIS] No routes returned")
#             return None

#         route = routes[0]
#         total_duration = float(route.get("total_duration", 0))
#         total_distance = float(route.get("total_distance", 0))

#         # Extract geometry from maneuvers
#         path_coords = []
#         for maneuver in route.get("maneuvers", []):
#             for geom in maneuver.get("outcoming_path", {}).get("geometry", []):
#                 selection = geom.get("selection", "")
#                 if not selection:
#                     continue
#                 coords = _parse_wkt_linestring(selection)
#                 if path_coords and coords:
#                     path_coords.extend(coords[1:])  # skip duplicate junction
#                 else:
#                     path_coords.extend(coords)

#         if not path_coords:
#             print("[2GIS] Empty geometry in response")
#             return None

#         return {
#             "path": path_coords,
#             "travel_time": total_duration,
#             "turn_penalty": 0.0,
#             "nodes": [],
#             "distance": total_distance,
#             "source": "2gis",
#         }

#     except requests.exceptions.Timeout:
#         print(f"[2GIS] Timeout routing {origin_coords} → {dest_coords}")
#         return None
#     except Exception as e:
#         print(f"[2GIS] Error: {e}")
#         return None

"""
twogis_routing.py  (OSRM drop-in replacement)
==============================================
Replaces the 2GIS Routing API with OSRM, which is free and has no request limits.

Two backend options — set OSRM_BASE_URL to switch:
  - Public demo server (no setup, good for testing):
        http://router.project-osrm.org
  - Local server (unlimited, production-ready):
        http://localhost:5000

The function signatures and return dicts are identical to the original
twogis_routing.py, so nothing else in app.py needs to change.

OSRM route endpoint used:
    GET /route/v1/driving/{lon1},{lat1};{lon2},{lat2}
        ?overview=full&geometries=geojson&steps=false

Dependencies: requests (already in your project)
"""

# import requests
# from typing import List, Optional, Tuple, Dict, Any

# # ── Configuration ──────────────────────────────────────────────────────────────

# # Option A: public OSRM demo server — no setup needed, use for testing
# OSRM_BASE_URL = "http://router.project-osrm.org"

# # Option B: your own local OSRM instance (run via Docker — see README below)
# # OSRM_BASE_URL = "http://localhost:5000"

# # ── Helpers ────────────────────────────────────────────────────────────────────


# def _dedupe_path(coords: List[List[float]]) -> List[List[float]]:
#     """Remove consecutive duplicate [lat, lon] points."""
#     if not coords:
#         return coords
#     out = [coords[0]]
#     for p in coords[1:]:
#         if p != out[-1]:
#             out.append(p)
#     return out


# def _geojson_to_latlon(geometry: dict) -> List[List[float]]:
#     """
#     Convert GeoJSON LineString geometry to [[lat, lon], ...].
#     OSRM returns [lon, lat] pairs — we flip them.
#     """
#     coords = []
#     for lon, lat in geometry.get("coordinates", []):
#         coords.append([lat, lon])
#     return coords


# # ── Main routing function ──────────────────────────────────────────────────────


# def get_route_osrm(
#     origin_coords: Tuple[float, float],
#     dest_coords: Tuple[float, float],
#     profile: str = "driving",
#     timeout: int = 8,
# ) -> Optional[Dict[str, Any]]:
#     """
#     Get a car route from OSRM.

#     Args:
#         origin_coords: (lat, lon)
#         dest_coords:   (lat, lon)
#         profile:       "driving" | "walking" | "cycling"
#         timeout:       seconds before giving up

#     Returns:
#         dict with keys matching the original twogis_routing return format:
#             path:         [[lat, lon], ...]  — for map polyline display
#             travel_time:  float (seconds)
#             turn_penalty: float — always 0.0 (OSRM folds this into travel_time)
#             nodes:        []    — OSRM doesn't expose graph node ids
#             distance:     float (meters)
#             source:       "osrm"
#         Returns None on failure.
#     """
#     try:
#         # OSRM expects lon,lat order in the URL
#         coords_str = (
#             f"{origin_coords[1]},{origin_coords[0]};{dest_coords[1]},{dest_coords[0]}"
#         )
#         url = f"{OSRM_BASE_URL}/route/v1/{profile}/{coords_str}"

#         params = {
#             "overview": "full",
#             "geometries": "geojson",
#             "steps": "false",
#         }

#         response = requests.get(url, params=params, timeout=timeout)

#         if response.status_code != 200:
#             print(f"[OSRM] HTTP {response.status_code}: {response.text[:200]}")
#             return None

#         data = response.json()

#         if data.get("code") != "Ok" or not data.get("routes"):
#             print(f"[OSRM] No routes returned: {data.get('code')}")
#             return None

#         route = data["routes"][0]
#         total_duration = float(route.get("duration", 0.0))  # seconds
#         total_distance = float(route.get("distance", 0.0))  # meters

#         path_coords = _geojson_to_latlon(route.get("geometry", {}))
#         path_coords = _dedupe_path(path_coords)

#         if len(path_coords) < 2:
#             print("[OSRM] Empty or too-short geometry in response")
#             return None

#         return {
#             "path": path_coords,
#             "travel_time": total_duration,
#             "turn_penalty": 0.0,
#             "nodes": [],
#             "distance": total_distance,
#             "source": "osrm",
#         }

#     except requests.exceptions.Timeout:
#         print("[OSRM] Request timed out")
#         return None
#     except Exception as e:
#         print(f"[OSRM] Error: {e}")
#         return None


# # ── Public API — matches original twogis_routing.py exactly ───────────────────


# def get_route_2gis(
#     origin_coords: Tuple[float, float],
#     dest_coords: Tuple[float, float],
#     route_type: str = "jam",  # ignored — kept for interface compatibility
#     timeout: int = 8,
# ) -> Optional[Dict[str, Any]]:
#     """
#     Drop-in replacement for the original get_route_2gis().
#     The route_type argument is accepted but ignored — OSRM always
#     returns a time-optimal driving route.
#     """
#     return get_route_osrm(origin_coords, dest_coords, timeout=timeout)


# def get_routes_2gis_both(
#     origin_coords: Tuple[float, float],
#     dest_coords: Tuple[float, float],
# ) -> Tuple[Optional[Dict], Optional[Dict]]:
#     """
#     Drop-in replacement for get_routes_2gis_both().
#     Returns (primary, alternative) — both are OSRM driving routes.
#     For a real alternative we request the route twice; OSRM doesn't
#     expose alternative routes in a single call on the public server,
#     so both results will be identical. If you run a local OSRM instance
#     you can enable alternatives=true in the request.
#     """
#     primary = get_route_osrm(origin_coords, dest_coords)
#     alternative = get_route_osrm(origin_coords, dest_coords)
#     return primary, alternative


# ── README: running a local OSRM server ───────────────────────────────────────
#
# For a fully self-hosted, unlimited instance:
#
# 1. Download Astana/Kazakhstan OSM extract:
#    wget https://download.geofabrik.de/asia/kazakhstan-latest.osm.pbf
#
# 2. Run OSRM via Docker (one-time preprocessing, then serve):
#    docker run -t -v $(pwd):/data osrm/osrm-backend osrm-extract \
#        -p /opt/car.lua /data/kazakhstan-latest.osm.pbf
#    docker run -t -v $(pwd):/data osrm/osrm-backend osrm-partition \
#        /data/kazakhstan-latest.osrm
#    docker run -t -v $(pwd):/data osrm/osrm-backend osrm-customize \
#        /data/kazakhstan-latest.osrm
#    docker run -t -i -p 5000:5000 -v $(pwd):/data osrm/osrm-backend \
#        osrm-routed --algorithm mld /data/kazakhstan-latest.osrm
#
# 3. Set OSRM_BASE_URL = "http://localhost:5000" at the top of this file.
#
# Preprocessing takes ~5-10 min on a modern laptop. After that, routing is
# instant and unlimited with no internet required.

"""
twogis_routing.py  (OSRM — display + optimization)
===================================================
Replaces 2GIS Routing API with OSRM.
Used for BOTH:
  1. Optimization — drive_fn gets travel_time from OSRM duration
  2. Display      — map polylines come from OSRM geometry

Backend options (set OSRM_BASE_URL):
  Public demo  : http://router.project-osrm.org   (no setup, use for testing)
  Local server : http://localhost:5000             (unlimited, run via Docker)

Docker setup (one time, ~10 min):
  wget https://download.geofabrik.de/asia/kazakhstan-latest.osm.pbf
  docker run -t -v $(pwd):/data osrm/osrm-backend osrm-extract -p /opt/car.lua /data/kazakhstan-latest.osm.pbf
  docker run -t -v $(pwd):/data osrm/osrm-backend osrm-partition /data/kazakhstan-latest.osrm
  docker run -t -v $(pwd):/data osrm/osrm-backend osrm-customize /data/kazakhstan-latest.osrm
  docker run -t -i -p 5000:5000 -v $(pwd):/data osrm/osrm-backend osrm-routed --algorithm mld /data/kazakhstan-latest.osrm
  Then set OSRM_BASE_URL = "http://localhost:5000"
"""

import requests
from typing import List, Optional, Tuple, Dict, Any

# OSRM_BASE_URL = "http://router.project-osrm.org"
OSRM_BASE_URL = "http://localhost:5001"


def _dedupe_path(coords: List[List[float]]) -> List[List[float]]:
    if not coords:
        return coords
    out = [coords[0]]
    for p in coords[1:]:
        if p != out[-1]:
            out.append(p)
    return out


def _geojson_to_latlon(geometry: dict) -> List[List[float]]:
    """OSRM returns [lon, lat] — flip to [lat, lon]."""
    return [[lat, lon] for lon, lat in geometry.get("coordinates", [])]


def get_route_osrm(
    origin_coords: Tuple[float, float],
    dest_coords: Tuple[float, float],
    profile: str = "driving",
    timeout: int = 8,
) -> Optional[Dict[str, Any]]:
    """
    Call OSRM and return route info used for BOTH optimization and display.

    Args:
        origin_coords: (lat, lon)
        dest_coords:   (lat, lon)
        profile:       "driving" | "walking" | "cycling"
        timeout:       seconds

    Returns:
        {
            path:         [[lat, lon], ...]   — full road geometry for map display
            travel_time:  float (seconds)     — OSRM duration, used as base cost
            turn_penalty: float               — always 0.0 (folded into OSRM duration)
            distance:     float (meters)      — road distance
            nodes:        []                  — not available from OSRM HTTP API
            source:       "osrm"
        }
        Returns None on failure.
    """
    try:
        coords_str = (
            f"{origin_coords[1]},{origin_coords[0]};{dest_coords[1]},{dest_coords[0]}"
        )
        url = f"{OSRM_BASE_URL}/route/v1/{profile}/{coords_str}"
        params = {
            "overview": "full",
            "geometries": "geojson",
            "steps": "false",
        }

        response = requests.get(url, params=params, timeout=timeout)

        if response.status_code != 200:
            print(f"[OSRM] HTTP {response.status_code}: {response.text[:200]}")
            return None

        data = response.json()

        if data.get("code") != "Ok" or not data.get("routes"):
            print(f"[OSRM] No routes: {data.get('code')}")
            return None

        route = data["routes"][0]
        path_coords = _dedupe_path(_geojson_to_latlon(route.get("geometry", {})))

        if len(path_coords) < 2:
            print("[OSRM] Geometry too short")
            return None

        return {
            "path": path_coords,
            "travel_time": float(route.get("duration", 0.0)),
            "turn_penalty": 0.0,
            "distance": float(route.get("distance", 0.0)),
            "nodes": [],
            "source": "osrm",
        }

    except requests.exceptions.Timeout:
        print("[OSRM] Timed out")
        return None
    except Exception as e:
        print(f"[OSRM] Error: {e}")
        return None


def get_route_2gis(
    origin_coords: Tuple[float, float],
    dest_coords: Tuple[float, float],
    route_type: str = "jam",
    timeout: int = 8,
) -> Optional[Dict[str, Any]]:
    """Drop-in replacement — route_type kept for interface compatibility."""
    return get_route_osrm(origin_coords, dest_coords, timeout=timeout)


def get_routes_2gis_both(
    origin_coords: Tuple[float, float],
    dest_coords: Tuple[float, float],
):
    primary = get_route_osrm(origin_coords, dest_coords)
    alternative = get_route_osrm(origin_coords, dest_coords)
    return primary, alternative
