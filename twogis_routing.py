"""
2GIS Routing API client (Navigation / Routing 7.0 global).

Pair with routing.get_route(..., twogis_display=True) for map legs only; use
twogis_display=False inside drive_fn / MDP (see TWOGIS_MODE=display_only).

Trial / rate-limit friendly:
  - In-process LRU cache for identical (or nearly identical) OD pairs
  - Minimum interval between real HTTP calls (TWOGIS_MIN_REQUEST_INTERVAL)
  - Circuit breaker: after HTTP 429, skip API calls for TWOGIS_COOLDOWN_AFTER_429 seconds

Environment:
  TWOGIS_API_KEY                 — API key
  TWOGIS_ROUTING_URL             — optional base URL override
  TWOGIS_TRAFFIC_MODE            — optional e.g. "jam"
  TWOGIS_MIN_REQUEST_INTERVAL    — seconds between requests (default 1.0, safer for trial)
  begin_solve_request/end_solve_request — wrap each /solve; after first 429, skip further
      HTTP in that solve (Option 1 legs). Set TWOGIS_MAP_OPTION2=true to 2GIS-map Option 2 too.
  TWOGIS_COOLDOWN_AFTER_429      — seconds to stop calling API after 429 (default 120)
  TWOGIS_CACHE_MAX               — max cached routes (default 2000)
  TWOGIS_SINGLE_CHAIN_REQUEST    — app build_solution: one multi-waypoint 2GIS call per solve (default true)
  TWOGIS_MAX_WAYPOINT_SPOTS      — max parking stops in chain for that call (default 8, 2GIS pref limit)

routing.py TWOGIS_MODE:
  display_only (default) — 2GIS only when get_route(..., twogis_display=True)
  all_legs — try 2GIS on every get_route (high API usage)

Docs: https://docs.2gis.com/en/api/navigation/routing/overview
"""

from __future__ import annotations

import copy
import os
import threading
import time
from collections import OrderedDict
from typing import Any, List, Optional, Tuple, Union

import requests
from geopy.distance import geodesic
from shapely import wkt as wkt_module

TWOGIS_DEFAULT_URL = "https://routing.api.2gis.com/routing/7.0.0/global"

_lock = threading.Lock()
# LRU: key is OD tuple or ("chain", rounded waypoints) -> successful route dict
_cache: "OrderedDict[Union[Tuple[float, float, float, float], Tuple[str, Tuple[Tuple[float, float], ...]]], dict]" = OrderedDict()
_last_http_monotonic: float = 0.0
_circuit_open_until: float = 0.0
_last_429_log_time: float = 0.0
_last_cooldown_log_time: float = 0.0
# After first 429 inside one Flask /solve, skip further HTTP (still use cache hits).
_solve_batch_suppress: bool = False


def begin_solve_request() -> None:
    """Call at start of POST /solve. Resets per-request 429 burst guard."""
    global _solve_batch_suppress
    with _lock:
        _solve_batch_suppress = False


def end_solve_request() -> None:
    """Call at end of POST /solve (finally block)."""
    global _solve_batch_suppress
    with _lock:
        _solve_batch_suppress = False


def reset_batch_suppress_only() -> None:
    """Clear per-request burst guard (e.g. start of Option 2 map fetch). Circuit stays as-is."""
    global _solve_batch_suppress
    with _lock:
        _solve_batch_suppress = False


def _collect_wkt_strings(obj: Any, out: List[str]) -> None:
    if isinstance(obj, str) and "LINESTRING" in obj.upper():
        out.append(obj)
    elif isinstance(obj, dict):
        for v in obj.values():
            _collect_wkt_strings(v, out)
    elif isinstance(obj, list):
        for item in obj:
            _collect_wkt_strings(item, out)


def _wkt_to_latlon_path(wkt_str: str) -> List[List[float]]:
    g = wkt_module.loads(wkt_str)
    if g.geom_type == "LineString":
        coords = list(g.coords)
    elif g.geom_type == "MultiLineString":
        coords = []
        for line in g.geoms:
            coords.extend(line.coords)
    else:
        return []
    return [[float(y), float(x)] for x, y in coords]


def _merge_paths(paths: List[List[List[float]]]) -> List[List[float]]:
    if not paths:
        return []
    merged: List[List[float]] = list(paths[0])
    for nxt in paths[1:]:
        if not nxt:
            continue
        if merged and nxt[0] == merged[-1]:
            merged.extend(nxt[1:])
        else:
            merged.extend(nxt)
    return merged


def _duration_from_route(route_obj: dict) -> Optional[float]:
    for key in ("total_duration", "duration", "time"):
        v = route_obj.get(key)
        if v is not None:
            return float(v)
    tot = route_obj.get("total")
    if isinstance(tot, dict):
        for key in ("duration", "time"):
            v = tot.get(key)
            if v is not None:
                return float(v)
    maneuvers = route_obj.get("maneuvers") or []
    s = 0.0
    found = False
    for step in maneuvers:
        if not isinstance(step, dict):
            continue
        d = step.get("duration") or step.get("time")
        if d is not None:
            s += float(d)
            found = True
    return s if found else None


def _distance_from_route(route_obj: dict) -> Optional[float]:
    for key in ("total_distance", "distance", "length"):
        v = route_obj.get(key)
        if v is not None:
            return float(v)
    tot = route_obj.get("total")
    if isinstance(tot, dict):
        for key in ("distance", "length"):
            v = tot.get(key)
            if v is not None:
                return float(v)
    return None


def _od_key(lat1: float, lon1: float, lat2: float, lon2: float) -> Tuple[float, float, float, float]:
    return (
        round(lat1, 5),
        round(lon1, 5),
        round(lat2, 5),
        round(lon2, 5),
    )


def _min_interval() -> float:
    return max(0.05, float(os.getenv("TWOGIS_MIN_REQUEST_INTERVAL", "1.0")))


def _cooldown_after_429() -> float:
    return max(5.0, float(os.getenv("TWOGIS_COOLDOWN_AFTER_429", "120")))


def _cache_max() -> int:
    return max(100, int(os.getenv("TWOGIS_CACHE_MAX", "2000")))


def _parse_routing_response_body(data: Any) -> Optional[dict]:
    """Build {path, duration_sec, distance_m} from 2GIS routing JSON or None."""
    routes = data.get("result")
    if not isinstance(routes, list) or not routes:
        meta = data.get("meta") or data.get("error")
        print(f"[2GIS] no result in response: {meta or data}")
        return None

    route0 = routes[0]
    if not isinstance(route0, dict):
        return None

    duration = _duration_from_route(route0)
    distance = _distance_from_route(route0)

    wkts: List[str] = []
    _collect_wkt_strings(route0.get("geometry"), wkts)
    if not wkts:
        _collect_wkt_strings(route0, wkts)

    segment_paths: List[List[List[float]]] = []
    for w in wkts:
        try:
            segment_paths.append(_wkt_to_latlon_path(w))
        except Exception:
            continue

    path = _merge_paths(segment_paths)
    if len(path) < 2:
        print("[2GIS] could not parse route geometry from response")
        return None

    if duration is None:
        duration = 0.0

    return {
        "path": path,
        "duration_sec": max(0.0, float(duration)),
        "distance_m": distance,
    }


def split_polyline_at_waypoints(
    path: List[List[float]],
    waypoints_lat_lon: List[Tuple[float, float]],
) -> List[List[List[float]]]:
    """
    Split a full route polyline into consecutive legs between waypoints (lat, lon).
    Indices advance monotonically along the path.
    """
    if len(waypoints_lat_lon) < 2 or len(path) < 2:
        return []

    idxs: List[int] = []
    lo_idx = 0
    for w in waypoints_lat_lon:
        la, lo = float(w[0]), float(w[1])
        best_j = lo_idx
        best_d = float("inf")
        for j in range(lo_idx, len(path)):
            d = geodesic((la, lo), (path[j][0], path[j][1])).meters
            if d < best_d:
                best_d = d
                best_j = j
        idxs.append(best_j)
        lo_idx = best_j

    out: List[List[List[float]]] = []
    for i in range(len(idxs) - 1):
        a, b = idxs[i], idxs[i + 1]
        if b < a:
            b = a
        seg = path[a : b + 1]
        if len(seg) < 2 and len(path) >= 2:
            nxt = min(b + 1, len(path) - 1)
            seg = [path[a], path[nxt]]
        out.append(seg)
    return out


def _throttle_http():
    """Space out HTTP calls; do not call while holding _lock (uses lock briefly + sleep outside)."""
    global _last_http_monotonic
    gap = _min_interval()
    with _lock:
        now = time.monotonic()
        wait = max(0.0, _last_http_monotonic + gap - now)
    if wait > 0:
        time.sleep(wait)
    with _lock:
        _last_http_monotonic = time.monotonic()


def request_driving_route(
    lat1: float,
    lon1: float,
    lat2: float,
    lon2: float,
    api_key: Optional[str] = None,
    timeout: float = 20.0,
) -> Optional[dict]:
    global _circuit_open_until, _last_429_log_time, _last_cooldown_log_time, _solve_batch_suppress

    key = (api_key or os.getenv("TWOGIS_API_KEY", "") or "").strip()
    if not key:
        return None

    ck = _od_key(lat1, lon1, lat2, lon2)

    with _lock:
        if ck in _cache:
            _cache.move_to_end(ck)
            return copy.deepcopy(_cache[ck])

        if _solve_batch_suppress:
            return None

        now_wall = time.time()
        if now_wall < _circuit_open_until:
            if now_wall - _last_cooldown_log_time > 30.0:
                _last_cooldown_log_time = now_wall
                remaining = int(_circuit_open_until - now_wall)
                print(
                    f"[2GIS] circuit open (trial rate limit): using internal routing "
                    f"~{remaining}s remaining"
                )
            return None

    base_url = (os.getenv("TWOGIS_ROUTING_URL") or TWOGIS_DEFAULT_URL).strip()
    traffic = (os.getenv("TWOGIS_TRAFFIC_MODE") or "").strip()

    payload: dict = {
        "points": [
            {"type": "stop", "lon": float(lon1), "lat": float(lat1)},
            {"type": "stop", "lon": float(lon2), "lat": float(lat2)},
        ],
        "transport": "driving",
        "route_mode": "fastest",
        "locale": "en",
    }
    if traffic:
        payload["traffic_mode"] = traffic

    url = f"{base_url.rstrip('/')}?key={key}"

    _throttle_http()

    try:
        r = requests.post(
            url,
            json=payload,
            headers={"Content-Type": "application/json"},
            timeout=timeout,
        )
    except requests.RequestException as e:
        print(f"[2GIS] request error: {e}")
        return None

    if r.status_code == 429:
        cd = _cooldown_after_429()
        with _lock:
            _circuit_open_until = time.time() + cd
            _solve_batch_suppress = True
            now_l = time.time()
            if now_l - _last_429_log_time > 5.0:
                _last_429_log_time = now_l
                print(
                    f"[2GIS] HTTP 429 (rate limit). Pausing API calls for {int(cd)}s; "
                    f"using internal OSM routing until then. "
                    f"Skipping further 2GIS HTTP for this /solve request."
                )
        return None

    if r.status_code != 200:
        print(f"[2GIS] HTTP {r.status_code}: {r.text[:500]}")
        return None

    with _lock:
        _circuit_open_until = 0.0

    try:
        data = r.json()
    except ValueError:
        print("[2GIS] invalid JSON response")
        return None

    out = _parse_routing_response_body(data)
    if not out:
        return None

    with _lock:
        max_sz = _cache_max()
        while len(_cache) >= max_sz:
            _cache.popitem(last=False)
        _cache[ck] = copy.deepcopy(out)

    return out


def _chain_cache_key(latlon_points: List[Tuple[float, float]]) -> Tuple[str, Tuple[Tuple[float, float], ...]]:
    rounded = tuple(
        (round(float(la), 5), round(float(lo), 5)) for la, lo in latlon_points
    )
    return ("chain", rounded)


def request_route_through_waypoints(
    latlon_points: List[Tuple[float, float]],
    api_key: Optional[str] = None,
    timeout: float = 30.0,
) -> Optional[dict]:
    """
    One HTTP request: drive through ordered waypoints (lat, lon).
    First and last points are type \"stop\"; inner points are \"pref\" (max 8 for driving per 2GIS).

    Returns same shape as request_driving_route: path, duration_sec, distance_m.
    """
    global _circuit_open_until, _last_429_log_time, _last_cooldown_log_time, _solve_batch_suppress

    key = (api_key or os.getenv("TWOGIS_API_KEY", "") or "").strip()
    if not key or len(latlon_points) < 2:
        return None

    ck = _chain_cache_key(latlon_points)

    with _lock:
        if ck in _cache:
            _cache.move_to_end(ck)
            return copy.deepcopy(_cache[ck])

        if _solve_batch_suppress:
            return None

        now_wall = time.time()
        if now_wall < _circuit_open_until:
            if now_wall - _last_cooldown_log_time > 30.0:
                _last_cooldown_log_time = now_wall
                remaining = int(_circuit_open_until - now_wall)
                print(
                    f"[2GIS] circuit open (trial rate limit): using internal routing "
                    f"~{remaining}s remaining"
                )
            return None

    base_url = (os.getenv("TWOGIS_ROUTING_URL") or TWOGIS_DEFAULT_URL).strip()
    traffic = (os.getenv("TWOGIS_TRAFFIC_MODE") or "").strip()

    pts_json: List[dict] = []
    n = len(latlon_points)
    for i, (la, lo) in enumerate(latlon_points):
        if i == 0 or i == n - 1:
            typ = "stop"
        else:
            typ = "pref"
        pts_json.append({"type": typ, "lon": float(lo), "lat": float(la)})

    payload: dict = {
        "points": pts_json,
        "transport": "driving",
        "route_mode": "fastest",
        "locale": "en",
    }
    if traffic:
        payload["traffic_mode"] = traffic

    url = f"{base_url.rstrip('/')}?key={key}"

    _throttle_http()

    try:
        r = requests.post(
            url,
            json=payload,
            headers={"Content-Type": "application/json"},
            timeout=timeout,
        )
    except requests.RequestException as e:
        print(f"[2GIS] chain request error: {e}")
        return None

    if r.status_code == 429:
        cd = _cooldown_after_429()
        with _lock:
            _circuit_open_until = time.time() + cd
            _solve_batch_suppress = True
            now_l = time.time()
            if now_l - _last_429_log_time > 5.0:
                _last_429_log_time = now_l
                print(
                    f"[2GIS] HTTP 429 (rate limit / quota). Pausing API for {int(cd)}s. "
                    "Trial keys allow very few requests per day — one chain call per solve is used. "
                    "If this persists, wait or set ROUTING_PROVIDER=internal."
                )
        return None

    if r.status_code != 200:
        print(f"[2GIS] chain HTTP {r.status_code}: {r.text[:500]}")
        return None

    with _lock:
        _circuit_open_until = 0.0

    try:
        data = r.json()
    except ValueError:
        print("[2GIS] invalid JSON response (chain)")
        return None

    out = _parse_routing_response_body(data)
    if not out:
        return None

    with _lock:
        max_sz = _cache_max()
        while len(_cache) >= max_sz:
            _cache.popitem(last=False)
        _cache[ck] = copy.deepcopy(out)

    return out
