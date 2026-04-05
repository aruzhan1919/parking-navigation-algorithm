"""
2GIS Routing API client (Navigation / Routing 7.0 global).

Environment:
  TWOGIS_API_KEY       — API key (also loaded from .env via dotenv in routing.py)
  TWOGIS_ROUTING_URL   — optional base URL override
  TWOGIS_TRAFFIC_MODE  — optional e.g. "jam"; omit if your key rejects it

Docs: https://docs.2gis.com/en/api/navigation/routing/overview
"""

from __future__ import annotations

import os
from typing import Any, List, Optional

import requests
from shapely import wkt as wkt_module

TWOGIS_DEFAULT_URL = "https://routing.api.2gis.com/routing/7.0.0/global"


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


def request_driving_route(
    lat1: float,
    lon1: float,
    lat2: float,
    lon2: float,
    api_key: Optional[str] = None,
    timeout: float = 20.0,
) -> Optional[dict]:
    key = (api_key or os.getenv("TWOGIS_API_KEY", "") or "").strip()
    if not key:
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

    if r.status_code != 200:
        print(f"[2GIS] HTTP {r.status_code}: {r.text[:500]}")
        return None

    try:
        data = r.json()
    except ValueError:
        print("[2GIS] invalid JSON response")
        return None

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
