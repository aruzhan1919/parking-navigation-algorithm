import json
import os
import osmnx as ox

# куда сохраняем
OUTPUT_FILE = "parking_spots.json"

# центр Астаны (или любой)
ASTANA_CENTER = (51.1284, 71.4306)


def load_osm_parking_spots(center, radius=6000):
    """
    Загружает ВСЕ парковки из OSM и приводит к формату manual spots
    """
    tags = {"amenity": "parking"}
    gdf = ox.features_from_point(center, tags=tags, dist=radius)

    spots = []

    for idx, row in gdf.iterrows():
        if row.geometry is None:
            continue

        if row.geometry.geom_type == "Point":
            lat, lon = row.geometry.y, row.geometry.x
        else:
            centroid = row.geometry.centroid
            lat, lon = centroid.y, centroid.x

        import pandas as pd

        def safe_value(val, default):
            return default if pd.isna(val) else val

        spot = {
            "id": f"osm_{idx}",
            "coords": [lat, lon],
            "street": safe_value(row.get("name"), "OSM Parking"),
            "type": safe_value(row.get("parking"), "surface"),
            "is_manual": False,
            "p_i": 0.5,
            "phi_exit_seconds": 0,
        }

        spots.append(spot)

    return spots


if __name__ == "__main__":
    spots = load_osm_parking_spots(ASTANA_CENTER, radius=6000)

    with open(OUTPUT_FILE, "w") as f:
        json.dump(spots, f, indent=2)

    print(f"Saved {len(spots)} parking spots to {OUTPUT_FILE}")
