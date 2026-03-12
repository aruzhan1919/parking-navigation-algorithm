import json
import osmnx as ox
import time

OUTPUT_FILE = "parking_spots.json"

# Center of Astana
ASTANA_CENTER = (51.1284, 71.4306)


def load_osm_parking_spots(center, radius=6000):
    """
    Load all OSM parking spots and convert to your format
    """
    tags = {"amenity": "parking"}
    gdf = ox.features_from_point(center, tags=tags, dist=radius)

    spots = []
    base_id = int(time.time() * 1000)

    for i, (_, row) in enumerate(gdf.iterrows()):
        if row.geometry is None:
            continue

        # Extract coordinates
        if row.geometry.geom_type == "Point":
            lat, lon = row.geometry.y, row.geometry.x
        else:
            centroid = row.geometry.centroid
            lat, lon = centroid.y, centroid.x

        spot = {
            "id": f"osm_{base_id + i}",
            "coords": [lat, lon],
            "street": row.get("name", "OSM Parking"),
            "type": row.get("parking", "surface"),
            "is_manual": False,
            "phi_exit_seconds": 0,
            "p_i": 0.5,
        }

        spots.append(spot)

    return spots


if __name__ == "__main__":
    spots = load_osm_parking_spots(ASTANA_CENTER, radius=6000)

    with open(OUTPUT_FILE, "w", encoding="utf-8") as f:
        json.dump(spots, f, indent=2, ensure_ascii=False)

    print(f"Saved {len(spots)} parking spots to {OUTPUT_FILE}")
