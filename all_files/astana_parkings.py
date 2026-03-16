import osmnx as ox
import folium

# 1. Центр Астаны
ASTANA_CENTER = (51.1282, 71.4304)

# 2. Загружаем парковки из OpenStreetMap
tags = {"amenity": "parking"}
gdf = ox.features_from_point(
    ASTANA_CENTER,
    tags=tags,
    dist=6000,  # радиус в метрах (6 км)
)

print(f"Найдено парковок: {len(gdf)}")

# 3. Создаём карту
m = folium.Map(location=ASTANA_CENTER, zoom_start=12)

# 4. Добавляем парковки точками
for _, row in gdf.iterrows():
    if row.geometry is None:
        continue

    if row.geometry.geom_type == "Point":
        lat, lon = row.geometry.y, row.geometry.x
    else:
        centroid = row.geometry.centroid
        lat, lon = centroid.y, centroid.x

    folium.CircleMarker(
        location=(lat, lon),
        radius=3,
        color="blue",
        fill=True,
        fill_opacity=0.7,
        popup=row.get("name", "Parking"),
    ).add_to(m)

# 5. Сохраняем карту
m.save("astana_parkings.html")
