# import routing
# import osmnx as ox

# # 1️⃣ Инициализация
# routing.initialize_graph()

# u = (51.1694, 71.4491)
# v = (51.1700, 71.4600)

# # 2️⃣ Маршрут с поворотами
# res_with_turns = routing.get_route(u, v)

# # 3️⃣ Отключаем повороты
# old_vals = (
#     routing.COST_LEFT,
#     routing.COST_RIGHT,
#     routing.COST_U_TURN,
#     routing.COST_STRAIGHT,
# )

# routing.COST_LEFT = 0
# routing.COST_RIGHT = 0
# routing.COST_U_TURN = 0
# routing.COST_STRAIGHT = 0

# res_no_turns = routing.get_route(u, v)

# # Вернуть штрафы
# (
#     routing.COST_LEFT,
#     routing.COST_RIGHT,
#     routing.COST_U_TURN,
#     routing.COST_STRAIGHT,
# ) = old_vals

# # 4️⃣ Вывести время
# print("Time without turns:", res_no_turns["travel_time"])
# print("Time with turns:", res_with_turns["travel_time"])

import routing

# 1️⃣ Initialize graph
routing.initialize_graph()

u = (51.133696570341655, 71.43163913937695)
v = (51.124765685067246, 71.43210200919789)

# -----------------------------
# WITH ALL PENALTIES
# -----------------------------
res_with = routing.get_route(u, v)

# -----------------------------
# DISABLE EVERYTHING
# -----------------------------
old_turn_vals = (
    routing.COST_LEFT,
    routing.COST_RIGHT,
    routing.COST_U_TURN,
    routing.COST_STRAIGHT,
)

routing.COST_LEFT = 0
routing.COST_RIGHT = 0
routing.COST_U_TURN = 0
routing.COST_STRAIGHT = 0

# Temporarily disable HARD intersections
old_hard = routing.HARD_INTERSECTIONS
routing.HARD_INTERSECTIONS = []

res_without = routing.get_route(u, v)

# -----------------------------
# Restore values
# -----------------------------
(
    routing.COST_LEFT,
    routing.COST_RIGHT,
    routing.COST_U_TURN,
    routing.COST_STRAIGHT,
) = old_turn_vals

routing.HARD_INTERSECTIONS = old_hard

# -----------------------------
# Print results
# -----------------------------
print("WITHOUT penalties:", res_without["travel_time"])
print("WITH penalties   :", res_with["travel_time"])
print("Penalty difference:", res_with["travel_time"] - res_without["travel_time"])

import routing
import folium

# 1️⃣ Initialize
routing.initialize_graph()

u = (51.133696570341655, 71.43163913937695)
v = (51.124765685067246, 71.43210200919789)

# 2️⃣ Get route
res = routing.get_route(u, v)

path = res["path"]

if not path:
    print("No route found")
else:
    # 3️⃣ Create map centered between points
    center_lat = (u[0] + v[0]) / 2
    center_lon = (u[1] + v[1]) / 2

    m = folium.Map(location=[center_lat, center_lon], zoom_start=14)

    # 4️⃣ Draw route
    folium.PolyLine(
        path,
        weight=5,
        color="blue",
        opacity=0.8,
    ).add_to(m)

    # 5️⃣ Start marker (green)
    folium.Marker(
        location=u,
        popup="Start",
        icon=folium.Icon(color="green", icon="play"),
    ).add_to(m)

    # 6️⃣ Destination marker (red)
    folium.Marker(
        location=v,
        popup="Destination",
        icon=folium.Icon(color="red", icon="stop"),
    ).add_to(m)

    # 7️⃣ Save
    m.save("route_map.html")

    print("Saved as route_map.html")
    print("Travel time:", res["travel_time"])
# import json
# import folium

# # Загрузка файла
# with open("manual_spots.json", "r") as f:
#     spots = json.load(f)

# # Центр карты (по первой точке)
# center = spots[0]["coords"]
# m = folium.Map(location=center, zoom_start=13)

# # for s in spots:
# #     lat, lon = s["coords"]
# #     phi = s.get("phi_exit_seconds", 0)
# #     p = s.get("p_i", 0)

# #     folium.CircleMarker(
# #         location=[lat, lon],
# #         radius=5,
# #         color="blue",
# #         fill=True,
# #         fill_opacity=0.8,
# #         tooltip=f"id={s['id']} | phi={phi} | p={p}",
# #     ).add_to(m)

# # m.save("all_spots.html")

# for s in spots:
#     lat, lon = s["coords"]
#     phi = s.get("phi_exit_seconds", 0)
#     p = s.get("p_i", 0.0)

#     # Цвет по phi (чтобы было наглядно)
#     if phi == 0:
#         color = "green"
#     elif phi <= 20:
#         color = "blue"
#     elif phi <= 120:
#         color = "orange"
#     else:
#         color = "red"

#     folium.CircleMarker(
#         location=[lat, lon],
#         radius=5,
#         color=color,
#         fill=True,
#         fill_opacity=0.8,
#         tooltip=f"phi={phi}, p={p}",
#     ).add_to(m)

# m.save("all_spots.html")


# def get_color(p):
#     if p >= 0.8:
#         return "red"
#     elif p >= 0.6:
#         return "orange"
#     elif p >= 0.5:
#         return "blue"
#     elif p >= 0.3:
#         return "purple"
#     else:
#         return "green"


# for s in spots:
#     lat, lon = s["coords"]
#     p = s.get("p_i", 0)

#     folium.CircleMarker(
#         location=[lat, lon],
#         radius=6,
#         color=get_color(p),
#         fill=True,
#         fill_opacity=0.9,
#         tooltip=f"id={s['id']} | p={p}",
#     ).add_to(m)

# m.save("spots_by_probability.html")


# import json

# # Paste your full list here
# data = [
#     {
#         "id": "manual_1768384179610",
#         "coords": [51.12041474774691, 71.42925381660463],
#         "street": "Manual Entry",
#         "type": "surface",
#         "is_manual": True,
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [0.0045344476166785915, -3.209893679861463e-06],
#     },
#     {
#         "id": "manual_1768384182890",
#         "coords": [51.11940795335722, 71.42885684967042],
#         "street": "Manual Entry",
#         "type": "surface",
#         "is_manual": True,
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [0.005507233990629037, -4.691724672881337e-06],
#     },
#     {
#         "id": "manual_1768384187875",
#         "coords": [51.118044200217014, 71.42830431461336],
#         "street": "Manual Entry",
#         "type": "surface",
#         "is_manual": True,
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [0.006830150086893394, -6.667237192280635e-06],
#     },
#     {
#         "id": "manual_1768384189872",
#         "coords": [51.11722929641127, 71.42799317836763],
#         "street": "Manual Entry",
#         "type": "surface",
#         "is_manual": True,
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [0.007613940347864912, -7.88838818829611e-06],
#     },
#     {
#         "id": "manual_1768384191421",
#         "coords": [51.117269705203135, 71.42817556858064],
#         "street": "Manual Entry",
#         "type": "surface",
#         "is_manual": True,
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [0.007516184335227768, -8.184895490460848e-06],
#     },
#     {
#         "id": "manual_1768384195714",
#         "coords": [51.11851562560814, 71.42866373062135],
#         "street": "Manual Entry",
#         "type": "surface",
#         "is_manual": True,
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [0.0063134389496859325, -6.3445045238529905e-06],
#     },
#     {
#         "id": "manual_1768384199143",
#         "coords": [51.11945172748259, 71.42902314662935],
#         "street": "Manual Entry",
#         "type": "surface",
#         "is_manual": True,
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [0.005412370685163895, -4.94602440997515e-06],
#     },
#     {
#         "id": "manual_1768384202824",
#         "coords": [51.120421482083685, 71.4293932914734],
#         "street": "Manual Entry",
#         "type": "surface",
#         "is_manual": True,
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [0.004479682341275779, -3.4925811271529115e-06],
#     },
#     {
#         "id": "manual_1768384206741",
#         "coords": [51.122601201530536, 71.43010883686753],
#         "street": "Manual Entry",
#         "type": "surface",
#         "is_manual": True,
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [0.0024243434627618035, 2.33207885684945e-08],
#     },
#     {
#         "id": "manual_1768384208692",
#         "coords": [51.12315002078036, 71.42834930775376],
#         "street": "Manual Entry",
#         "type": "surface",
#         "is_manual": True,
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [0.002591001107358756, 5.056751109884718e-06],
#     },
#     {
#         "id": "manual_1768384216386",
#         "coords": [51.12127000066144, 71.4258795976639],
#         "street": "Manual Entry",
#         "type": "surface",
#         "is_manual": True,
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [0.005017165045791852, 5.9860832096036615e-06],
#     },
#     {
#         "id": "manual_1768384217908",
#         "coords": [51.12115888602022, 71.42568647861482],
#         "street": "Manual Entry",
#         "type": "surface",
#         "is_manual": True,
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [0.00517718962617838, 6.141847658369578e-06],
#     },
#     {
#         "id": "manual_1768384219876",
#         "coords": [51.119990482551806, 71.42522513866426],
#         "street": "Manual Entry",
#         "type": "surface",
#         "is_manual": True,
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [0.006306356310950683, 4.423550862502635e-06],
#     },
#     {
#         "id": "manual_1768384220976",
#         "coords": [51.11994334173409, 71.42536461353303],
#         "street": "Manual Entry",
#         "type": "surface",
#         "is_manual": True,
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [0.00629615386658112, 4.016139952995243e-06],
#     },
#     {
#         "id": "manual_1768384223069",
#         "coords": [51.11886245693172, 71.42477989196779],
#         "street": "Manual Entry",
#         "type": "surface",
#         "is_manual": True,
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [0.00739644811813604, 2.7643139807975885e-06],
#     },
#     {
#         "id": "manual_1768384224108",
#         "coords": [51.11879511124737, 71.42493546009065],
#         "street": "Manual Entry",
#         "type": "surface",
#         "is_manual": True,
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [0.007397281785490731, 2.275711212011243e-06],
#     },
#     {
#         "id": "manual_1768384226659",
#         "coords": [51.11806103692094, 71.42443656921388],
#         "street": "Manual Entry",
#         "type": "surface",
#         "is_manual": True,
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [0.008180437944031121, 1.6432119169025892e-06],
#     },
#     {
#         "id": "manual_1768384227813",
#         "coords": [51.11776807739692, 71.42454385757448],
#         "street": "Manual Entry",
#         "type": "surface",
#         "is_manual": True,
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [0.008384917114085434, 7.355527147878978e-07],
#     },
#     {
#         "id": "manual_1768384241442",
#         "coords": [51.119131838687586, 71.42489254474641],
#         "street": "Manual Entry",
#         "type": "surface",
#         "is_manual": True,
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [0.007133894615639144, 3.1470285021286736e-06],
#     },
#     {
#         "id": "manual_1768384251542",
#         "coords": [51.119764879631184, 71.42514467239381],
#         "street": "Manual Entry",
#         "type": "surface",
#         "is_manual": True,
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [0.006521345467135765, 4.0733530334710274e-06],
#     },
#     {
#         "id": "manual_1768384254923",
#         "coords": [51.12153600129262, 71.4258259534836],
#         "street": "Manual Entry",
#         "type": "surface",
#         "is_manual": True,
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [0.004816063780489439, 6.716609123693801e-06],
#     },
#     {
#         "id": "manual_1768384261994",
#         "coords": [51.121219494039465, 71.42973124980928],
#         "street": "Manual Entry",
#         "type": "surface",
#         "is_manual": True,
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [0.0037004036020814464, -2.367896646407985e-06],
#     },
#     {
#         "id": "manual_1768384272526",
#         "coords": [51.122640392595514, 71.43173754215242],
#         "street": "Manual Entry",
#         "type": "surface",
#         "is_manual": True,
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [0.0018174567309636638, -3.3690617591384216e-06],
#     },
#     {
#         "id": "manual_1768384274659",
#         "coords": [51.12233736102492, 71.43374919891359],
#         "street": "Manual Entry",
#         "type": "surface",
#         "is_manual": True,
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [0.001358566453929998, -8.372676679310364e-06],
#     },
#     {
#         "id": "manual_1768384276724",
#         "coords": [51.12262692456791, 71.43323957920076],
#         "street": "Manual Entry",
#         "type": "surface",
#         "is_manual": True,
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [0.0012988048341548535, -6.61246275211259e-06],
#     },
#     {
#         "id": "manual_1768384277701",
#         "coords": [51.122650493613634, 71.43302500247957],
#         "street": "Manual Entry",
#         "type": "surface",
#         "is_manual": True,
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [0.0013549942671689413, -6.099010485181262e-06],
#     },
#     {
#         "id": "manual_1768384279456",
#         "coords": [51.12275823766946, 71.43230617046358],
#         "street": "Manual Entry",
#         "type": "surface",
#         "is_manual": True,
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [0.0015194170797540576, -4.31230041867556e-06],
#     },
#     {
#         "id": "manual_1768384284772",
#         "coords": [51.124004010063615, 71.43083095550539],
#         "street": "Manual Entry",
#         "type": "surface",
#         "is_manual": True,
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [0.001009307917382886, 1.7265807940109618e-06],
#     },
#     {
#         "id": "manual_1768384293375",
#         "coords": [51.124488842129416, 71.42701685428621],
#         "street": "Manual Entry",
#         "type": "surface",
#         "is_manual": True,
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [0.0019535722950348527, 1.1005738494919622e-05],
#     },
#     {
#         "id": "manual_1768384294860",
#         "coords": [51.12498713644982, 71.42721533775331],
#         "street": "Manual Entry",
#         "type": "surface",
#         "is_manual": True,
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [0.0014713999503760192, 1.1734840602240118e-05],
#     },
#     {
#         "id": "manual_1768384328377",
#         "coords": [51.12388280125201, 71.42512857913972],
#         "street": "Manual Entry",
#         "type": "surface",
#         "is_manual": True,
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [0.0031208831702999123, 1.3640947774493837e-05],
#     },
#     {
#         "id": "manual_1768384329993",
#         "coords": [51.123791894434596, 71.42424881458284],
#         "street": "Manual Entry",
#         "type": "surface",
#         "is_manual": True,
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [0.0035063834675274487, 1.531193855761593e-05],
#     },
#     {
#         "id": "manual_1768384331892",
#         "coords": [51.124128585455196, 71.42350852489473],
#         "street": "Manual Entry",
#         "type": "surface",
#         "is_manual": True,
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [0.003489001267734177, 1.7674560268918977e-05],
#     },
#     {
#         "id": "manual_1768384334230",
#         "coords": [51.123936671874205, 71.42328321933748],
#         "street": "Manual Entry",
#         "type": "surface",
#         "is_manual": True,
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [0.003727211371832444, 1.7712104779566106e-05],
#     },
#     {
#         "id": "manual_1768384338443",
#         "coords": [51.12553256055726, 71.42742455005647],
#         "street": "Manual Entry",
#         "type": "surface",
#         "is_manual": True,
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [0.0009464599765921424, 1.2550105893605606e-05],
#     },
#     {
#         "id": "manual_1768384344072",
#         "coords": [51.12642811950479, 71.4270329475403],
#         "street": "Manual Entry",
#         "type": "surface",
#         "is_manual": True,
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [0.0003438227793647493, 1.5460838210285653e-05],
#     },
#     {
#         "id": "manual_1768384345793",
#         "coords": [51.12629681683561, 71.42709732055665],
#         "street": "Manual Entry",
#         "type": "surface",
#         "is_manual": True,
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [0.00042972440450118155, 1.5019199835592284e-05],
#     },
#     {
#         "id": "manual_1768384350727",
#         "coords": [51.12540125534222, 71.43347561359407],
#         "street": "Manual Entry",
#         "type": "surface",
#         "is_manual": True,
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [-0.0010792357360240674, -6.945370475369584e-07],
#     },
#     {
#         "id": "manual_1768384364026",
#         "coords": [51.12497366910644, 71.43558382987977],
#         "street": "Manual Entry",
#         "type": "surface",
#         "is_manual": True,
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [-0.0014691586842251706, -6.193001728778663e-06],
#     },
#     {
#         "id": "manual_1768384365863",
#         "coords": [51.12529351745112, 71.43425345420839],
#         "street": "Manual Entry",
#         "type": "surface",
#         "is_manual": True,
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [-0.0012644769039310958, -2.607427273812312e-06],
#     },
#     {
#         "id": "manual_1768384371910",
#         "coords": [51.124131952353004, 71.4372092485428],
#         "street": "Manual Entry",
#         "type": "surface",
#         "is_manual": True,
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [-0.0013462433200937042, -1.1617697725618483e-05],
#     },
#     {
#         "id": "manual_1768384380475",
#         "coords": [51.12040127907043, 71.43580913543703],
#         "street": "Manual Entry",
#         "type": "surface",
#         "is_manual": True,
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [0.002233424683256049, -1.7260128387979976e-05],
#     },
#     {
#         "id": "manual_1768384382797",
#         "coords": [51.11946519643586, 71.4354979991913],
#         "street": "Manual Entry",
#         "type": "surface",
#         "is_manual": True,
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [0.0031174480057403384, -1.8761813949922117e-05],
#     },
#     {
#         "id": "manual_1768384385327",
#         "coords": [51.11855603327498, 71.43516540527345],
#         "street": "Manual Entry",
#         "type": "surface",
#         "is_manual": True,
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [0.003986773343852258, -2.0155290807204454e-05],
#     },
#     {
#         "id": "manual_1768896877867",
#         "coords": [51.12652912130398, 71.43185019493104],
#         "street": "Manual Entry",
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [-0.0014388394018947442, 5.39260753648753e-06],
#     },
#     {
#         "id": "manual_1768896889082",
#         "coords": [51.12914162443226, 71.43285870552064],
#         "street": "Manual Entry",
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [-0.003955488688002631, 9.283894963690033e-06],
#     },
#     {
#         "id": "manual_1768896891016",
#         "coords": [51.129195488921276, 71.43271923065187],
#         "street": "Manual Entry",
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [-0.003950847727982738, 9.706871481698732e-06],
#     },
#     {
#         "id": "manual_1768896902149",
#         "coords": [51.12815521986827, 71.43525660037996],
#         "street": "Manual Entry",
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [-0.003985359253757625, 1.87224019748708e-06],
#     },
#     {
#         "id": "manual_1768896904730",
#         "coords": [51.126879259164625, 71.435444355011],
#         "street": "Manual Entry",
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [-0.0029961724256588845, -1.4831952932074987e-06],
#     },
#     {
#         "id": "manual_1768896914699",
#         "coords": [51.125768235647996, 71.43783688545228],
#         "street": "Manual Entry",
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [-0.0029210723174747546, -9.171876449254246e-06],
#     },
#     {
#         "id": "manual_1768900136894",
#         "coords": [51.12134407694047, 71.42924308776857],
#         "street": "Manual Entry",
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [0.0037695372543585146, -1.0355093827127156e-06],
#     },
#     {
#         "id": "manual_1768905203748",
#         "coords": [51.125290150637966, 71.43352925777437],
#         "street": "Manual Entry",
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [-0.0010062566280554079, -1.0664717617700258e-06],
#     },
#     {
#         "id": "manual_1768905208846",
#         "coords": [51.125135276967946, 71.43456995487215],
#         "street": "Manual Entry",
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [-0.0012452230346883732, -3.6506223162371145e-06],
#     },
#     {
#         "id": "manual_1768980030811",
#         "coords": [51.12211513660978, 71.43006384372713],
#         "street": "Manual Entry",
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [0.0028422618197073996, -1.0057205785766476e-06],
#     },
#     {
#         "id": "manual_1768981547757",
#         "coords": [51.12552246016935, 71.43208622932436],
#         "street": "Manual Entry",
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [-0.0006894327250816215, 2.557362953181438e-06],
#     },
#     {
#         "id": "manual_1768981746718",
#         "coords": [51.125909640125236, 71.42958641052248],
#         "street": "Manual Entry",
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [-0.00012796338835564832, 8.799757372171324e-06],
#     },
#     {
#         "id": "manual_1770706639964",
#         "coords": [51.12391647039825, 71.40173435211183],
#         "street": "Manual Entry",
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [0.01134454374988118, 6.374924932196869e-05],
#     },
#     {
#         "id": "manual_1770706641764",
#         "coords": [51.12098042861288, 71.40100479125978],
#         "street": "Manual Entry",
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [0.014030418079957939, 5.851239951913216e-05],
#     },
#     {
#         "id": "manual_1770706642197",
#         "coords": [51.11322191569263, 71.40066146850587],
#         "street": "Manual Entry",
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [0.02056896691863085, 4.128530717234706e-05],
#     },
#     {
#         "id": "manual_1770706642663",
#         "coords": [51.11526942758351, 71.4188575744629],
#         "street": "Manual Entry",
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [0.012457316216696622, 7.111617692205785e-06],
#     },
#     {
#         "id": "manual_1770706643313",
#         "coords": [51.12959947058639, 71.40701293945314],
#         "street": "Manual Entry",
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [0.004782010091024231, 6.561699081901915e-05],
#     },
#     {
#         "id": "manual_1770706643763",
#         "coords": [51.129976516952254, 71.42460823059083],
#         "street": "Manual Entry",
#         "phi_exit_seconds": 0,
#         "p_i": 0.5,
#         "_cached_topo": [-0.0017359983548438642, 2.8860985521610627e-05],
#     },
# ]

# # Write to new file
# with open("manual_spots_new.json", "w", encoding="utf-8") as f:
#     json.dump(data, f, indent=2)

# print("manual_spots_new.json successfully created.")
# import json
# import folium

# # Load data
# with open("manual_spots_new.json", "r") as f:
#     spots = json.load(f)

# # Center map on Astana
# m = folium.Map(location=[51.12, 71.43], zoom_start=13)

# # Add all spots (same color)
# for spot in spots:
#     lat, lon = spot["coords"]

#     folium.CircleMarker(
#         location=[lat, lon],
#         radius=4,
#         color="blue",
#         fill=True,
#         fill_color="blue",
#         fill_opacity=0.8,
#         popup=spot["id"],
#     ).add_to(m)

# # Save map
# m.save("manual_spots_map.html")

# print("Map saved as manual_spots_map.html")
