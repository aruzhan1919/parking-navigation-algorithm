from flask import Flask, request, jsonify, render_template
import os, time, json, math
import numpy as np
from jinja2 import FileSystemLoader
import routing
from scenarios import SCENARIOS
from locations import LOCATIONS
from transformation import StateAdapter
from algorithms import FiniteHorizonMDP, HeuristicLookahead, MDP_Difference

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
STATIC_DIR = os.path.join(BASE_DIR, 'static')
MANUAL_FILE = os.path.join(BASE_DIR, "manual_spots.json")
DISTANCE_CACHE_FILE = os.path.join(BASE_DIR, "distance_cache.json")

if not os.path.exists(STATIC_DIR): os.makedirs(STATIC_DIR, exist_ok=True)

app = Flask(__name__, static_folder=STATIC_DIR, template_folder=BASE_DIR)
app.jinja_loader = FileSystemLoader(BASE_DIR)
routing.initialize_graph()

def load_json(path, default):
    if os.path.exists(path):
        try:
            with open(path, 'r') as f: return json.load(f)
        except: return default
    return default

def save_json(path, data):
    with open(path, 'w') as f: json.dump(data, f)

MANUAL_SPOTS = load_json(MANUAL_FILE, [])
DISTANCE_CACHE = load_json(DISTANCE_CACHE_FILE, {})

@app.route('/')
def index(): return render_template('index.html')

@app.route('/get_locations')
def get_locations(): return jsonify(LOCATIONS)

@app.route('/get_manual_spots')
def get_manual(): return jsonify(MANUAL_SPOTS)

@app.route('/add_manual_spot', methods=['POST'])
def add_manual_spot():
    data = request.json
    spot = {
        'id': f"manual_{int(time.time()*1000)}",
        'coords': (data['lat'], data['lng']),
        'street': "Manual Entry",
        'phi_exit_seconds': int(data.get('phi', 60)),
        'p_i': float(data.get('pi', 0.8))
    }
    MANUAL_SPOTS.append(spot)
    save_json(MANUAL_FILE, MANUAL_SPOTS)
    return jsonify({'status': 'success', 'spot': spot})

@app.route('/delete_spot', methods=['POST'])
def delete_spot():
    spot_id = request.json.get('id')
    global MANUAL_SPOTS
    MANUAL_SPOTS = [s for s in MANUAL_SPOTS if s['id'] != spot_id]
    save_json(MANUAL_FILE, MANUAL_SPOTS)
    return jsonify({'status': 'success'})

@app.route('/update_spot', methods=['POST'])
def update_spot():
    data = request.json
    for s in MANUAL_SPOTS:
        if s['id'] == data.get('id'):
            s['phi_exit_seconds'] = int(data.get('phi', 60))
            s['p_i'] = float(data.get('pi', 0.8))
            break
    save_json(MANUAL_FILE, MANUAL_SPOTS)
    return jsonify({'status': 'success'})

def parse_coord(c):
    if isinstance(c, (list, tuple)) and len(c) >= 2: return (float(c[0]), float(c[1]))
    return (0.0, 0.0)

@app.route('/solve', methods=['POST'])
def solve():
    data = request.json
    start = parse_coord(data.get('start'))
    dest = parse_coord(data.get('dest'))
    ref = parse_coord(data.get('ref'))
    exit_multiplier = float(data.get('exit_multiplier', 1.0))

    if not MANUAL_SPOTS: return jsonify({'status': 'error', 'message': 'No spots defined.'})

    # 1. Setup Augmented Routing
    G_aug = routing.augment_graph_with_spots(MANUAL_SPOTS)
    
    # 2. Resolve Scenario & Parameters
    scenario_key = data.get('scenario', 'Short Stay (10-40m)')
    scenario_cfg = SCENARIOS.get(scenario_key, SCENARIOS["Short Stay (10-40m)"])
    params = scenario_cfg['params'].copy()
    traffic_mult = params.get('traffic_multiplier', 1.4)
    
    if scenario_key == 'Custom':
        custom = data.get('custom_params', {})
        params = {
            'lambda_w': custom.get('lambda_w', 1.5),
            'lambda_e': custom.get('lambda_e', 1.0),
            'lambda_tr': custom.get('lambda_tr', 1.0),
            'k': custom.get('k', 5)
        }

    # 3. Cached Augmented Router
    def drive_fn(u, v):
        u_id = f"spot_{u[1]['id']}" if u[0] == 'spot' else f"coord_{start[0]}_{start[1]}"
        v_id = f"spot_{v[1]['id']}" if v[0] == 'spot' else f"coord_{ref[0]}_{ref[1]}"
        cache_key = f"{u_id}_to_{v_id}_{traffic_mult}"
        if cache_key in DISTANCE_CACHE: return float(DISTANCE_CACHE[cache_key])
        u_t = f"spot_{u[1]['id']}" if u[0] == 'spot' else start
        v_t = f"spot_{v[1]['id']}" if v[0] == 'spot' else ref
        res = routing.get_route(u_t, v_t, custom_G=G_aug)
        final_time = res['travel_time'] * traffic_mult
        DISTANCE_CACHE[cache_key] = final_time
        return final_time

    adapter = StateAdapter(start, dest, ref, MANUAL_SPOTS, traffic_multiplier=traffic_mult)
    state = adapter.get_state()
    state['drive_fn'] = drive_fn
    state['exit_multiplier'] = exit_multiplier
    
    # Choose Solver
    algo_choice = data.get('algo', 'MDP_Difference')
    solver_class = MDP_Difference if algo_choice == 'MDP_Difference' else (FiniteHorizonMDP if algo_choice == 'MDP' else HeuristicLookahead)
    solver = solver_class(state)
    chain, metrics, _ = solver.solve(**params)
    
    if not chain: return jsonify({'status': 'error', 'message': 'Pathfinding failed.'})

    # 4. Persistence & UI Breakdown
    save_json(DISTANCE_CACHE_FILE, DISTANCE_CACHE)
    
    segments = []
    details = []
    curr_origin_id = start
    
    for i, idx in enumerate(chain):
        spot = state['spots'][idx]
        spot_node_id = f"spot_{spot['id']}"
        
        # Physical drive for this leg
        res = routing.get_route(curr_origin_id, spot_node_id, custom_G=G_aug)
        leg_drive_time = res['travel_time'] * traffic_mult
        
        # Search penalty from the PREVIOUS failed attempt
        phi_prev = state['spots'][chain[i-1]]['phi_exit_seconds'] if i > 0 else 0
        
        # Transition = Drive + Phi_prev
        total_arrival_time = leg_drive_time + phi_prev
        
        segments.append({'coords': res['path'], 'type': 'drive_transition', 'label': spot['street']})
        
        details.append({
            'order': i + 1,
            'street': spot.get('street', 'Spot'),
            'coords': spot['coords'],
            'p': spot.get('p_i', 0.8),
            'phi': spot.get('phi_exit_seconds', 60),
            'drive_leg': leg_drive_time,
            'phi_prev': phi_prev,
            'arrival_time': total_arrival_time,
            'walk_b': state['walk_fn'](spot['coords']),
            'exit_ref': state['drive_fn'](('spot', spot), ('node', 'ref'))
        })
        curr_origin_id = spot_node_id

    # Add final exit segments
    last_spot_id = f"spot_{state['spots'][chain[-1]]['id']}"
    res_ex = routing.get_route(last_spot_id, ref, custom_G=G_aug)
    segments.append({'coords': res_ex['path'], 'type': 'drive_exit'})
    segments.append({'coords': [state['spots'][chain[-1]]['coords'], dest], 'type': 'walk'})

    return jsonify({
        'status': 'success',
        'metrics': list(metrics) + [1.0 - np.prod([1-state['spots'][idx]['p_i'] for idx in chain])],
        'details': details,
        'segments': segments
    })
if __name__ == '__main__':
    app.run(debug=True, port=5000)