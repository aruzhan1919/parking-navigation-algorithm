import networkx as nx
import osmnx as ox
import os
import pickle
import math
from functools import lru_cache
from shapely.geometry import Point
from shapely.ops import nearest_points

# ==========================================
# CONFIGURATION
# ==========================================
PLACE_NAME = "Astana, Kazakhstan"
GRAPH_FILENAME = "astana_drive.graphml"

# PENALTIES (Seconds)
COST_U_TURN = 300.0   
COST_LEFT = 45.0      
COST_RIGHT = 10.0     
COST_STRAIGHT = 0.0

G = None      
G_TURN = None 

def initialize_graph():
    global G, G_TURN
    if G is not None: return
    if os.path.exists(GRAPH_FILENAME):
        G = ox.load_graphml(GRAPH_FILENAME)
    else:
        G = ox.graph_from_place(PLACE_NAME, network_type='drive')
        ox.save_graphml(G, GRAPH_FILENAME)
    
    # Add travel times if missing
    G = ox.add_edge_speeds(G)
    G = ox.add_edge_travel_times(G)

def augment_graph_with_spots(spots):
    """
    Realizes the 'Split-Edge' theory. 
    Transforms the graph by injecting spot-nodes into existing edges.
    """
    global G
    initialize_graph()
    
    # Work on a copy to keep the base graph clean
    G_aug = G.copy()
    
    # 1. Find nearest edges for all spots
    lats = [s['coords'][0] for s in spots]
    lngs = [s['coords'][1] for s in spots]
    nearest_edges = ox.nearest_edges(G_aug, lngs, lats)
    
    # 2. Group spots by the edge they belong to
    edge_to_spots = {}
    for i, edge in enumerate(nearest_edges):
        if edge not in edge_to_spots: edge_to_spots[edge] = []
        edge_to_spots[edge].append(spots[i])
        
    for (u, v, k), spot_list in edge_to_spots.items():
        edge_data = G_aug.get_edge_data(u, v, k)
        if not edge_data: continue
        
        edge_geom = edge_data.get('geometry', None)
        total_len = edge_data['length']
        total_time = edge_data['travel_time']
        
        # Calculate distance along edge for each spot
        spot_offsets = []
        for s in spot_list:
            p = Point(s['coords'][1], s['coords'][0])
            if edge_geom:
                dist_along = edge_geom.project(p)
            else:
                # Fallback to simple linear interp if no geometry
                dist_along = 0 # Simplified
            spot_offsets.append((s['id'], dist_along, s['coords']))
            
        # Sort spots by distance along the edge
        spot_offsets.sort(key=lambda x: x[1])
        
        # 3. Split the edge
        curr_u = u
        last_offset = 0
        
        for spot_id, offset, coords in spot_offsets:
            new_node = f"spot_{spot_id}"
            G_aug.add_node(new_node, x=coords[1], y=coords[0])
            
            # Calculate segment metrics
            seg_len = offset - last_offset
            ratio = seg_len / total_len if total_len > 0 else 0
            seg_time = total_time * ratio
            
            # Add the segment from current position to this spot
            G_aug.add_edge(curr_u, new_node, length=seg_len, travel_time=seg_time, highway=edge_data.get('highway'))
            
            curr_u = new_node
            last_offset = offset
            
        # Final segment to the original end intersection
        final_len = total_len - last_offset
        ratio = final_len / total_len if total_len > 0 else 0
        G_aug.add_edge(curr_u, v, length=final_len, travel_time=total_time * ratio, highway=edge_data.get('highway'))
        
        # Remove the original long edge
        G_aug.remove_edge(u, v, k)
        
    return G_aug

def get_route(u_coords, v_coords, custom_G=None):
    """Modified to use Augmented nodes if IDs are provided"""
    target_G = custom_G if custom_G else G
    
    # If coords are actually IDs (e.g., 'spot_manual_123')
    u_node = u_coords if isinstance(u_coords, str) else ox.nearest_nodes(target_G, u_coords[1], u_coords[0])
    v_node = v_coords if isinstance(v_coords, str) else ox.nearest_nodes(target_G, v_coords[1], v_coords[0])
    
    try:
        path = nx.shortest_path(target_G, u_node, v_node, weight='travel_time')
        path_coords = [[target_G.nodes[n]['y'], target_G.nodes[n]['x']] for n in path]
        
        # Calculate total time
        time = 0
        for i in range(len(path)-1):
            edge_data = target_G.get_edge_data(path[i], path[i+1])
            # Handle multigraph return (dict of keys)
            if isinstance(edge_data, dict):
                edge_data = list(edge_data.values())[0]
            time += edge_data.get('travel_time', 1.0)
            
        return {'path': path_coords, 'travel_time': time}
    except:
        return {'path': [], 'travel_time': float('inf')}