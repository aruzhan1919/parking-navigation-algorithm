import folium
import os
import routing

class MapVisualizer:
    def generate_map(self, state, chain, output_file="parking_solution.html"):
        start, dest, ref = state['grid'].nodes['start'], state['dest_coords'], state['grid'].nodes['ref']
        spots = state['spots']
        m = folium.Map(location=dest, zoom_start=15)
        
        # 1. Markers
        folium.Marker(start, popup="A (Start)", icon=folium.Icon(color='green')).add_to(m)
        folium.Marker(dest, popup="B (Dest)", icon=folium.Icon(color='red')).add_to(m)
        folium.Marker(ref, popup="C (Ref)", icon=folium.Icon(color='blue')).add_to(m)

        for i, spot in enumerate(spots):
            if i in chain:
                color = 'orange'
                opacity = 1.0
            else:
                color = 'purple' if spot.get('is_manual') else 'gray'
                opacity = 0.6
            
            folium.Marker(
                spot['coords'], 
                popup=f"{spot['street']} (P={spot['p_i']})", 
                icon=folium.Icon(color=color, icon='info-sign'),
                opacity=opacity
            ).add_to(m)

        # 2. Draw Paths
        if chain:
            # Start -> First
            first_spot = spots[chain[0]]
            self._draw_segment(m, start, first_spot['coords'], "blue", "Start -> 1")
            
            # Spot -> Spot
            for i in range(len(chain) - 1):
                u = spots[chain[i]]
                v = spots[chain[i+1]]
                self._draw_segment(m, u['coords'], v['coords'], "orange", f"{i+1} -> {i+2}")

            # Last -> Exit
            last_spot = spots[chain[-1]]
            self._draw_segment(m, last_spot['coords'], ref, "blue", "Exit", dash_array='5, 5')
            
            # Walk Path
            folium.PolyLine(
                [last_spot['coords'], dest], 
                color="red", weight=3, dash_array='5, 10', opacity=0.8, tooltip="Walk"
            ).add_to(m)

        m.save(output_file)

    def _draw_segment(self, m, start_coords, end_coords, color, tooltip, dash_array=None):
        route_data = routing.get_route(start_coords, end_coords)
        if route_data and route_data.get('path'):
            folium.PolyLine(
                route_data['path'], 
                color=color, 
                weight=5, 
                opacity=0.7, 
                tooltip=tooltip,
                dash_array=dash_array
            ).add_to(m)