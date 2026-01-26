import json
import numpy as np

def print_stats(name, data):
    if not data:
        print(f"\n--- {name} ---")
        print("No data found for this category.")
        return

    dist_array = np.array(data)
    
    # Percentiles
    p0 = np.min(dist_array)
    p25 = np.percentile(dist_array, 25)
    p50 = np.percentile(dist_array, 50)
    p75 = np.percentile(dist_array, 75)
    p100 = np.max(dist_array)

    # Moments
    mean = np.mean(dist_array)
    variance = np.var(dist_array)
    std_dev = np.std(dist_array)

    print(f"\n{'='*10} {name} {'='*10}")
    print(f"Sample Count: {len(data)}")
    print(f"Mean:         {mean:.6f}")
    print(f"Std Dev:      {std_dev:.6f}")
    print(f"Variance:     {variance:.6f}")
    
    print(f"\nPercentiles:")
    print(f"  0% (Min):  {p0:.6e}")
    print(f" 25% (Q1):   {p25:.6e}")
    print(f" 50% (Med):  {p50:.6e}")
    print(f" 75% (Q3):   {p75:.6e}")
    print(f"100% (Max):  {p100:.6e}")

    # Visual Histogram
    bins = 10
    counts, bin_edges = np.histogram(dist_array, bins=bins)
    max_count = max(counts) if any(counts) else 1
    print("\nRough Distribution:")
    for i in range(bins):
        bar = "█" * int((counts[i] / max_count) * 20)
        print(f"{bin_edges[i]:10.4f} | {bar} ({counts[i]})")

def analyze_cache(cache_path):
    try:
        with open(cache_path, 'r') as f:
            cache = json.load(f)
    except Exception as e:
        print(f"Error: {e}")
        return

    # Categorization buckets
    spot_to_coord = []
    spot_to_spot = []
    coord_to_spot = []

    for key, dist in cache.items():
        # Check for Spot to Coord: spot_manual_..._to_coord_...
        if key.startswith("spot_manual_") and "_to_coord_" in key:
            spot_to_coord.append(dist)
        
        # Check for Spot to Spot: spot_manual_..._to_spot_manual_...
        elif key.startswith("spot_manual_") and "_to_spot_manual_" in key:
            spot_to_spot.append(dist)
            
        # Check for Coord to Spot: coord_..._to_spot_manual_...
        elif key.startswith("coord_") and "_to_spot_manual_" in key:
            coord_to_spot.append(dist)

    print_stats("SPOT TO COORD", spot_to_coord)
    print_stats("SPOT TO SPOT", spot_to_spot)
    print_stats("COORD TO SPOT", coord_to_spot)

if __name__ == "__main__":
    analyze_cache("distance_cache.json")