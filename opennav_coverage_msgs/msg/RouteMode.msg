string mode "UNKNOWN"  # BOUSTROPHEDON, SNAKE, SPIRAL, CUSTOM, TSP

# Specific mode setting
uint16 spiral_n 4  # If mode=SPIRAL, this is the number of swaths to spiral
uint16[] custom_order  # If mode=CUSTOM, this is the order of swaths to set. Must be specified.

# TSP (OR-Tools) knobs — only used when mode=TSP
bool tsp_redirect_swaths true    # Allow RoutePlannerBase to reverse swath direction for shorter turns
uint16 tsp_time_limit 1          # OR-Tools time limit in seconds (genRoute time_limit_seconds)
bool tsp_search_for_optimum false  # Enable guided local search (slower but better quality)
float64 tsp_d_tol 0.0001         # Distance tolerance for OR-Tools (genRoute d_tol)
