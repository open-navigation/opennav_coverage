string objective "UNKNOWN"  # LENGTH, NUMBER, or COVERAGE
string mode "UNKNOWN"  # BRUTE_FORCE, SET_ANGLE

# Specific mode settings
float32 best_angle 0.0  # If mode=SET_ANGLE for angle to generate swaths based on
float32 step_angle 1.7453e-2 # If mode=BRUTE_FORCE, angular step to search
