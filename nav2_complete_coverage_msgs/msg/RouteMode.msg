string mode "UNKNOWN"  # BOUSTROPHEDON, SNAKE, SPIRAL, CUSTOM

# Specific mode setting
uint16 spiral_n 4  # If mode=SPIRAL, this is the number of swaths to spiral
uint16[] custom_order  # If mode=CUSTOM, this is the order of swaths to set. Must be specified.
