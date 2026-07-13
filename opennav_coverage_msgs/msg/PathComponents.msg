# An ordered set of swaths (if route generated) and turns (if path generated)
std_msgs/Header header
opennav_coverage_msgs/Swath[] swaths
nav_msgs/Path[] turns
bool contains_turns
bool swaths_ordered

# Per-pose velocity (m/s) and direction flags, parallel to nav_path.poses.
# Populated only when generate_path=true. Empty when path is not generated.
float64[] velocities
bool[] is_backward
