uint64 timestamp # time since system start (microseconds)

float32 lookahead_distance       # [m] Lookahead distance of pure the pursuit controller
float32 desired_heading          # [rad] Desired heading calculated by the pure pursuit controller
float32 crosstrack_error         # [m] Shortest distance from the vehicle to the path
float32 distance_to_waypoint     # [m] Distance from the vehicle to the current waypoint
float32 distance_on_line_segment # [m] Projection of the vector from the previous waypoint to current position onto the vector from previous to current waypoint
float32 heading_error            # [rad] Difference between actual and desired heading

# TOPICS pure_pursuit
