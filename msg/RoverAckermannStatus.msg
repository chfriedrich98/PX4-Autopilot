uint64 timestamp # time since system start (microseconds)

float32 throttle_setpoint 	      # [-1, 1] Normalized throttle setpoint
float32 steering_setpoint 	      # [-1, 1] Normalized steering setpoint
float32 actual_speed       	      # [m/s]   Rover ground speed
float32 lateral_acceleration          # [m/s^2] Lateral acceleration
float32 lateral_acceleration_setpoint # [m/s^2] Lateral acceleration setpoint

# TOPICS rover_ackermann_status
