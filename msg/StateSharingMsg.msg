# Customized statesharing message
uint64 timestamp
uint64 timestamp_drone
uint16 frame_id

# global position coordinates
float32 global_position_lon
float32 global_position_lat
float32 global_position_alt

# cartesian angles based on front right down x,y,z axis
float32 roll
float32 pitch
float32 yaw
# quaternion
float32[4] q

# TOPICS incoming_state_sharing outgoing_state_sharing
